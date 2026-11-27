/* Competition route. Odom is travel magnitude; each state has its own origin. */
#include "tune_params.h"
#include "fsm2.h"
#include "entry_wall.h"
#include "route_ranges.h"
#include "app_config.h"
#include "laser_wire.h"
#include "safety.h"
#include "bsp_types.h"
#include <math.h>
#include <stdlib.h>

static Fsm2State state;
static Fsm2Fault fault;
static EntryWall entry_wall;
static u32 start_ms, missing_ms;
#define REV_EARLIEST_EDGE_CM 32u
static u16 origin, edge_cm, first_match_cm;
static u8 turning, quiet_ticks, stopping;
static u16 stop_lead_mm;
static u8 endpoint_from_tail;
static u8 braking;
static u8 wait_reason, ridge_sum_warning;
static u8 entered, missing, votes, previous_seq, have_seq, saw_plate, saw_exit;

u8 Fsm2_TraceWait(void) { return missing ? wait_reason : 0; }

static const Fsm2Action actions[] = {
  F2A_STOP,F2A_FWD,F2A_CRAB_R,F2A_REV,F2A_CRAB_L,F2A_REV,
  F2A_CRAB_R,F2A_REV,F2A_CRAB_L,F2A_REV,F2A_CRAB_R,F2A_CRAB_L,
  F2A_REV,F2A_STOP,F2A_STOP
};

void Fsm2_Init(void) { state=F2_IDLE; fault=F2_OK; entered=0; }
void Fsm2_Start(void)
{
  if (state!=F2_IDLE && state!=F2_DONE && state!=F2_FAULT) return;
  state=F2_ENTRY; fault=F2_OK; entered=0;
  entry_wall=(EntryWall){0};
}
void Fsm2_Fail(Fsm2Fault reason)
{
  if (state!=F2_FAULT) fault=reason;
  state=F2_FAULT;
}
void Fsm2_Abort(void) { Fsm2_Fail(F2_ABORTED); }
Fsm2State Fsm2_GetState(void) { return state; }
u8 Fsm2_GetFaultCode(void) { return (u8)fault; }

/* Diagnostic bits only; all are sampled by the control task. */
u8 Fsm2_TraceFlags(void)
{
  return (entry_wall.acquired ? 1u : 0u) | (entry_wall.cruising ? 2u : 0u) |
         (saw_plate ? 4u : 0u) | (saw_exit ? 8u : 0u) | (missing ? 16u : 0u) |
         (ridge_sum_warning ? 32u : 0u) | (turning ? 64u : 0u) | (braking ? 128u : 0u);
}

static void enter(const AppInputs *in, u16 odom)
{
  turning=1; quiet_ticks=0; stopping=0; braking=0; stop_lead_mm=0;
  origin=odom; start_ms=in->ms; entered=1;
  missing=0; votes=0; have_seq=0; saw_plate=0; saw_exit=0;
}

/* All four replies must follow entry into this stage. Wheel encoders can move
   slightly while the steering axes swivel, so qualify standstill concurrently. */
static int ready_to_drive(const AppInputs *in, int target10)
{
  int quiet=1, aligned=1;
  for (unsigned i=0; i<4; ++i) {
    if (abs(in->wheel_cnt[i])>20) quiet=0;
    if ((s32)(in->steer_sample_ms[i]-start_ms)<=0 ||
        abs((int)in->steer_feedback10[i]-target10)>40) aligned=0;
  }
  if (!quiet) quiet_ticks=0;
  else if (quiet_ticks<3) quiet_ticks++;
  return aligned && quiet_ticks==3;
}

/* Track a distance-dependent velocity ceiling, then command zero PWM for the
   measured final coast. Only short-range, fresh returns enter this planner. */
static s16 approach_limit(float remaining_mm, int entry)
{
  float terminal=entry ? app_tune->entry_final_speed_mm_s : ROUTE_FINAL_SPEED_MM_S;
  float decel=entry ? app_tune->entry_decel_mm_s2 : ROUTE_DECEL_MM_S2;
  float v=sqrtf(terminal*terminal+2.0f*decel*fmaxf(0,remaining_mm));
  float counts=v*(ENCODER_CPR*APP_CTRL_MS/(3.14159265f*WHEEL_DIAMETER_MM*1000.0f));
  return (s16)fminf(counts,entry ? app_tune->route_entry_cruise : SHIFT_SPD);
}

/* Count independent sensor measurements; a repeated 50 Hz report has no vote. */
static int confirmed(int match, u8 seq, u16 cm)
{
  if (!match) { votes=0; return 0; }
  if (!have_seq || previous_seq!=seq) {
    if (!votes) first_match_cm=cm;
    previous_seq=seq; have_seq=1;
    if (votes<3) votes++;
  }
  return votes==3;
}

static void intent(Fsm2Output *out)
{
  out->action=actions[state];
  out->steer_deg=out->action==F2A_CRAB_R ? 90 : out->action==F2A_CRAB_L ? -90 : 0;
  out->wall_left=state==F2_ENTRY || state==F2_REV_23 || state==F2_REV_45 || state==F2_REV_PARK;
}

/* Missing samples brake immediately; a sustained loss requires a new start. */
static void wait_sensor(const AppInputs *in, Fsm2Output *out, u8 reason)
{
  wait_reason=reason;
  out->hold=1; votes=0;
  if (!missing) { missing=1; missing_ms=in->ms; }
  else if (in->ms-missing_ms>=400u) Fsm2_Fail(F2_MISSING_RANGE);
}

void Fsm2_Tick(const AppInputs *in, u16 odom, Fsm2Output *out)
{
  braking=0;
  ridge_sum_warning=0;
  *out=(Fsm2Output){0};
  intent(out);
  if (out->action==F2A_STOP) { out->hold=1; return; }
  if (!entered) enter(in,odom);
  u32 elapsed=in->ms-start_ms;
  u16 cm=(u16)(odom-origin);
  int complete=0;
  out->speed_limit=SHIFT_SPD;
  if (!in->laser_fresh) Fsm2_Fail(F2_LOST_LINK);
  else if (!in->imu_ok) Fsm2_Fail(F2_LOST_IMU);
  else if (elapsed>60000u) Fsm2_Fail(F2_TIMEOUT);
  else if (turning && !ready_to_drive(in,(int)(10*out->steer_deg))) {
    origin=odom;
    out->hold=1;
    if (elapsed>=ROUTE_TURN_TIMEOUT_MS) Fsm2_Fail(F2_TURN_TIMEOUT);
    else return;
  }
  else if (stopping) complete=1;
  else if (state==F2_ENTRY || out->action==F2A_CRAB_L || out->action==F2A_CRAB_R) {
    turning=0;
    int entry=state==F2_ENTRY;
    int right=out->action==F2A_CRAB_R;
    unsigned ch=entry ? LW_FRONT : right ? LW_RIGHT : LW_LEFT;
    /* Across the open ends of the ridges, a side beam may see a ridge end
       rather than the perimeter. Only use the alternate endpoint beam inside
       the mapped arrival window. */
    RouteRange endpoint_range = {in->dF, in->range_age[ch], in->sample_seq[ch], 0};
    if (!entry) endpoint_range = RouteRange_Side(in,ch,cm>=CRAB_TRAVEL_CM-35u);
    if (endpoint_from_tail != endpoint_range.from_tail) {
      votes=0; have_seq=0;
      endpoint_from_tail=endpoint_range.from_tail;
    }
    u16 d=endpoint_range.mm;
    u16 target=entry ? app_tune->entry_stop_mm : SIDE_CENTER_MM;
    u16 min_cm=entry ? (u16)app_tune->entry_min_cm : CRAB_TRAVEL_CM-35u;
    u16 max_cm=entry ? (u16)app_tune->entry_max_cm : CRAB_TRAVEL_CM+20u;
    /* Outside the two end aisles the forward/backward beams see the furrow.
       A single beam can guide when the other drops out. */
    out->track_ridge=!entry && cm>=35u && cm<=CRAB_TRAVEL_CM-35u;
    float ridge_error;
    RidgeStatus ridge = !entry ? RouteRange_Ridge(in,&ridge_error) : RIDGE_MISSING;
    /* A consistent pair identifies the ridge even at the open ends. */
    if (ridge == RIDGE_PAIR) out->track_ridge=1;
    ridge_sum_warning = ridge == RIDGE_DISAGREE;
    if (entry) {
      EntryWallCommand wall=EntryWall_Update(&entry_wall,in,cm,app_tune->route_entry_search,
                                             app_tune->route_entry_cruise);
      out->forward_speed=wall.speed;
      out->hold=wall.coast;
      out->steer_deg=wall.angle;
      out->steer_trim_deg=wall.yaw_trim_deg;
    }
    if (cm>max_cm || (d<=target && cm<min_cm)) Fsm2_Fail(F2_BAD_LANDMARK);
    else if (d==BSP_DIST_INVALID) wait_sensor(in,out,F2W_ENDPOINT);
    /* A zero wall-loop speed means "lost the rail" only while the car has not yet
       covered the minimum entry distance.  Past that point the same zero appears
       naturally as the car brakes to a stop on the endpoint, and treating it as
       a lost rail faulted the run at the instant of arrival (observed: dF=217,
       V=0, then F2_MISSING_RANGE). */
    else if (entry && !out->forward_speed && cm < min_cm) wait_sensor(in,out,F2W_ENTRY_WALL);
    else if (out->track_ridge && ridge == RIDGE_MISSING) wait_sensor(in,out,F2W_NO_RIDGE);
    else {
      missing=0;
      float v=in->wheel_speed_mm_s;
      float lag_mm=v*(endpoint_range.age+APP_CTRL_MS)/1000.0f;
      float coast=entry ? app_tune->entry_final_coast_mm : ROUTE_FINAL_COAST_MM;
      float lag_ms=entry ? app_tune->entry_drive_lag_ms : ROUTE_DRIVE_LAG_MS;
      float terminal=entry ? app_tune->entry_final_speed_mm_s : ROUTE_FINAL_SPEED_MM_S;
      float remaining=(float)d-target-lag_mm-coast;
      out->speed_limit=approach_limit(remaining-v*lag_ms/1000.0f,entry);
      /* Keep the odometric approach fallback when the endpoint is not visible. */
      if (cm>=max_cm-45u && d>target+300u)
        out->speed_limit=entry ? app_tune->entry_creep_counts : ROUTE_CREEP_COUNTS;
      u16 lead=(u16)fminf(coast*v/terminal,coast);
      lead+=(u16)fminf(lag_mm,12.0f);
      if (!votes) stop_lead_mm=lead;
      int endpoint=cm>=min_cm && d<=target+stop_lead_mm;
      complete=confirmed(endpoint,endpoint_range.seq,cm);
      out->hold|=endpoint;
      /* The velocity ceiling drives the PI throughout approach. Only endpoint
         arrival requests zero PWM; cruise overshoot must not reset the PI. */
      braking=endpoint;
    }
  } else if (state==F2_REV_PARK) {
    turning=0;
    out->speed_limit=ROUTE_CREEP_COUNTS;
    /* The outside wall can end at the mouth of the start box. Check the
       initial alignment while still in the aisle, then finish by odometry. */
    RouteRange park_wall=RouteRange_Side(in,LW_LEFT,1);
    if (cm<10u && (park_wall.mm<35u || park_wall.mm>85u)) wait_sensor(in,out,F2W_PARK_ALIGN);
    else { missing=0; complete=cm>=PARK_TRAVEL_CM; }
  } else {
    turning=0;
    unsigned inner=out->wall_left ? LW_RIGHT : LW_LEFT;
    u16 d=out->wall_left ? in->dR : in->dL;
    u16 wall_mm=RouteRange_Side(in,out->wall_left ? LW_LEFT : LW_RIGHT,1).mm;
    int wall=wall_mm<300u;
    if ((!saw_plate && cm>34u) || (!saw_exit && cm>58u) || cm>ROW_PITCH_CM+18u)
      Fsm2_Fail(F2_BAD_LANDMARK);
    else if (!wall) wait_sensor(in,out,F2W_OUTER_WALL);
    else if (!saw_exit && d==BSP_DIST_INVALID) wait_sensor(in,out,F2W_INNER_RANGE);
    else {
      missing=0;
      if (!saw_plate && cm>=5u && confirmed(d<=140u,in->sample_seq[inner],cm)) {
        saw_plate=1; votes=0;
      } else if (saw_plate && !saw_exit && cm>=REV_EARLIEST_EDGE_CM &&
                 confirmed(d>=240u,in->sample_seq[inner],cm)) {
        saw_exit=1; edge_cm=first_match_cm;
      }
      out->speed_limit=saw_plate ? REV_SPD : REV_FIND_PLATE_COUNTS;
      /* Before edge confirmation use the earliest legal edge as a braking
         reference. It cannot complete the stage; only the observed edge can. */
      u16 reference_cm=saw_exit ? edge_cm : REV_EARLIEST_EDGE_CM;
      int remaining_cm=(int)reference_cm+EDGE_TO_CENTER_CM-cm;
      float v=in->wheel_speed_mm_s;
      float margin_mm=ROUTE_FINAL_COAST_MM+v*(ROUTE_DRIVE_LAG_MS+APP_CTRL_MS)/1000.0f;
      s16 limit=approach_limit(10.0f*remaining_cm-margin_mm,0);
      if (limit<out->speed_limit) out->speed_limit=limit;
      complete=saw_exit && remaining_cm<=0;
    }
  }
  /* The tail beam sees behind the chassis. During reverse motion it can
     tighten the normal braking envelope without voting for a row or park end. */
  if (out->action==F2A_REV && in->use_tail && in->tail_back<BSP_DIST_OVERRANGE) {
    float lag_mm=in->wheel_speed_mm_s*(LW_MAX_AGE_MS+APP_CTRL_MS+ROUTE_DRIVE_LAG_MS)/1000.0f;
    float clearance=(float)in->tail_back-SAFE_STOP_MM-ROUTE_FINAL_COAST_MM-lag_mm;
    s16 limit=approach_limit(clearance,0);
    if (limit<out->speed_limit) out->speed_limit=limit;
  }
  if (complete) {
    /* Keep the rolling orientation during initial braking. Below the measured
       low-speed coast regime, steering may settle in parallel with the wheels. */
    Fsm2State next=(ROUTE_TEST_STOP_AFTER_ROW2 && state==F2_CRAB_L2)
                     ? F2_DONE : (Fsm2State)(state+1);
    float turn_counts=ROUTE_TURN_COAST_MM_S*in->sample_dt_s*ENCODER_CPR/
                      (3.14159265f*WHEEL_DIAMETER_MM);
    for (unsigned w=0; w<4; ++w) {
      if (abs(in->wheel_cnt[w])>(next==F2_DONE ? 150.0f : turn_counts)) {
        stopping=1;
        out->hold=1;
        return;
      }
    }
    state=next;
    enter(in,odom);
    if (state==F2_DONE) turning=0;
  }
  if (complete || state==F2_FAULT) {
    *out=(Fsm2Output){0}; intent(out); out->hold=1;
  }
}
