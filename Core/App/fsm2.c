/* Competition route. Odom is travel magnitude; each state has its own origin. */
#include "fsm2.h"
#include "entry_wall.h"
#include "app_config.h"
#include "laser_wire.h"
#include "bsp_types.h"

static Fsm2State state;
static Fsm2Fault fault;
static EntryWall entry_wall;
static u32 start_ms, missing_ms;
static u16 origin, edge_cm, first_match_cm;
static u8 entered, missing, votes, previous_seq, have_seq, saw_plate, saw_exit;

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

static void enter(const AppInputs *in, u16 odom)
{
  origin=odom; start_ms=in->ms; entered=1;
  missing=0; votes=0; have_seq=0; saw_plate=0; saw_exit=0;
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
static void wait_sensor(const AppInputs *in, Fsm2Output *out)
{
  out->hold=1; votes=0;
  if (!missing) { missing=1; missing_ms=in->ms; }
  else if (in->ms-missing_ms>=400u) Fsm2_Fail(F2_MISSING_RANGE);
}

void Fsm2_Tick(const AppInputs *in, u16 odom, Fsm2Output *out)
{
  *out=(Fsm2Output){0};
  intent(out);
  if (out->action==F2A_STOP) { out->hold=1; return; }
  if (!entered) enter(in,odom);
  u32 elapsed=in->ms-start_ms;
  u16 cm=(u16)(odom-origin);
  int complete=0;
  if (!in->laser_fresh) Fsm2_Fail(F2_LOST_LINK);
  else if (!in->imu_ok) Fsm2_Fail(F2_LOST_IMU);
  else if (elapsed>60000u) Fsm2_Fail(F2_TIMEOUT);
  else if (elapsed<ROUTE_SETTLE_MS) { origin=odom; out->hold=1; return; }
  else if (state==F2_ENTRY || out->action==F2A_CRAB_L || out->action==F2A_CRAB_R) {
    int entry=state==F2_ENTRY;
    int right=out->action==F2A_CRAB_R;
    unsigned ch=entry ? LW_FRONT : right ? LW_RIGHT : LW_LEFT;
    u16 d=entry ? in->dF : right ? in->dR : in->dL;
    u16 target=entry ? ENTRY_STOP_MM : SIDE_CENTER_MM;
    u16 min_cm=entry ? 270u : CRAB_TRAVEL_CM-35u;
    u16 max_cm=entry ? 490u : CRAB_TRAVEL_CM+20u;
    /* Outside the two end aisles the forward/backward beams see the furrow.
       A single beam can guide when the other drops out. */
    out->track_ridge=!entry && cm>=35u && cm<=CRAB_TRAVEL_CM-35u;
    int front_ref=in->vF && in->dF>=70u && in->dF<=380u;
    int back_ref=in->vB && in->dB>=40u && in->dB<=300u;
    int sum_error=(int)in->dF+in->dB-(int)(ENTRY_STOP_MM+REAR_TARGET_MM);
    int disagreement=front_ref && back_ref &&
        (sum_error<-(int)RIDGE_SUM_TOL_MM || sum_error>(int)RIDGE_SUM_TOL_MM);
    if (entry) {
      EntryWallCommand wall=EntryWall_Update(&entry_wall,in,cm);
      out->forward_speed=wall.speed;
      out->steer_deg=wall.angle;
    }
    if (cm>max_cm || (d<=target && cm<min_cm)) Fsm2_Fail(F2_BAD_LANDMARK);
    else if (d==BSP_DIST_INVALID || (entry && !out->forward_speed) ||
             (out->track_ridge && ((!front_ref && !back_ref) || disagreement))) wait_sensor(in,out);
    else {
      missing=0;
      out->slow=(d<=target+200u || cm>=max_cm-45u);
      out->hold=d<=target;
      complete=confirmed(out->hold,in->sample_seq[ch],cm);
    }
  } else if (state==F2_REV_PARK) {
    out->slow=1;
    /* The outside wall can end at the mouth of the start box. Check the
       initial alignment while still in the aisle, then finish by odometry. */
    if (cm<10u && (!in->vL || in->dL<35u || in->dL>85u)) wait_sensor(in,out);
    else { missing=0; complete=cm>=PARK_TRAVEL_CM; }
  } else {
    unsigned inner=out->wall_left ? LW_RIGHT : LW_LEFT;
    u16 d=out->wall_left ? in->dR : in->dL;
    u16 wall_mm=out->wall_left ? in->dL : in->dR;
    int wall=wall_mm<300u;
    out->slow=1;
    if ((!saw_plate && cm>34u) || (!saw_exit && cm>58u) || cm>ROW_PITCH_CM+18u)
      Fsm2_Fail(F2_BAD_LANDMARK);
    else if (!wall || (!saw_exit && d==BSP_DIST_INVALID)) wait_sensor(in,out);
    else {
      missing=0;
      if (!saw_plate && cm>=5u && confirmed(d<=140u,in->sample_seq[inner],cm)) {
        saw_plate=1; votes=0;
      } else if (saw_plate && !saw_exit && cm>=32u &&
                 confirmed(d>=240u,in->sample_seq[inner],cm)) {
        saw_exit=1; edge_cm=first_match_cm;
      }
      /* Creep until the near plate is confirmed, then finish the ridge change
         at REV_SPD: the exit edge is found from odometry, so it does not need
         the creep speed once the plate is behind us. */
      out->slow = (saw_plate == 0u);
      complete=saw_exit && cm>=edge_cm+EDGE_TO_CENTER_CM;
    }
  }
  if (complete) {
    state=(Fsm2State)(state+1);
    enter(in,odom);
  }
  if (complete || state==F2_FAULT) {
    *out=(Fsm2Output){0}; intent(out); out->hold=1;
  }
}
