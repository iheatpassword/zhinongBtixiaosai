/**
  * @file    fsm.c
  * @brief   Competition state machine + 70 cm lateral lane shift (T4 / T5).
  *
  *  Implements the app_if.h contract:
  *      void     FSM_Init(void);
  *      void     FSM_Start(void);
  *      void     FSM_Tick(const FsmInput *in);
  *      CarState FSM_GetState(void);
  *
  *  State sequence:
  *      IDLE -> ENTRY -> RIDGE(1) -> SHIFT(1) -> RIDGE(2) -> SHIFT(2)
  *           -> RIDGE(3) -> ... -> SHIFT(5) -> RIDGE(6) -> EXIT -> PARK
  *      RIDGE/SHIFT --laser invalid > 200 ms--> FAULT (latched)
  *      any active state --run > 4.5 min--> FAULT (latched)
  *
  *  The module never touches the hardware: it only uses the Lead hooks
  *  declared in app_if.h (App_SetWheelSpeed, App_DriveStop, App_SetSteerAll,
  *  App_SetServoCenter, App_GetDistanceCm) and Centering_Update().
  *
  *  Notes
  *  -----
  *  * Every phase keeps its own App_GetDistanceCm() baseline; the contract
  *    has no distance reset hook.
  *  * A state that changes the steering angle waits FSM_STEER_MS before the
  *    first wheel-speed command, so the car never drives while the servos
  *    are travelling (this is a safety margin, not a race requirement).
  *  * Reverse ridge legs use a negative base speed; Centering_Update() owns
  *    the sign handling of its lateral/heading correction.
  *
  *  ASCII only (Keil ARMCC AC5 / GBK project), C90 declarations.
  */
#include "fsm.h"

/* ======================================================================= */
/*  configuration instance                                                 */
/* ======================================================================= */
/* Ridge length table (cm) - edit FSM_RIDGE_LEN_CM_INIT in fsm.h.
   It must contain exactly FSM_RIDGE_CNT entries.                          */
static const u16 ridge_len_cm[FSM_RIDGE_CNT] = FSM_RIDGE_LEN_CM_INIT;

/* ======================================================================= */
/*  module state (single competition instance, no dynamic memory)          */
/* ======================================================================= */
static CarState state;            /* current state                         */
static u8       state_entered;    /* 0: entry action still pending         */
static u32      state_t0_ms;      /* time the current state was entered    */

static u8       t0_set;           /* competition timer started             */
static u32      t0_ms;            /* competition start, 1 ms base          */

static u8       ridges_done;      /* completed ridges, 0..FSM_RIDGE_CNT    */
static u8       shifts_done;      /* completed lateral shifts, 0..N-1      */
static u8       fault_code;       /* 0 none, 1 laser, 2 run time           */

static u16      ph_prev_cm;       /* App distance at the previous tick     */
static u16      ph_cm;            /* distance in the current phase, cm     */

static u8       laser_lost;       /* 1: laser currently invalid            */
static u32      laser_lost_ms;    /* time the laser became invalid         */

static u8       door_blocked;     /* front beam has seen a near obstacle   */
static u8       door_cnt;         /* consecutive clear readings            */

/* ======================================================================= */
/*  helpers                                                                */
/* ======================================================================= */

/* Start a new phase: remember the current odometry value as the baseline. */
static void fsm_phase_reset(void)
{
  ph_prev_cm = App_GetDistanceCm();
  ph_cm      = 0u;
}

/* Accumulate the distance driven since the previous tick.
   The absolute value is used so that the reverse ridge legs count as well.
   A jump above FSM_MAX_INC_CM (u16 wrap, odometry glitch) is discarded for
   one tick and the baseline is re-synchronised.                           */
static void fsm_dist_tick(void)
{
  u16 d;
  s32 inc;

  d   = App_GetDistanceCm();
  inc = (s32)d - (s32)ph_prev_cm;
  if (inc < 0) inc = -inc;
  if (inc > (s32)FSM_MAX_INC_CM) inc = 0;

  ph_cm      = (u16)(ph_cm + (u16)inc);
  ph_prev_cm = d;
}

/* Forget the doorway edge detector.                                      */
static void fsm_door_reset(void)
{
  door_blocked = 0u;
  door_cnt     = 0u;
}

/* Simple doorway detector, shared by ENTRY and EXIT.
   "blocked" = front laser sees a near obstacle (< FSM_BLOCK_MM, e.g. the
   gate frame), "clear" = it sees the open field (>= FSM_DOOR_MM).
   A doorway crossing is reported after FSM_DOOR_TICKS consecutive clear
   frames that follow at least one blocked frame - the edge requirement
   stops a clear reading at power-up from firing immediately.              */
static u8 fsm_door_ahead(const FsmInput *in)
{
  if ((in->laser.valid == 0u) || ((in->laser.flags & 0x01u) == 0u) ||
      (in->laser.dF == 0xFFFFu))
  {
    door_cnt = 0u;
    return 0u;
  }

  if (in->laser.dF < FSM_BLOCK_MM)
  {
    door_blocked = 1u;
    door_cnt     = 0u;
    return 0u;
  }

  if ((in->laser.dF >= FSM_DOOR_MM) && (door_blocked != 0u))
  {
    if (door_cnt < 250u) door_cnt++;
    if (door_cnt >= FSM_DOOR_TICKS) return 1u;
    return 0u;
  }

  door_cnt = 0u;
  return 0u;
}

/* All four wheels at the same signed target (FL, FR, RL, RR).             */
static void fsm_drive_all(s16 spd)
{
  s16 tgt[4];

  tgt[0] = spd;
  tgt[1] = spd;
  tgt[2] = spd;
  tgt[3] = spd;
  App_SetWheelSpeed(tgt);
}

/* Signed base speed of the current ridge.
   ridges_done counts completed ridges, so the ridge being driven is
   (ridges_done + 1): odd ridges forward, even ridges backward.            */
static s16 fsm_ridge_base(void)
{
  if ((ridges_done & 1u) == 0u) return (s16)FSM_SPEED_RIDGE;
  return (s16)(-(s32)FSM_SPEED_RIDGE);
}

/* Request a state change; the entry action runs before the next body.     */
static void fsm_goto(CarState s)
{
  if (s != state)
  {
    state         = s;
    state_entered = 0u;
  }
}

/* Entry action: issued once per state change, before the state body.      */
static void fsm_enter(CarState s, const FsmInput *in)
{
  state_t0_ms = in->ms;

  switch (s)
  {
    case ST_IDLE:
      App_DriveStop();
      App_SetServoCenter();
      break;

    case ST_ENTRY:
      App_DriveStop();
      App_SetServoCenter();
      fsm_door_reset();
      fsm_phase_reset();
      break;

    case ST_RIDGE:
      App_DriveStop();
      fsm_phase_reset();
      break;

    case ST_SHIFT:
      App_DriveStop();                       /* stop motors first           */
      App_SetSteerAll(FSM_SHIFT_STEER_DEG);  /* 4 servos to 90 deg          */
      fsm_phase_reset();
      break;

    case ST_EXIT:
      App_DriveStop();                       /* reverse leg -> forward      */
      App_SetServoCenter();
      fsm_door_reset();
      fsm_phase_reset();
      break;

    case ST_PARK:
      App_DriveStop();
      break;

    case ST_FAULT:
    default:
      App_DriveStop();
      break;
  }
}

/* 1 while the steering servos may still be travelling after the state was
   entered: no wheel-speed command is issued during that time.             */
static u8 fsm_steering_busy(const FsmInput *in)
{
  if ((u32)(in->ms - state_t0_ms) < FSM_STEER_MS) return 1u;
  return 0u;
}

/* Laser supervision: only the ridge and the shift need the laser; a loss
   longer than FSM_LASER_FAULT_MS is a hard fault.                         */
static void fsm_fault_check(const FsmInput *in)
{
  if ((state != ST_RIDGE) && (state != ST_SHIFT)) return;

  if (in->laser.valid != 0u)
  {
    laser_lost = 0u;
    return;
  }

  if (laser_lost == 0u)
  {
    laser_lost    = 1u;
    laser_lost_ms = in->ms;
  }

  if ((u32)(in->ms - laser_lost_ms) > FSM_LASER_FAULT_MS)
  {
    fault_code = 1u;
    fsm_goto(ST_FAULT);
  }
}

/* Competition time limit: active states only, PARK is never faulted.      */
static void fsm_time_check(const FsmInput *in)
{
  if ((t0_set == 0u) || (state == ST_IDLE) || (state == ST_PARK) ||
      (state == ST_FAULT))
  {
    return;
  }

  if ((u32)(in->ms - t0_ms) > FSM_RUN_TIMEOUT_MS)
  {
    fault_code = 2u;
    fsm_goto(ST_FAULT);
  }
}

/* ======================================================================= */
/*  state bodies (called every 20 ms)                                      */
/* ======================================================================= */

static void fsm_run_entry(const FsmInput *in)
{
  fsm_dist_tick();
  if ((fsm_door_ahead(in) != 0u) || (ph_cm >= FSM_ENTRY_CM))
  {
    fsm_goto(ST_RIDGE);                    /* ridge 1, ridges_done == 0   */
    return;
  }
  if (fsm_steering_busy(in) != 0u) return;

  fsm_drive_all((s16)FSM_SPEED_ENTRY);
}

static void fsm_run_ridge(const FsmInput *in)
{
  s16 spd[4];
  s16 base;

  fsm_dist_tick();
  if (ph_cm >= ridge_len_cm[ridges_done])
  {
    ridges_done++;
    if (ridges_done >= (u8)FSM_RIDGE_CNT) fsm_goto(ST_EXIT);
    else                                  fsm_goto(ST_SHIFT);
    return;
  }
  if (fsm_steering_busy(in) != 0u) return;

  base = fsm_ridge_base();
  Centering_Update(&in->laser, in->yaw_deg, in->yaw_rate_dps, base, spd);
  App_SetWheelSpeed(spd);
}

static void fsm_run_shift(const FsmInput *in)
{
  fsm_dist_tick();
  if (ph_cm >= FSM_SHIFT_CM)
  {
    App_SetServoCenter();                  /* wheels straight again       */
    App_DriveStop();                       /* clean stop before reversing */
    shifts_done++;
    fsm_goto(ST_RIDGE);
    return;
  }
  if (fsm_steering_busy(in) != 0u) return;

  fsm_drive_all((s16)FSM_SPEED_SHIFT);     /* creep sideways              */
}

static void fsm_run_exit(const FsmInput *in)
{
  fsm_dist_tick();
  if ((fsm_door_ahead(in) != 0u) || (ph_cm >= FSM_EXIT_CM))
  {
    fsm_goto(ST_PARK);
    return;
  }
  if (fsm_steering_busy(in) != 0u) return;

  fsm_drive_all((s16)FSM_SPEED_EXIT);
}

static void fsm_run(const FsmInput *in)
{
  switch (state)
  {
    case ST_ENTRY: fsm_run_entry(in); break;
    case ST_RIDGE: fsm_run_ridge(in); break;
    case ST_SHIFT: fsm_run_shift(in); break;
    case ST_EXIT:  fsm_run_exit(in);  break;

    case ST_IDLE:
    case ST_PARK:
    case ST_FAULT:
    default:
      break;                               /* passive states: nothing     */
  }
}

/* ======================================================================= */
/*  public API                                                             */
/* ======================================================================= */

void FSM_Init(void)
{
  state         = ST_IDLE;
  state_entered = 0u;          /* first tick issues the IDLE entry action  */
  state_t0_ms   = 0u;

  t0_set = 0u;
  t0_ms  = 0u;

  ridges_done = 0u;
  shifts_done = 0u;
  fault_code  = 0u;

  ph_prev_cm = 0u;
  ph_cm      = 0u;

  laser_lost    = 0u;
  laser_lost_ms = 0u;

  door_blocked = 0u;
  door_cnt     = 0u;
}

void FSM_Start(void)
{
  /* Only a fresh sequence (IDLE) or a latched fault (FAULT) may start;
     a running or parked sequence ignores the request.                    */
  if ((state != ST_IDLE) && (state != ST_FAULT)) return;

  ridges_done   = 0u;
  shifts_done   = 0u;
  fault_code    = 0u;
  t0_set        = 0u;          /* captured on the first tick after start  */
  t0_ms         = 0u;
  ph_prev_cm    = 0u;
  ph_cm         = 0u;
  laser_lost    = 0u;
  laser_lost_ms = 0u;
  door_blocked  = 0u;
  door_cnt      = 0u;

  fsm_goto(ST_ENTRY);
}

void FSM_Tick(const FsmInput *in)
{
  u8 pass;

  if (in == 0) return;

  if ((t0_set == 0u) && (state != ST_IDLE))
  {
    t0_ms  = in->ms;
    t0_set = 1u;
  }

  /* Supervision first: a fault must stop the car on the tick it is seen. */
  fsm_fault_check(in);
  fsm_time_check(in);

  /* Up to two passes: the first may finish the current state, the second
     runs the entry action and body of the new state, so a transition takes
     effect on the same 20 ms tick.                                       */
  for (pass = 0u; pass < 2u; pass++)
  {
    if (state_entered == 0u)
    {
      fsm_enter(state, in);
      state_entered = 1u;
    }

    fsm_run(in);

    if (state_entered != 0u) break;
  }
}

CarState FSM_GetState(void)
{
  return state;
}

/* ======================================================================= */
/*  diagnostics                                                            */
/* ======================================================================= */

u8 FSM_GetRidge(void)
{
  if (state == ST_RIDGE) return (u8)(ridges_done + 1u);
  return 0u;
}

u8 FSM_GetShiftCount(void)
{
  return shifts_done;
}

u16 FSM_GetPhaseCm(void)
{
  return ph_cm;
}

u8 FSM_GetFaultCode(void)
{
  return fault_code;
}

const char *FSM_StateName(CarState st)
{
  switch (st)
  {
    case ST_IDLE:  return "IDLE";
    case ST_ENTRY: return "ENTRY";
    case ST_RIDGE: return "RIDGE";
    case ST_SHIFT: return "SHIFT";
    case ST_EXIT:  return "EXIT";
    case ST_PARK:  return "PARK";
    case ST_FAULT: return "FAULT";
    default:       return "?";
  }
}
