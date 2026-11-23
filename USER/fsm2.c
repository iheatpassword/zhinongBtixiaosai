/**
  * @file    fsm2.c
  * @brief   Cross-ridge competition FSM - pure logic, no hardware access.
  *
  *  Implements fsm2.h:
  *      void       Fsm2_Init(void);
  *      void       Fsm2_Start(void);
  *      void       Fsm2_Abort(void);
  *      void       Fsm2_Tick(const Fsm2Input *in, Fsm2Output *out);
  *      Fsm2State  Fsm2_GetState(void);
  *      const char *Fsm2_StateName(Fsm2State s);
  *
  *  Sequence:
  *      IDLE -> ENTRY -> CRAB_R1 -> REV_12 -> CRAB_L2 -> REV_23
  *           -> CRAB_R3 -> REV_34 -> CRAB_L4 -> REV_45 -> CRAB_R5
  *           -> EXIT_L -> REV_PARK -> DONE
  *
  *  The module never calls App_*() or any driver.  It maps one input snapshot
  *  to one motion intent; the caller owns motors, servos, IMU and safety.
  *  See fsm2.h for the state table, the sensor model and all tuning values.
  *
  *  Notes
  *  -----
  *  * Every crab / reverse state starts with a F2_SETTLE_MS hold: action =
  *    STOP, request_stop = 1, but steer_deg already carries the new angle, so
  *    the servos can travel and the chassis can settle before the next move.
  *    ENTRY is excluded - Start goes forward on the first tick.
  *  * Crab states stop on the boundary laser (raw value, no filter lag) and
  *    fall back to CRAB_FALLBACK_CM of encoder travel.  If both front and
  *    rear lateral sensors are invalid the state faults - no IMU fallback.
  *  * The encoder baseline is re-armed after every hold and whenever the
  *    reading jumps by more than F2_DIST_JUMP_MAX_CM (external reset/glitch).
  *  * Faults are latched; only Fsm2_Start() / Fsm2_Init() clear them.
  *
  *  C90, ASCII only (Keil ARMCC5 / GBK project).
  */
#include "fsm2.h"

/* ======================================================================= */
/*  module state (single instance, no dynamic memory)                      */
/* ======================================================================= */
static Fsm2State s_state;            /* current state                        */
static u8        s_entered;          /* 1: entry action already applied      */
static u32       s_t0_ms;            /* time the current state was entered   */

static s32       s_dist0_cm;         /* encoder baseline at motion start     */
static s32       s_last_dist_cm;     /* encoder value at the previous tick   */
static u16       s_phase_cm;         /* |dist - dist0| in the current action */

static u8        s_fault;            /* F2_FAULT_xxx, latched                */

/* One-pole filters on the two front/rear lateral channels.  They feed the
   diagnostic lateral error; the stop tests deliberately use raw values.    */
static u16       s_dF_f;
static u16       s_dB_f;
static u8        s_dF_i;             /* 1: filter initialised                */
static u8        s_dB_i;

static s16       s_lat_mm;           /* fused front/rear lateral error, mm   */
static u8        s_lat_ok;           /* 1: dF and/or dB valid                */

/* ======================================================================= */
/*  small helpers                                                          */
/* ======================================================================= */

/* A channel is usable only when the caller marked it valid and it is not the
   0xFFFF sentinel.  Over-range values must be flagged with vX = 0 by the
   caller (the safety module already does that).                           */
static u8 f2_valid(u8 v, u16 d)
{
  if ((v != 0u) && (d != 0xFFFFu)) return 1u;
  return 0u;
}

/* One-pole low pass: f = ((DIV-1)*f + raw + DIV/2) / DIV, DIV = 2^SHIFT. */
static void f2_filter(u16 raw, u16 *filt, u8 *init)
{
  s32 div;
  s32 acc;

  div = (s32)(1u << F2_FILTER_SHIFT);

  if (*init == 0u)
  {
    *filt = raw;
    *init = 1u;
    return;
  }

  acc = (s32)(*filt) * (div - 1) + (s32)raw + (div / 2);
  acc = acc / div;

  if (acc < 0)    acc = 0;
  if (acc > 65535) acc = 65535;
  *filt = (u16)acc;
}

/* Refresh all filters and the front/rear lateral error.
   e = mean(dF - 206, dB - 137) over the valid channels only.
   Both invalid -> s_lat_ok = 0 (the crab state turns that into FAULT).     */
static void f2_update_filters(const Fsm2Input *in)
{
  u8  ok_f;
  u8  ok_b;
  s32 e;

  ok_f = f2_valid(in->vF, in->dF);
  ok_b = f2_valid(in->vB, in->dB);

  if (ok_f != 0u) f2_filter(in->dF, &s_dF_f, &s_dF_i);
  else            s_dF_i = 0u;

  if (ok_b != 0u) f2_filter(in->dB, &s_dB_f, &s_dB_i);
  else            s_dB_i = 0u;

  e         = 0;
  s_lat_ok  = 0u;

  if ((ok_f != 0u) && (ok_b != 0u))
  {
    e  = ((s32)s_dF_f - (s32)F2_DF_NOMINAL_MM);
    e += ((s32)s_dB_f - (s32)F2_DB_NOMINAL_MM);
    e  = e / 2;
    s_lat_ok = 1u;
  }
  else if (ok_f != 0u)
  {
    e = (s32)s_dF_f - (s32)F2_DF_NOMINAL_MM;
    s_lat_ok = 1u;
  }
  else if (ok_b != 0u)
  {
    e = (s32)s_dB_f - (s32)F2_DB_NOMINAL_MM;
    s_lat_ok = 1u;
  }
  else
  {
    e = 0;
  }

  e = e * (s32)F2_CRAB_ERR_SIGN;
  if (e > 32767)  e = 32767;
  if (e < -32768) e = -32768;
  s_lat_mm = (s16)e;
}

/* Re-arm the encoder baseline.  Called while the state holds still, so the
   action distance always starts counting when the wheels actually move.   */
static void f2_sync_dist(const Fsm2Input *in)
{
  s_dist0_cm     = in->dist_cm;
  s_last_dist_cm = in->dist_cm;
  s_phase_cm     = 0u;
}

/* Update the action distance as |dist - dist0|.
   A per-tick jump larger than F2_DIST_JUMP_MAX_CM means the caller reset the
   encoder counter between actions (or a glitch): take a new baseline and do
   not count the jump.  Handles unsigned wrap through the u32 subtraction.  */
static void f2_dist_tick(const Fsm2Input *in)
{
  u32 now;
  u32 last;
  u32 base;
  u32 jump;
  u32 delta;

  now  = (u32)in->dist_cm;
  last = (u32)s_last_dist_cm;
  jump = (now >= last) ? (now - last) : (last - now);

  if (jump > (u32)F2_DIST_JUMP_MAX_CM)
  {
    s_dist0_cm     = in->dist_cm;
    s_last_dist_cm = in->dist_cm;
    s_phase_cm     = 0u;
    return;
  }

  base  = (u32)s_dist0_cm;
  delta = (now >= base) ? (now - base) : (base - now);
  if (delta > 65535u) delta = 65535u;

  s_phase_cm     = (u16)delta;
  s_last_dist_cm = in->dist_cm;
}

/* 1 while the servo travel / settle hold of a crab or reverse state runs. */
static u8 f2_hold_active(const Fsm2Input *in)
{
  u32 elapsed;

  elapsed = (u32)(in->ms - s_t0_ms);      /* wrap-safe unsigned difference */
  if (elapsed < (u32)F2_SETTLE_MS) return 1u;
  return 0u;
}

static void f2_goto(Fsm2State s)
{
  s_state   = s;
  s_entered = 0u;
}

/* Entry action: timestamp and distance baseline only (no hardware).       */
static void f2_enter(Fsm2State s, const Fsm2Input *in)
{
  s_state        = s;
  s_t0_ms        = in->ms;
  s_dist0_cm     = in->dist_cm;
  s_last_dist_cm = in->dist_cm;
  s_phase_cm     = 0u;
  s_entered      = 1u;
}

/* Latch a fault.  The first cause is kept if a state is already faulted.   */
static void f2_fault(u8 code)
{
  if (s_state != F2_FAULT)
  {
    s_fault = code;
  }
  f2_goto(F2_FAULT);
}

static u8 f2_is_passive(Fsm2State s)
{
  if ((s == F2_IDLE) || (s == F2_DONE) || (s == F2_FAULT)) return 1u;
  return 0u;
}

/* ======================================================================= */
/*  supervision                                                           */
/* ======================================================================= */

/* Hard safety, checked on every active tick before any motion intent:
   stall, or any valid laser closer than F2_LASER_MIN_MM.                   */
static u8 f2_check_safety(const Fsm2Input *in)
{
  if (in->stall != 0u)
  {
    f2_fault(F2_FAULT_STALL);
    return 1u;
  }

  if ((f2_valid(in->vF, in->dF) != 0u) && (in->dF < F2_LASER_MIN_MM))
  {
    f2_fault(F2_FAULT_NEAR_F);
    return 1u;
  }

  if ((f2_valid(in->vB, in->dB) != 0u) && (in->dB < F2_LASER_MIN_MM))
  {
    f2_fault(F2_FAULT_NEAR_B);
    return 1u;
  }

  if ((f2_valid(in->vL, in->dL) != 0u) && (in->dL < F2_LASER_MIN_MM))
  {
    f2_fault(F2_FAULT_NEAR_L);
    return 1u;
  }

  if ((f2_valid(in->vR, in->dR) != 0u) && (in->dR < F2_LASER_MIN_MM))
  {
    f2_fault(F2_FAULT_NEAR_R);
    return 1u;
  }

  return 0u;
}

static u32 f2_timeout_for(Fsm2State s)
{
  switch (s)
  {
    case F2_ENTRY:
      return (u32)F2_ENTRY_TIMEOUT_MS;

    case F2_CRAB_R1:
    case F2_CRAB_L2:
    case F2_CRAB_R3:
    case F2_CRAB_L4:
    case F2_CRAB_R5:
    case F2_EXIT_L:
      return (u32)CRAB_TIMEOUT_MS;

    case F2_REV_12:
    case F2_REV_23:
    case F2_REV_34:
    case F2_REV_45:
    case F2_REV_PARK:
      return (u32)F2_REV_TIMEOUT_MS;

    case F2_IDLE:
    case F2_DONE:
    case F2_FAULT:
    default:
      return 0u;                        /* passive states never time out  */
  }
}

static u8 f2_check_timeout(const Fsm2Input *in)
{
  u32 tmo;

  tmo = f2_timeout_for(s_state);
  if (tmo == 0u) return 0u;

  if ((u32)(in->ms - s_t0_ms) >= tmo)
  {
    f2_fault(F2_FAULT_TIMEOUT);
    return 1u;
  }

  return 0u;
}

/* ======================================================================= */
/*  transition tables                                                      */
/* ======================================================================= */

static Fsm2State f2_crab_next(Fsm2State s)
{
  switch (s)
  {
    case F2_CRAB_R1: return F2_REV_12;
    case F2_CRAB_L2: return F2_REV_23;
    case F2_CRAB_R3: return F2_REV_34;
    case F2_CRAB_L4: return F2_REV_45;
    case F2_CRAB_R5: return F2_EXIT_L;
    case F2_EXIT_L:  return F2_REV_PARK;
    default:         return F2_FAULT;
  }
}

static Fsm2State f2_rev_next(Fsm2State s)
{
  switch (s)
  {
    case F2_REV_12:   return F2_CRAB_L2;
    case F2_REV_23:   return F2_CRAB_R3;
    case F2_REV_34:   return F2_CRAB_L4;
    case F2_REV_45:   return F2_CRAB_R5;
    case F2_REV_PARK: return F2_DONE;
    default:          return F2_FAULT;
  }
}

/* ======================================================================= */
/*  state bodies                                                           */
/* ======================================================================= */

static void f2_run_entry(const Fsm2Input *in)
{
  /* ENTRY has no settle hold: the servos are already straight and the car
     must leave the start area as soon as Start is given.                    */
  f2_dist_tick(in);

  if (((f2_valid(in->vF, in->dF) != 0u) && (in->dF <= ENTRY_STOP_MM)) ||
      (s_phase_cm >= (u16)F2_ENTRY_FALLBACK_CM))
  {
    f2_goto(F2_CRAB_R1);
  }
}

static void f2_run_crab(const Fsm2Input *in)
{
  u8        left;
  u8        ok;
  u16       d;
  Fsm2State next;

  if (f2_hold_active(in) != 0u)
  {
    f2_sync_dist(in);
    return;
  }

  f2_dist_tick(in);

  /* Right crab watches dR, left crab watches dL. */
  left = 0u;
  if ((s_state == F2_CRAB_L2) ||
      (s_state == F2_CRAB_L4) ||
      (s_state == F2_EXIT_L))
  {
    left = 1u;
  }

  if (left != 0u)
  {
    ok = f2_valid(in->vL, in->dL);
    d  = in->dL;
  }
  else
  {
    ok = f2_valid(in->vR, in->dR);
    d  = in->dR;
  }

  /* Normal completion wins over the lateral-loss test on the same tick:
     the move is already over when the fallback distance is reached.        */
  if (((ok != 0u) && (d <= CRAB_STOP_MM)) ||
      (s_phase_cm >= (u16)CRAB_FALLBACK_CM))
  {
    next = f2_crab_next(s_state);
    f2_goto(next);
    return;
  }

  /* Neither front nor rear lateral reference: stop, do not use the IMU. */
  if (s_lat_ok == 0u)
  {
    f2_fault(F2_FAULT_NO_LATERAL);
  }
}

static void f2_run_rev(const Fsm2Input *in)
{
  u16 target_cm;

  if (f2_hold_active(in) != 0u)
  {
    f2_sync_dist(in);
    return;
  }

  f2_dist_tick(in);

  if (s_state == F2_REV_PARK) target_cm = (u16)REV_PARK_CM;
  else                        target_cm = (u16)REV_RIDGE_CM;

  if (s_phase_cm >= target_cm)
  {
    f2_goto(f2_rev_next(s_state));
  }
}

static void f2_run_state(const Fsm2Input *in)
{
  switch (s_state)
  {
    case F2_ENTRY:
      f2_run_entry(in);
      break;

    case F2_CRAB_R1:
    case F2_CRAB_L2:
    case F2_CRAB_R3:
    case F2_CRAB_L4:
    case F2_CRAB_R5:
    case F2_EXIT_L:
      f2_run_crab(in);
      break;

    case F2_REV_12:
    case F2_REV_23:
    case F2_REV_34:
    case F2_REV_45:
    case F2_REV_PARK:
      f2_run_rev(in);
      break;

    case F2_IDLE:
    case F2_DONE:
    case F2_FAULT:
    default:
      break;
  }
}

/* ======================================================================= */
/*  output mapping                                                         */
/* ======================================================================= */

static void f2_output_safe(Fsm2Output *out)
{
  out->action       = F2A_STOP;
  out->target_cm    = 0u;
  out->stop_mm      = 0u;
  out->steer_deg    = 0;
  out->request_stop = 1u;
  out->done         = (s_state == F2_DONE) ? 1u : 0u;
}

static void f2_fill_output(const Fsm2Input *in, Fsm2Output *out)
{
  Fsm2Action act;
  u16        target_cm;
  u16        stop_mm;
  s16        steer;

  act       = F2A_STOP;
  target_cm = 0u;
  stop_mm   = 0u;
  steer     = F2_STEER_STRAIGHT_DEG;

  switch (s_state)
  {
    case F2_ENTRY:
      act       = F2A_FWD;
      target_cm = (u16)F2_ENTRY_FALLBACK_CM;
      stop_mm   = (u16)ENTRY_STOP_MM;
      steer     = F2_STEER_STRAIGHT_DEG;
      break;

    case F2_CRAB_R1:
    case F2_CRAB_R3:
    case F2_CRAB_R5:
      act       = F2A_CRAB_R;
      target_cm = (u16)CRAB_FALLBACK_CM;
      stop_mm   = (u16)CRAB_STOP_MM;
      steer     = (s16)F2_STEER_CRAB_DEG;
      break;

    case F2_CRAB_L2:
    case F2_CRAB_L4:
    case F2_EXIT_L:
      act       = F2A_CRAB_L;
      target_cm = (u16)CRAB_FALLBACK_CM;
      stop_mm   = (u16)CRAB_STOP_MM;
      steer     = (s16)(-(s32)F2_STEER_CRAB_DEG);
      break;

    case F2_REV_12:
    case F2_REV_23:
    case F2_REV_34:
    case F2_REV_45:
      act       = F2A_REV;
      target_cm = (u16)REV_RIDGE_CM;
      stop_mm   = 0u;
      steer     = F2_STEER_STRAIGHT_DEG;
      break;

    case F2_REV_PARK:
      act       = F2A_REV;
      target_cm = (u16)REV_PARK_CM;
      stop_mm   = 0u;
      steer     = F2_STEER_STRAIGHT_DEG;
      break;

    case F2_IDLE:
    case F2_DONE:
    case F2_FAULT:
    default:
      act       = F2A_STOP;
      target_cm = 0u;
      stop_mm   = 0u;
      steer     = F2_STEER_STRAIGHT_DEG;
      break;
  }

  /* Settle hold: keep the new steering angle but do not start the move.
     ENTRY is excluded so Start goes forward immediately.                    */
  if ((act != F2A_STOP) && (s_state != F2_ENTRY) &&
      (f2_hold_active(in) != 0u))
  {
    act = F2A_STOP;
  }

  out->action       = act;
  out->target_cm    = target_cm;
  out->stop_mm      = stop_mm;
  out->steer_deg    = steer;
  out->request_stop = (act == F2A_STOP) ? 1u : 0u;
  out->done         = (s_state == F2_DONE) ? 1u : 0u;
}

/* ======================================================================= */
/*  public API                                                             */
/* ======================================================================= */

void Fsm2_Init(void)
{
  s_state = F2_IDLE;
  s_entered = 0u;
  s_t0_ms = 0u;

  s_dist0_cm = 0;
  s_last_dist_cm = 0;
  s_phase_cm = 0u;

  s_fault = F2_FAULT_NONE;

  s_dF_f = 0u;
  s_dB_f = 0u;
  s_dF_i = 0u;
  s_dB_i = 0u;

  s_lat_mm = 0;
  s_lat_ok = 0u;
}

void Fsm2_Start(void)
{
  /* A running sequence is not restarted by a second Start. */
  if ((s_state != F2_IDLE) && (s_state != F2_DONE) && (s_state != F2_FAULT))
  {
    return;
  }

  s_fault = F2_FAULT_NONE;
  s_phase_cm = 0u;
  s_dist0_cm = 0;
  s_last_dist_cm = 0;
  s_t0_ms = 0u;
  s_entered = 0u;
  s_state = F2_ENTRY;
}

void Fsm2_Abort(void)
{
  if (s_state != F2_FAULT)
  {
    s_fault = F2_FAULT_ABORT;
  }

  s_state = F2_FAULT;
  s_entered = 1u;                    /* passive state: no entry action     */
}

void Fsm2_Tick(const Fsm2Input *in, Fsm2Output *out)
{
  u8 pass;

  if (out == 0) return;

  if (in == 0)
  {
    f2_output_safe(out);
    return;
  }

  f2_update_filters(in);

  /* Up to three passes: safety/timeout may fault, a body may finish, and the
     new state must get its entry action plus its settle-hold output on the
     same tick.  A passive state stops the loop immediately.                 */
  for (pass = 0u; pass < 3u; pass++)
  {
    if (s_entered == 0u)
    {
      f2_enter(s_state, in);
    }

    if (f2_is_passive(s_state) != 0u)
    {
      break;
    }

    if (f2_check_safety(in) != 0u)
    {
      continue;                      /* state is now F2_FAULT              */
    }

    if (f2_check_timeout(in) != 0u)
    {
      continue;                      /* state is now F2_FAULT              */
    }

    f2_run_state(in);

    if (s_entered != 0u)
    {
      break;
    }
  }

  /* Safety net for F2_SETTLE_MS = 0 (several transitions in one tick): never
     leave the output mapped from a state whose entry action has not run.   */
  if (s_entered == 0u)
  {
    f2_enter(s_state, in);
  }

  f2_fill_output(in, out);
}

Fsm2State Fsm2_GetState(void)
{
  return s_state;
}

const char *Fsm2_StateName(Fsm2State s)
{
  switch (s)
  {
    case F2_IDLE:     return "IDLE";
    case F2_ENTRY:    return "ENTRY";
    case F2_CRAB_R1:  return "CRAB_R1";
    case F2_REV_12:   return "REV_12";
    case F2_CRAB_L2:  return "CRAB_L2";
    case F2_REV_23:   return "REV_23";
    case F2_CRAB_R3:  return "CRAB_R3";
    case F2_REV_34:   return "REV_34";
    case F2_CRAB_L4:  return "CRAB_L4";
    case F2_REV_45:   return "REV_45";
    case F2_CRAB_R5:  return "CRAB_R5";
    case F2_EXIT_L:   return "EXIT_L";
    case F2_REV_PARK: return "REV_PARK";
    case F2_DONE:     return "DONE";
    case F2_FAULT:    return "FAULT";
    default:          return "?";
  }
}

/* ======================================================================= */
/*  diagnostics                                                            */
/* ======================================================================= */

u8 Fsm2_GetFaultCode(void)
{
  return s_fault;
}

u16 Fsm2_GetPhaseCm(void)
{
  return s_phase_cm;
}

s16 Fsm2_GetLateralMm(void)
{
  if (s_lat_ok == 0u) return 0;
  return s_lat_mm;
}

u8 Fsm2_GetLateralValid(void)
{
  return s_lat_ok;
}
