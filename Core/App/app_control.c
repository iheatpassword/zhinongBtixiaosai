/**
  * @file    app_control.c
  * @brief   Mode manager, wheel speed loop and the motion modes.
  *
  *  Replaces the 20 ms block of the old USER/main.c.  The eleven independent
  *  mode flags there are collapsed into one AppMode variable: only one mode
  *  can own the actuators at a time, which removes the implicit ordering
  *  between them.
  *
  *  Everything here runs in the control task at exactly 50 Hz and must never
  *  block.  The servo frames are the only I/O, and they are ~3.5 ms.
  */
#include "tune_params.h"
#include "app_if.h"
#include "app_config.h"
#include "app_tasks.h"

#include "app_report.h"
#include "fsm2.h"
#include "entry_wall.h"
#include "route_ranges.h"
#include "laser_wire.h"
#include "chassis_motion.h"
#include "safety.h"

#include "board.h"
#include "motor/motor.h"
#include "servo/servo.h"
#include "imu/imu.h"
#include <math.h>

/* ----------------------------------------------------------------------- */
/*  state                                                                  */
/* ----------------------------------------------------------------------- */
static AppMode s_mode;
static volatile u8 s_pending_cmd;
static s16 s_base_spd;
static volatile u16 s_rev_target_cm = 370u;   /* distance for the R command */

/* speed loop */
static s16 s_tgt[4];            /* desired counts per 20 ms, signed        */
static s16 s_mtgt[4];           /* magnitude actually commanded           */
static s16 s_meas[4];
static u16 s_prev_mag[4];
static u32 s_prev_encoder_ms;
static s16 s_pwm[4];
static s32 s_integ[4];
static s8  s_drive;             /* 0 stop, +1 forward, -1 reverse          */

/* steering */
static s16 s_steer10[4];
static u8  s_steer_dirty;
static u32 s_side_block_ms;   /* when a side beam first blocked a wall-follow */

/* odometry */
static s32 s_odom;

/* heading trim */
static float s_diff_i;

static u16 s_test_origin;
static u32 s_test_ms;
static EntryWall s_test_wall;
static u32 s_test_wall_last_ok;
static u32 s_edge_last_ok;     /* last tick with a valid leading range */
static u32 s_edge_lost_ms;     /* when the leading range was first lost   */
static u8  s_edge_lost;
static s16 s_crab_dir;
static Fsm2Output s_f2out;

/* wheel identification */
static u8  s_wheel_idx;
static u32 s_wheel_on_ms;
static s16 s_wheel_peak[4];

/* safety layer */
static SafetyFilt  s_sf;
static SafetyStall s_stall;
static u8          s_safety;      /* bit0 = distance stop, bit1 = stall */

/* last PWM actually handed to the motors, for telemetry */
static s16 s_applied[4];

/* Boot settle: motion commands are dropped until this window expires. */
#define BOOT_SETTLE_MS 3000u
static u8  s_armed;
static u8 s_run_tail, s_tail_wait;
static u32 s_tail_lost_ms;
static u32 s_boot_ms;

/* Motor driver enable (PD3), refreshed on every control tick. */
static u8  s_en_ok;

/* ----------------------------------------------------------------------- */
/*  helpers                                                                */
/* ----------------------------------------------------------------------- */
static s16 clamp_s16f(float v, float lo, float hi)
{
  if (v < lo) v = lo;
  if (v > hi) v = hi;
  return (s16)v;
}

static void set_steer_all(float deg)
{
  s16 next=clamp_s16f(deg*10.0f,-(90+app_tune->crab_trim_max_deg)*10,(90+app_tune->crab_trim_max_deg)*10);
  if (next==s_steer10[0] && next==s_steer10[1] && next==s_steer10[2] && next==s_steer10[3]) return;
  s_steer10[0]=next;
  s_steer10[1] = s_steer10[0];
  s_steer10[2] = s_steer10[0];
  s_steer10[3] = s_steer10[0];
  s_steer_dirty = 1u;
}

/* Apply a crab angle with an optional front/rear split.  With trim == 0 this is
   exactly set_steer_all(); with a trim the four wheels are no longer parallel,
   so the body rotates while it translates.  That rotation is the only way to
   correct a heading error on this chassis - the left/right differential is
   mechanically coupled and delivers a few percent of what it is asked for. */
static void apply_steer_split(float angle, float trim)
{
  if ((trim > -0.05f) && (trim < 0.05f))
  {
    set_steer_all(angle);
    return;
  }
  App_SetSteer4(angle + trim, angle + trim, angle - trim, angle - trim);
}

static void drive_zero(void)
{
  u8 w;
  for (w = 0u; w < 4u; w++)
  {
    s_tgt[w]  = 0;
    s_mtgt[w] = 0;
  }
  s_drive = 0;
}

static void all_zero(void)
{
  drive_zero();
}

/* Position and wheel split both reverse sign when driving backwards. */
static float reverse_wall_angle(const AppInputs *in, int left, int standalone, float *trim)
{
  float center = RouteRange_Side(in,left ? LW_LEFT : LW_RIGHT,1).mm;
  *trim = 0;
  if (center >= 300u) return 0;
  EntryWall_Pair(in, left, 300u, &center, trim);
  *trim = -*trim;
  float target = left ? app_tune->entry_side_target_mm : SIDE_CENTER_MM;
  float kp = standalone ? app_tune->entry_wall_kp : app_tune->edge_kp;
  float limit = standalone ? app_tune->entry_wall_max_deg : app_tune->edge_max;
  float angle = (left ? 1.0f : -1.0f) * kp * (center - target);
  return fminf(limit, fmaxf(-limit, angle));
}

void App_SetRevTargetCm(u16 cm)
{
  s_rev_target_cm = (cm < 20u) ? 20u : cm;
}

/* Route speeds and existing trim gains use encoder counts per control period.
   Convert to chassis velocity here; only the inner wheel loop uses counts. */
static void command_motion(ChassisMotion motion, int roll_sign, float wheel_bias)
{
  WheelMotion wheels[4];
  float extra = app_tune->crab_trim_max_deg;
  Chassis_Solve(motion, roll_sign, extra, wheels);
  s_drive = roll_sign;
  for (unsigned i = 0; i < 4; ++i) {
    float bias = (i == 0 || i == 2) ? wheel_bias : -wheel_bias;
    float counts = wheels[i].speed_mm_s / MM_S_PER_COUNT;
    /* A speed calibration offset alone must never start or reverse a wheel. */
    bias = fminf(fabsf(counts), fmaxf(-fabsf(counts), bias));
    s_tgt[i] = clamp_s16f(roundf(counts + bias), -32000, 32000);
    s16 angle = clamp_s16f(roundf(wheels[i].steer_deg * 10),
                           -(90 + extra) * 10, (90 + extra) * 10);
    if (angle != s_steer10[i]) {
      s_steer10[i] = angle;
      s_steer_dirty = 1;
    }
  }
}

static void drive_longitudinal(s16 speed, float angle, float split,
                               float yaw_counts, float wheel_bias)
{
  if (speed == 0) {
    drive_zero();
    apply_steer_split(angle, split);
    return;
  }
  float v = speed * MM_S_PER_COUNT;
  float steer = angle * CHASSIS_RAD_PER_DEG;
  float yaw = 2 * yaw_counts * MM_S_PER_COUNT / CHASSIS_TRACK_MM;
  yaw += 2 * v * tanf(split * CHASSIS_RAD_PER_DEG) / CHASSIS_WHEELBASE_MM;
  /* Keep gyro corrections inside the configured fore/aft steering authority,
     including at creep speed. */
  float limit = 2 * fabsf(v) * tanf(app_tune->entry_yaw_trim_max_deg * CHASSIS_RAD_PER_DEG)
                / CHASSIS_WHEELBASE_MM;
  ChassisMotion motion = {v * cosf(steer), v * sinf(steer), fminf(limit, fmaxf(-limit, yaw))};
  command_motion(motion, speed < 0 ? -1 : 1, wheel_bias);
}

static void longitudinal(const AppInputs *in, s16 speed, float angle, float split, HeadingLog *log)
{
  float bias = speed > 0 ? app_tune->diff_ff * speed / EDGE_SPD : -app_tune->diff_ff;
  float corr = app_tune->yaw_drive_kp * in->yaw_deg + app_tune->yaw_drive_kd * in->yaw_rate_dps;
  log->raw = (int)(corr + bias);
  log->correction = clamp_s16f(corr + bias, -app_tune->yaw_drive_max, app_tune->yaw_drive_max);
  if (s_mode == APP_MODE_EDGE && !s_test_wall.acquired && speed > 0) {
    /* E entrance comparison: until the rail is confirmed, use the original
       left/right speed trim and keep the search steering independent of yaw. */
    s_tgt[0] = s_tgt[2] = speed + log->correction;
    s_tgt[1] = s_tgt[3] = speed - log->correction;
    s_drive = 1;
    apply_steer_split(angle, split);
    return;
  }
  bias = fminf(app_tune->yaw_drive_max, fmaxf(-app_tune->yaw_drive_max, bias));
  drive_longitudinal(speed, angle, split, log->correction - bias, bias);
}

static void crab_step(const AppInputs *in, int dir, s16 speed, int tracking, HeadingLog *log)
{
  float error=0;
  RidgeStatus ridge = tracking ? RouteRange_Ridge(in,&error) : RIDGE_MISSING;
  float trim=app_tune->crab_trim_kp*error;
  if (trim>app_tune->crab_trim_max_deg) trim=app_tune->crab_trim_max_deg;
  if (trim<-app_tune->crab_trim_max_deg) trim=-app_tune->crab_trim_max_deg;
  if (in->use_tail && (ridge == RIDGE_PAIR || ridge == RIDGE_VALID)) {
    /* IMU positive is counterclockwise. Transform the desired field
       translation into body axes before solving the four wheel angles. */
    trim -= dir * in->yaw_deg;
    trim = fminf(90.0f, fmaxf(-app_tune->crab_trim_max_deg, trim));
    /* Leave wheel-speed headroom while recovering a large offset. */
    float recovery = fminf(1.0f, fabsf(error) / 65.0f);
    speed = (s16)fminf(speed, SHIFT_SPD * (1.0f - 0.28f * recovery));
  }
  float trim_rad = trim * CHASSIS_RAD_PER_DEG;
  float demand=app_tune->shift_yaw_spd_kp*in->yaw_deg+app_tune->shift_yaw_spd_kd*in->yaw_rate_dps;
  s16 cy=dir*SHIFT_YAW_SPD_SIGN*clamp_s16f(demand,-app_tune->shift_yaw_spd_max,app_tune->shift_yaw_spd_max);
  log->raw=(int)(dir*SHIFT_YAW_SPD_SIGN*demand);
  log->correction=cy;
  if (speed == 0) {
    drive_zero();
    set_steer_all(dir * (90.0f - trim));
    return;
  }
  float v = speed * MM_S_PER_COUNT;
  ChassisMotion motion = {v * sinf(trim_rad), dir * v * cosf(trim_rad),
                          -2 * dir * cy * MM_S_PER_COUNT / CHASSIS_WHEELBASE_MM};
  command_motion(motion, 1, 0);
}

/* ----------------------------------------------------------------------- */
/*  wheel speed loop                                                       */
/* ----------------------------------------------------------------------- */
static void speed_step(const AppInputs *in, AppOutputs *out)
{
  u8 w;
  u32 window_ms = in->encoder_period_ms + s_prev_encoder_ms;
  s_prev_encoder_ms = in->encoder_period_ms;

  for (w = 0u; w < 4u; w++)
  {
    int raw = (int)in->wheel_cnt[w];
    int mag = (raw < 0) ? -raw : raw;
    s32 e;
    s32 p;

    /* Keep the two-sample filter in the same counts/20 ms units as the target. */
    if (window_ms != 0u) {
      u32 speed = ((mag + s_prev_mag[w]) * APP_CTRL_MS) / window_ms;
      s_meas[w] = (s16)(speed > 32000u ? 32000u : speed);
    }
    s_prev_mag[w] = (u16)mag;

    /* A wheel whose demand points against the chassis direction is held at
       zero, matching the old behaviour where such a target clamped the PWM
       to 0 instead of spinning the wheel backwards. */
    if ((s_drive == 0) || ((s_tgt[w] > 0) && (s_drive < 0)) ||
        ((s_tgt[w] < 0) && (s_drive > 0)))
    {
      s_mtgt[w] = 0;
    }
    else
    {
      s_mtgt[w] = (s16)((s_tgt[w] < 0) ? -s_tgt[w] : s_tgt[w]);
    }

    if ((s_drive == 0) || (s_mtgt[w] == 0))
    {
      s_integ[w] = 0;
      s_pwm[w]   = 0;
      out->pwm[w] = 0;
      continue;
    }

    e = (s32)s_mtgt[w] - (s32)s_meas[w];
    p = (s32)(app_tune->kp_num * e) / 10 + app_tune->pwm_ff
        + ((s32)s_mtgt[w] * app_tune->wheel_ff_num) / 10;
    s32 lower = s_pwm[w] - app_tune->pwm_slew;
    s32 upper = s_pwm[w] + app_tune->pwm_slew;
    if (lower < 0) lower = 0;
    if (upper > app_tune->pwm_lim) upper = app_tune->pwm_lim;
    if (lower > upper) lower = upper; /* PWM_LIM may be reduced while driving. */

    /* Integrate only the part of this step the actuator can deliver.
       An opposite error can still unwind an existing integral. */
    s32 delta = e / app_tune->ki_div;
    s32 demand = p + s_integ[w];
    if (delta > 0 && demand + delta > upper)
      delta = demand < upper ? upper - demand : 0;
    if (delta < 0 && demand + delta < lower)
      delta = demand > lower ? lower - demand : 0;
    s_integ[w] += delta;
    if (s_integ[w] >  app_tune->integ_max) s_integ[w] =  app_tune->integ_max;
    if (s_integ[w] < -app_tune->integ_max) s_integ[w] = -app_tune->integ_max;

    p += s_integ[w];
    if (p > upper) p = upper;
    if (p < lower) p = lower;

    s_pwm[w]    = (s16)p;
    out->pwm[w] = (s16)((s_drive > 0) ? s_pwm[w] : -s_pwm[w]);
  }
}

/* ----------------------------------------------------------------------- */
/*  safety layer                                                           */
/*                                                                         */
/*  Runs AFTER the speed loop so the stall detector sees the PWM that is    */
/*  actually about to be applied, and BEFORE the output is written, so a    */
/*  trip takes effect on the same tick.  The old firmware checked the       */
/*  filtered distances one tick late and had the whole layer compiled out   */
/*  (SAFETY_ENABLE = 0).                                                    */
/* ----------------------------------------------------------------------- */
static void safety_eval(const AppInputs *in, AppOutputs *out)
{
  SafetyRaw raw;
  u16       apwm[4];
  u8        w;
  u8        trip = 0u;

  raw.dF = in->dF;
  raw.dB = in->dB;
  raw.dL = in->dL;
  raw.dR = in->dR;
  raw.vF = in->vF;
  raw.vB = in->vB;
  raw.vL = in->vL;
  raw.vR = in->vR;

  /* Side clearance is the nearer end, independent of the guidance geometry.
     Keep the old nose-mounted back-facing beam distinct from the tail beam. */
  if (in->tail_left < raw.dL) { raw.dL=in->tail_left; raw.vL=raw.dL<BSP_DIST_OVERRANGE; }
  if (in->tail_right < raw.dR) { raw.dR=in->tail_right; raw.vR=raw.dR<BSP_DIST_OVERRANGE; }
  if (in->tail_back < raw.dB) { raw.dB=in->tail_back; raw.vB=raw.dB<BSP_DIST_OVERRANGE; }

  Safety_Filter(&raw, &s_sf);

  for (w = 0u; w < 4u; w++)
  {
    int p = (int)out->pwm[w];
    apwm[w] = (u16)((p < 0) ? -p : p);
  }

  /* s_meas[] already holds |counts| per wheel. */
  Safety_StallUpdate(&s_stall, apwm, (const s16 *)s_meas, (u32)APP_CTRL_MS);

  if (Safety_MinDistanceStop(&s_sf, (u16)SAFE_STOP_MM) != 0u) trip |= 0x01u;
  if (s_stall.stalled != 0u)                                  trip |= 0x02u;
  if (s_en_ok == 0u)                                          trip |= 0x04u;

  s_safety = trip;

  if (trip == 0u) return;

  /* A side beam that is too close is not "stop the task": the only way out is
     to drive while crabbing away from that side, and the old code zeroed the
     drive and dropped to IDLE, so every drift into the right wall ended the
     run and waited for a human.  Keep the mode and allow a creep-level drive
     while the commanded steering points away from the tripped side; freeze
     otherwise, and give up only if it stays blocked for SIDE_BLOCK_GIVEUP_MS. */
  if (trip == 0x01u) {
    int front_bad = (s_sf.vF && (s_sf.dF < SAFE_STOP_MM)) ||
                    (in->tail_back < SAFE_STOP_MM);
    int right_bad = s_sf.vR && (s_sf.dR < SAFE_STOP_MM);
    int left_bad  = s_sf.vL && (s_sf.dL < SAFE_STOP_MM);
    /* FSM2 belongs here too: the entry follows the left rail with no right-hand
       loop, so on a section where the aisle narrows the car is pushed against
       the right fence and the right beam reaches 9 mm.  Without this the safety
       layer failed the whole competition run with F2_OBSTRUCTION at the exact
       moment the entry arrived (observed: dL 85->106, dR 31->9, then FA=7). */
    int wall_mode = (s_mode == APP_MODE_EDGE) || (s_mode == APP_MODE_REV_EDGE) ||
                    (s_mode == APP_MODE_FSM2);

    int entering = s_mode == APP_MODE_EDGE ||
                   (s_mode == APP_MODE_FSM2 && Fsm2_GetState() == F2_ENTRY);
    if (!entering && !front_bad && (left_bad || right_bad) && wall_mode) {
      unsigned side = right_bad ? 1u : 0u;
      int away_sign = right_bad ? -s_drive : s_drive;
      /* The near-side front and rear wheels can now point in different
         directions. Both ends must translate away, also when reversing. */
      int away = !(left_bad && right_bad) && away_sign * s_steer10[side] > 5 &&
                 away_sign * s_steer10[side + 2] > 5;

      if (away) {
        s_side_block_ms = 0u;
        for (w = 0u; w < 4u; w++) {
          s16 p = out->pwm[w];
          if (p >  SAFE_SIDE_ESCAPE_PWM) p =  SAFE_SIDE_ESCAPE_PWM;
          if (p < -SAFE_SIDE_ESCAPE_PWM) p = -SAFE_SIDE_ESCAPE_PWM;
          out->pwm[w] = p;
        }
        return;                     /* keep the mode and let it slide away */
      }

      if (s_side_block_ms == 0u) s_side_block_ms = in->ms;
      if (in->ms - s_side_block_ms < SIDE_BLOCK_GIVEUP_MS) {
        for (w = 0u; w < 4u; w++) out->pwm[w] = 0;
        drive_zero();
        return;                     /* keep the mode, just stop for now */
      }
    }
    s_side_block_ms = 0u;
  }

  /* A distance trip means "do not drive further into this", not "freeze".
     Backing away is always allowed and clipped to a creep PWM, so a car that
     stopped hard against a wall can still get itself out with B or R.  Stall
     and enable-low keep the full veto.
     The test is on the mode's intent, not s_drive: the settle phase of a mode
     zeroes s_drive, and using it here let the trip kill the mode before the
     car ever had a chance to move. */
  if ((trip == 0x01u) && in->tail_back >= SAFE_STOP_MM &&
      (!s_sf.vB || s_sf.dB >= SAFE_STOP_MM) &&
      ((s_mode == APP_MODE_REV_EDGE &&
        (!s_sf.vL || s_sf.dL >= SAFE_STOP_MM) && (!s_sf.vR || s_sf.dR >= SAFE_STOP_MM)) ||
       ((s_mode == APP_MODE_DRIVE) && (s_drive < 0))))
  {
    for (w = 0u; w < 4u; w++)
    {
      s16 p = out->pwm[w];
      if (p >  SAFE_ESCAPE_PWM) p =  SAFE_ESCAPE_PWM;
      if (p < -SAFE_ESCAPE_PWM) p = -SAFE_ESCAPE_PWM;
      out->pwm[w] = p;
    }
    return;                       /* keep the mode so the escape can finish */
  }

  /* Stop immediately and drop back to IDLE.  The fault is NOT latched: once
     the obstruction is gone a new command may move again, which is the
     behaviour the old safety_restart_ok() emulated. */
  for (w = 0u; w < 4u; w++) out->pwm[w] = 0;
  drive_zero();
  s_integ[0] = s_integ[1] = s_integ[2] = s_integ[3] = 0;
  s_pwm[0]   = s_pwm[1]   = s_pwm[2]   = s_pwm[3]   = 0;

  if (s_mode == APP_MODE_FSM2) Fsm2_Fail((trip & 2u) ? F2_STALLED : F2_OBSTRUCTION);
  s_mode = APP_MODE_IDLE;
}

u8 AppControl_SafetyFlags(void)
{
  return s_safety;
}

/* ----------------------------------------------------------------------- */
/*  command handling                                                       */
/* ----------------------------------------------------------------------- */
void App_PostCmd(AppCmd c)
{
  /* While the board is still settling after reset, a bouncing key or a glitch
     on an idle UART must not be able to start the car.  Stop / centre are
     always honoured, everything that moves waits for the settle window. */
  if (s_armed == 0u)
  {
    if ((c != APP_CMD_STOP) && (c != APP_CMD_SERVO_CENTER) &&
        (c != APP_CMD_SERVO_P90) && (c != APP_CMD_SERVO_M90))
    {
      return;
    }
  }

  if (c==APP_CMD_STOP) {
    __atomic_store_n(&s_pending_cmd,(u8)c,__ATOMIC_RELEASE);
  } else {
    u8 prev=__atomic_load_n(&s_pending_cmd,__ATOMIC_ACQUIRE);
    while (prev!=APP_CMD_STOP &&
           !__atomic_compare_exchange_n(&s_pending_cmd,&prev,(u8)c,0,
                                        __ATOMIC_ACQ_REL,__ATOMIC_ACQUIRE)) { }
  }
}

static void handle_cmd(AppCmd c, const AppInputs *in)
{
  if (c == APP_CMD_SERVO_P6 || c == APP_CMD_SERVO_M6) {
    if (s_mode == APP_MODE_IDLE) set_steer_all(c == APP_CMD_SERVO_P6 ? 6.0f : -6.0f);
    return;
  }
  if (s_mode==APP_MODE_FSM2) {
    if (c==APP_CMD_FSM2) return;
    Fsm2_Abort();
    all_zero();
    s_mode=APP_MODE_IDLE;
  }
  if (c == APP_CMD_EDGE || c == APP_CMD_FSM2 || c == APP_CMD_REV_EDGE ||
      c == APP_CMD_CRAB_L || c == APP_CMD_CRAB_R) {
    s_run_tail = in->tail_fresh;
    s_tail_wait = 0;
  }
  switch (c)
  {
    case APP_CMD_STOP:
      s_mode = APP_MODE_IDLE;
      all_zero();
      Fsm2_Abort();
      App_SetServoCenter();
      break;

    case APP_CMD_FWD:
      s_mode  = APP_MODE_DRIVE;
      s_drive = 1;
      s_diff_i = 0.0f;
      App_SetServoCenter();
      IMU_ResetYaw();
      break;

    case APP_CMD_BACK:
      s_mode  = APP_MODE_DRIVE;
      s_drive = -1;
      s_diff_i = 0.0f;
      App_SetServoCenter();
      IMU_ResetYaw();
      break;

    case APP_CMD_EDGE:
      s_mode = APP_MODE_EDGE;
      s_test_wall=(EntryWall){0};
      s_test_wall_last_ok=in->ms;
      s_test_origin=App_GetDistanceCm();
      s_test_ms=in->ms;
      s_diff_i  = 0.0f;
      IMU_ResetYaw();
      App_SetServoCenter();
      break;

    case APP_CMD_FSM2:
      Fsm2_Start();
      s_mode     = APP_MODE_FSM2;

      s_diff_i   = 0.0f;

      IMU_ResetYaw();
      break;

    case APP_CMD_CRAB_R:
    case APP_CMD_CRAB_L:
      s_mode      = APP_MODE_CRAB;
      s_crab_dir  = (s16)((c == APP_CMD_CRAB_R) ? 1 : -1);
      s_test_origin=App_GetDistanceCm();
      s_test_ms=in->ms;
      set_steer_all(90*s_crab_dir);
      IMU_ResetYaw();
      break;

    case APP_CMD_REV_EDGE:
      s_mode       = APP_MODE_REV_EDGE;
      s_test_origin= App_GetDistanceCm();
      s_test_ms    = in->ms;
      App_SetServoCenter();
      IMU_ResetYaw();
      break;

    case APP_CMD_SERVO_CENTER: App_SetServoCenter(); break;
    case APP_CMD_SERVO_P90:    set_steer_all(90.0f);  break;
    case APP_CMD_SERVO_M90:    set_steer_all(-90.0f); break;

    case APP_CMD_WHEELTEST:
      s_mode        = APP_MODE_WHEELTEST;
      s_wheel_idx   = 0u;
      s_wheel_on_ms = in->ms;
      s_wheel_peak[0] = s_wheel_peak[1] = s_wheel_peak[2] = s_wheel_peak[3] = 0;
      all_zero();
      App_SetServoCenter();
      break;

    default:
      break;
  }
}

/* ----------------------------------------------------------------------- */
/*  public API                                                             */
/* ----------------------------------------------------------------------- */
void AppControl_Init(void)
{
  u8 w;

  s_mode        = APP_MODE_IDLE;
  s_pending_cmd = (u8)APP_CMD_NONE;
  s_base_spd    = SPD_DEFAULT;
  s_drive       = 0;
  s_odom        = 0;
  s_prev_encoder_ms = 0;
  s_diff_i      = 0.0f;
  s_steer_dirty = 1u;
  s_safety      = 0u;
  s_armed       = 0u;
  s_boot_ms     = BSP_Ms();

  for (w = 0u; w < 4u; w++)
  {
    s_tgt[w]      = 0;
    s_mtgt[w]     = 0;
    s_meas[w]     = 0;
    s_prev_mag[w] = 0;
    s_pwm[w]      = 0;
    s_integ[w]    = 0;
    s_steer10[w]  = 0;
    s_applied[w]  = 0;
  }

  Fsm2_Init();
  Safety_Init();
  App_SetServoCenter();
}

AppMode AppControl_Mode(void)
{
  return s_mode;
}

void AppControl_Tick(AppInputs *in, AppOutputs *out)
{
  u8 w;
  int wheel_override=-1;
  out->heading=(HeadingLog){.param_revision=Tune_Revision()};

  /* Open the motion gate once the board has settled after reset. */
  if ((s_armed == 0u) && ((u32)(BSP_Ms() - s_boot_ms) >= BOOT_SETTLE_MS))
  {
    s_armed = 1u;
  }

  /* 0. Motor driver enable, re-sampled EVERY tick.  The hardware only drives
        the motors while PD3 is high, so if it drops mid-run we must give up
        the mode immediately and command a full brake. */
  s_en_ok = Board_MotorEnabled();
  if (s_en_ok == 0u)
  {
    s_pending_cmd = (u8)APP_CMD_NONE;   /* discard, do not queue */
    if (s_mode == APP_MODE_FSM2) Fsm2_Abort();
    s_mode = APP_MODE_IDLE;
    all_zero();
    for (w = 0u; w < 4u; w++)
    {
      s_integ[w] = 0;
      s_pwm[w]   = 0;
      s_mtgt[w]  = 0;
    }
  }

  /* 1. commands from the UI / serial side */
  if ((s_en_ok != 0u) && (s_pending_cmd != (u8)APP_CMD_NONE))
  {
    AppCmd c=(AppCmd)__atomic_exchange_n(&s_pending_cmd,APP_CMD_NONE,__ATOMIC_ACQ_REL);
    handle_cmd(c, in);
  }

  /* 2. mode logic -> s_tgt / steering */
  int guided = s_mode==APP_MODE_FSM2 || s_mode==APP_MODE_EDGE ||
               s_mode==APP_MODE_CRAB || s_mode==APP_MODE_REV_EDGE;
  in->use_tail = guided && s_run_tail;
  /* A run which began with the rear board must not silently lose its clearance
     protection. A fresh 2000 (no echo) is distinct from a missing sample. */
  int tail_missing = in->use_tail && (!in->tail_fresh || (in->tail_flags & 0x16u) != 0x16u);
  if (!tail_missing) s_tail_wait = 0;
  if (tail_missing) {
    all_zero();
    if (!s_tail_wait) { s_tail_wait = 1; s_tail_lost_ms = in->ms; }
    if (in->ms - s_tail_lost_ms >= 400u) {
      if (s_mode == APP_MODE_FSM2) Fsm2_Fail(F2_MISSING_RANGE);
      s_mode = APP_MODE_IDLE;
    }
  } else switch (s_mode)
  {
    case APP_MODE_FSM2:
      Fsm2_Tick(in,App_GetDistanceCm(),&s_f2out);
      if (s_f2out.hold || s_f2out.action==F2A_STOP) {
        all_zero();
        set_steer_all(s_f2out.steer_deg);
      } else if (s_f2out.action==F2A_FWD) {
        longitudinal(in,s_f2out.forward_speed < s_f2out.speed_limit ? s_f2out.forward_speed : s_f2out.speed_limit,
                     s_f2out.steer_deg,s_f2out.steer_trim_deg,&out->heading);
      } else if (s_f2out.action==F2A_REV) {
        float trim;
        float angle = reverse_wall_angle(in, s_f2out.wall_left, 0, &trim);
        longitudinal(in,-s_f2out.speed_limit,angle,trim,&out->heading);
      } else {
        crab_step(in,s_f2out.action==F2A_CRAB_R ? 1 : -1,
                  s_f2out.speed_limit,s_f2out.track_ridge,&out->heading);
      }
      if (Fsm2_GetState()==F2_FAULT || Fsm2_GetState()==F2_DONE) s_mode=APP_MODE_IDLE;
      break;

    case APP_MODE_EDGE:
    case APP_MODE_CRAB:
    {
      int edge=s_mode==APP_MODE_EDGE;
      u16 d=edge ? in->dF : RouteRange_Side(in,s_crab_dir>0 ? LW_RIGHT : LW_LEFT,1).mm;
      u16 stop=edge ? app_tune->entry_stop_mm : SIDE_CENTER_MM;
      u16 cm=(u16)(App_GetDistanceCm()-s_test_origin);
      /* One missing front sample must not end the test: near 2 m that channel
         is over-range and drops out now and then, which used to stop a run
         after a few hundred millimetres.  Only a sustained loss is fatal. */
      int d_lost = (d==BSP_DIST_INVALID);
      if (d_lost) {
        if (!s_edge_lost) { s_edge_lost=1u; s_edge_lost_ms=in->ms; }
      } else {
        s_edge_lost=0u;
        s_edge_last_ok=in->ms;
      }
      /* Exit reasons are printed once.  Without them a run that stops after a
         metre looks identical whether the ridge, the link, the IMU, the travel
         cap or a lost rail ended it, and every diagnosis becomes a guess. */
      if (!in->laser_fresh) {
        s_mode=APP_MODE_IDLE; all_zero(); App_SetServoCenter();
        App_Puts("\r\nEDGE EXIT laser\r\n");
      } else if (!in->imu_ok) {
        s_mode=APP_MODE_IDLE; all_zero(); App_SetServoCenter();
        App_Puts("\r\nEDGE EXIT imu\r\n");
      } else if (!d_lost && d<=stop) {
        s_mode=APP_MODE_IDLE; all_zero(); App_SetServoCenter();
        App_Puts("\r\nEDGE EXIT front\r\n");
      } else if (cm>(edge ? (u16)app_tune->entry_max_cm : CRAB_TRAVEL_CM+20u)) {
        s_mode=APP_MODE_IDLE; all_zero(); App_SetServoCenter();
        App_Puts("\r\nEDGE EXIT cm\r\n");
      } else if (in->ms-s_test_ms>60000u) {
        s_mode=APP_MODE_IDLE; all_zero(); App_SetServoCenter();
        App_Puts("\r\nEDGE EXIT timeout\r\n");
      } else if (d_lost && (in->ms-s_edge_last_ok)>=400u) {
        s_mode=APP_MODE_IDLE; all_zero(); App_SetServoCenter();
        App_Puts("\r\nEDGE EXIT glost\r\n");
      } else if (d_lost) {
        all_zero(); /* Hold the individual wheel angles during a brief range dropout. */
      } else if (in->ms-s_test_ms<ROUTE_SETTLE_MS) {
        all_zero(); set_steer_all(edge ? 0 : 90*s_crab_dir);
        s_test_wall_last_ok=in->ms;
      } else if (edge) {
        EntryWallCommand wall=EntryWall_Update(&s_test_wall,in,cm,app_tune->entry_cruise_counts,
                                               app_tune->entry_cruise_counts);
        if (!wall.speed) {
          all_zero(); set_steer_all(0);
          if (in->ms-s_test_wall_last_ok>=400u) {
            s_mode=APP_MODE_IDLE;
            App_Puts("\r\nEDGE EXIT wall\r\n");
          }
        } else {
          s_test_wall_last_ok=in->ms;
          if (wall.coast) {
            drive_zero();
            apply_steer_split(wall.angle, wall.yaw_trim_deg);
          } else {
            s16 speed=wall.speed;
            if (d<stop+app_tune->entry_e_slow_mm && speed>app_tune->entry_creep_counts) speed=app_tune->entry_creep_counts;
            longitudinal(in,speed,wall.angle,wall.yaw_trim_deg,&out->heading);
          }
        }
      }
      else crab_step(in,s_crab_dir,d<stop+200u ? ROUTE_CREEP_COUNTS : SHIFT_SPD,1,&out->heading);
      break;
    }

    case APP_MODE_REV_EDGE:
    {
      /* Back out along the left wall so the tuning loop can repeat E without
         anyone touching the car.  The caller passes the distance to cover. */
      u16 cm=(u16)(App_GetDistanceCm()-s_test_origin);
      if (!in->laser_fresh || !in->imu_ok || cm>=s_rev_target_cm ||
          in->ms-s_test_ms>90000u) {
        s_mode=APP_MODE_IDLE; all_zero(); App_SetServoCenter();
      } else if (in->ms-s_test_ms<ROUTE_SETTLE_MS) {
        all_zero(); set_steer_all(0);
      } else {
        float rev_trim;
        float rev_angle=reverse_wall_angle(in,1,1,&rev_trim);
        longitudinal(in,(s16)-REV_EDGE_SPD,rev_angle,rev_trim,&out->heading);
      }
      break;
    }

    case APP_MODE_WHEELTEST:
    {
      u32 elapsed = (u32)(in->ms - s_wheel_on_ms);

      if (elapsed>=1500u) {
        s_wheel_on_ms=in->ms;
        if (++s_wheel_idx>=4u) s_mode=APP_MODE_IDLE;
      } else {
        int mag=in->wheel_cnt[s_wheel_idx];
        if (mag<0) mag=-mag;
        if (mag>s_wheel_peak[s_wheel_idx] && mag<=32767) s_wheel_peak[s_wheel_idx]=(s16)mag;
        wheel_override=s_wheel_idx;
      }
      all_zero();
      break;
    }

    case APP_MODE_DRIVE:
      if (s_drive != 0)
      {
        s_diff_i += app_tune->diff_ki * in->yaw_deg * ((float)APP_CTRL_MS / 1000.0f);
        if (s_diff_i >  app_tune->diff_max) s_diff_i =  app_tune->diff_max;
        if (s_diff_i < -app_tune->diff_max) s_diff_i = -app_tune->diff_max;
        float corr = app_tune->diff_kp * in->yaw_deg + s_diff_i +
                     app_tune->diff_kd * in->yaw_rate_dps;
        float bias = fminf(app_tune->diff_max, fmaxf(-app_tune->diff_max, app_tune->diff_ff));
        out->heading.raw = (int)(corr + bias);
        out->heading.correction = clamp_s16f(corr + bias, -app_tune->diff_max, app_tune->diff_max);
        /* Retain F/B intent when the operator temporarily sets speed to zero. */
        int dir = s_drive;
        drive_longitudinal(dir * s_base_spd, 0, 0, out->heading.correction - bias, bias);
        s_drive = dir;
      }
      break;

    case APP_MODE_IDLE:
    default:
      all_zero();
      break;
  }

  /* 3. odometry: mean absolute counts per wheel */
  {
    s32 acc = 0;

    for (w = 0u; w < 4u; w++)
    {
      int raw = (int)in->wheel_cnt[w];
      acc += (s32)((raw < 0) ? -raw : raw);
    }
    s_odom += acc / 4;
  }

  /* 4. wheel speed loop */
  speed_step(in, out);
  if (wheel_override>=0) out->pwm[wheel_override]=2500;

  /* 5. safety veto: may zero the output and drop back to IDLE */
  safety_eval(in, out);

  out->heading.mode=s_mode;
  out->heading.stage=Fsm2_GetState();
  for (w=0; w<4; ++w) {
    out->heading.target[w]=s_mtgt[w];
    out->heading.measured[w]=s_meas[w];
  }

  /* 6. steering */
  for (w = 0u; w < 4u; w++) out->steer_deg10[w] = s_steer10[w];
  out->steer_dirty = s_steer_dirty;
  s_steer_dirty = 0u;

  /* 7. remember what was actually commanded, for the 1 Hz log */
  for (w = 0u; w < 4u; w++) s_applied[w] = out->pwm[w];
}

void AppControl_GetPwm(s16 out[4])
{
  u8 w;

  for (w = 0u; w < 4u; w++) out[w] = s_applied[w];
}

u8 AppControl_Enabled(void)
{
  return s_en_ok;
}

/* ----------------------------------------------------------------------- */
/*  hooks used by fsm2 / safety / centering                                */
/* ----------------------------------------------------------------------- */
void App_SetWheelSpeed(const s16 spd_tgt[4])
{
  u8  w;
  s32 sum = 0;

  for (w = 0u; w < 4u; w++)
  {
    s_tgt[w] = spd_tgt[w];
    sum += (s32)spd_tgt[w];
  }

  if (sum > 0)       s_drive = 1;
  else if (sum < 0)  s_drive = -1;
  else               s_drive = 0;
}

void App_DriveStop(void)
{
  all_zero();
}

void App_SetSteerAll(float deg)
{
  set_steer_all(deg);
}

void App_SetSteer4(float fl, float fr, float rl, float rr)
{
  s_steer10[0] = clamp_s16f(fl * 10.0f, -900.0f, 900.0f);
  s_steer10[1] = clamp_s16f(fr * 10.0f, -900.0f, 900.0f);
  s_steer10[2] = clamp_s16f(rl * 10.0f, -900.0f, 900.0f);
  s_steer10[3] = clamp_s16f(rr * 10.0f, -900.0f, 900.0f);
  s_steer_dirty = 1u;
}

void App_SetServoCenter(void)
{
  set_steer_all(0.0f);
}

u16 App_GetDistanceCm(void)
{
  float cm = (float)s_odom * 3.14159f * WHEEL_DIAMETER_MM
             / (10.0f * ENCODER_CPR);
  return (u16)cm;
}

u32 App_MsNow(void)
{
  return BSP_Ms();
}

void App_SetBaseSpeed(s16 counts)
{
  if (counts < 0) counts = 0;
  if (counts > SPD_MAX) counts = SPD_MAX;
  s_base_spd = counts;
}

s16 App_GetBaseSpeed(void)
{
  return s_base_spd;
}
