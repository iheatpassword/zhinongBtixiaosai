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
#include "app_if.h"
#include "app_config.h"
#include "app_tasks.h"

#include "fsm2.h"
#include "entry_wall.h"
#include "safety.h"

#include "board.h"
#include "motor/motor.h"
#include "servo/servo.h"
#include "imu/imu.h"

/* ----------------------------------------------------------------------- */
/*  state                                                                  */
/* ----------------------------------------------------------------------- */
static AppMode s_mode;
static volatile u8 s_pending_cmd;
static s16 s_base_spd;

/* speed loop */
static s16 s_tgt[4];            /* desired counts per 20 ms, signed        */
static s16 s_mtgt[4];           /* magnitude actually commanded           */
static s16 s_meas[4];
static s16 s_prev_mag[4];
static s16 s_pwm[4];
static s32 s_integ[4];
static s8  s_drive;             /* 0 stop, +1 forward, -1 reverse          */

/* steering */
static s16 s_steer10[4];
static u8  s_steer_dirty;

/* odometry */
static s32 s_odom;

/* heading trim */
static float s_diff_i;

static u16 s_test_origin;
static u32 s_test_ms;
static EntryWall s_test_wall;
static u32 s_test_wall_last_ok;
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
  s16 next=clamp_s16f(deg*10.0f,-(90+CRAB_TRIM_MAX_DEG)*10,(90+CRAB_TRIM_MAX_DEG)*10);
  if (next==s_steer10[0] && next==s_steer10[1] && next==s_steer10[2] && next==s_steer10[3]) return;
  s_steer10[0]=next;
  s_steer10[1] = s_steer10[0];
  s_steer10[2] = s_steer10[0];
  s_steer10[3] = s_steer10[0];
  s_steer_dirty = 1u;
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

static float reverse_wall_angle(const AppInputs *in, int left_wall)
{
  u16 d=left_wall ? in->dL : in->dR;
  float angle=0;
  if (d<300u) {
    float error=(float)d-SIDE_CENTER_MM;
    angle=(left_wall ? 1.0f : -1.0f)*EDGE_KP*error;
  }
  if (angle>EDGE_MAX) angle=EDGE_MAX;
  if (angle<-EDGE_MAX) angle=-EDGE_MAX;
  return angle;
}

static void longitudinal(const AppInputs *in, s16 speed, float angle)
{
  int dir=speed<0 ? -1 : 1;
  /* Forward feed-forward was measured at EDGE_SPD. Keep its wheel-speed
     ratio at the slower entry speeds; reverse retains its existing trim. */
  float feedforward=speed>0 ? DIFF_FF*(float)speed/EDGE_SPD : -DIFF_FF;
  float corr=feedforward + YAW_DRIVE_KP*in->yaw_deg + YAW_DRIVE_KD*in->yaw_rate_dps;
  corr=clamp_s16f(corr,-150,150);
  set_steer_all(angle);
  s_tgt[0]=s_tgt[2]=speed+(s16)corr;
  s_tgt[1]=s_tgt[3]=speed-(s16)corr;
  s_drive=dir;
}

static void crab_step(const AppInputs *in, int dir, s16 speed, int tracking)
{
  float error=0;
  int f=tracking && in->vF && in->dF>=70 && in->dF<=380;
  int b=tracking && in->vB && in->dB>=40 && in->dB<=300;
  if (f && b) {
    int sum=(int)in->dF+in->dB-(int)(ENTRY_STOP_MM+REAR_TARGET_MM);
    if (sum>=-(int)RIDGE_SUM_TOL_MM && sum<=(int)RIDGE_SUM_TOL_MM)
      error=0.5f*((float)in->dF-ENTRY_STOP_MM+REAR_TARGET_MM-in->dB);
  } else if (f) error=(float)in->dF-ENTRY_STOP_MM;
  else if (b) error=(float)REAR_TARGET_MM-in->dB;
  float trim=0.10f*error;
  if (trim>CRAB_TRIM_MAX_DEG) trim=CRAB_TRIM_MAX_DEG;
  if (trim<-CRAB_TRIM_MAX_DEG) trim=-CRAB_TRIM_MAX_DEG;
  set_steer_all(dir*(90.0f-trim));
  s16 cy=dir*SHIFT_YAW_SPD_SIGN*clamp_s16f(
      SHIFT_YAW_SPD_KP*in->yaw_deg+SHIFT_YAW_SPD_KD*in->yaw_rate_dps,
      -SHIFT_YAW_SPD_MAX,SHIFT_YAW_SPD_MAX);
  s_tgt[0]=s_tgt[1]=speed-cy;
  s_tgt[2]=s_tgt[3]=speed+cy;
  s_drive=1;
}

/* ----------------------------------------------------------------------- */
/*  wheel speed loop                                                       */
/* ----------------------------------------------------------------------- */
static void speed_step(const AppInputs *in, AppOutputs *out)
{
  u8 w;

  for (w = 0u; w < 4u; w++)
  {
    int raw = (int)in->wheel_cnt[w];
    int mag = (raw < 0) ? -raw : raw;
    s32 e;
    s32 p;

    if (mag > 32000) mag = 32000;

    s_meas[w]     = (s16)((mag + (int)s_prev_mag[w]) / 2);
    s_prev_mag[w] = (s16)mag;

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
    s_integ[w] += e / KI_DIV;
    if (s_integ[w] >  INTEG_MAX) s_integ[w] =  INTEG_MAX;
    if (s_integ[w] < -INTEG_MAX) s_integ[w] = -INTEG_MAX;

    p = (s32)(KP_NUM * e) / 10 + s_integ[w] + PWM_FF + ((s32)s_mtgt[w] * 25) / 10;
    if (p > PWM_LIM) p = PWM_LIM;
    if (p < 0)       p = 0;

    if (p > (s32)s_pwm[w] + PWM_SLEW) p = (s32)s_pwm[w] + PWM_SLEW;
    if (p < (s32)s_pwm[w] - PWM_SLEW) p = (s32)s_pwm[w] - PWM_SLEW;
    if (p < 0) p = 0;

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
  if (s_mode==APP_MODE_FSM2) {
    if (c==APP_CMD_FSM2) return;
    Fsm2_Abort();
    all_zero();
    s_mode=APP_MODE_IDLE;
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
      s_tgt[0] = s_base_spd; s_tgt[1] = s_base_spd;
      s_tgt[2] = s_base_spd; s_tgt[3] = s_base_spd;
      App_SetServoCenter();
      IMU_ResetYaw();
      break;

    case APP_CMD_BACK:
      s_mode  = APP_MODE_DRIVE;
      s_drive = -1;
      s_diff_i = 0.0f;
      s_tgt[0] = (s16)-s_base_spd; s_tgt[1] = (s16)-s_base_spd;
      s_tgt[2] = (s16)-s_base_spd; s_tgt[3] = (s16)-s_base_spd;
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

void AppControl_Tick(const AppInputs *in, AppOutputs *out)
{
  u8 w;
  int wheel_override=-1;

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
  switch (s_mode)
  {
    case APP_MODE_FSM2:
      Fsm2_Tick(in,App_GetDistanceCm(),&s_f2out);
      if (s_f2out.hold || s_f2out.action==F2A_STOP) {
        all_zero();
        set_steer_all(s_f2out.steer_deg);
      } else if (s_f2out.action==F2A_FWD) {
        longitudinal(in,s_f2out.slow ? ROUTE_CREEP_COUNTS : s_f2out.forward_speed,
                     s_f2out.steer_deg);
      } else if (s_f2out.action==F2A_REV) {
        s16 speed=s_f2out.slow ? ROUTE_CREEP_COUNTS : REV_SPD;
        longitudinal(in,-speed,reverse_wall_angle(in,s_f2out.wall_left));
      } else {
        crab_step(in,s_f2out.action==F2A_CRAB_R ? 1 : -1,
                  s_f2out.slow ? ROUTE_CREEP_COUNTS : SHIFT_SPD,s_f2out.track_ridge);
      }
      if (Fsm2_GetState()==F2_FAULT || Fsm2_GetState()==F2_DONE) s_mode=APP_MODE_IDLE;
      break;

    case APP_MODE_EDGE:
    case APP_MODE_CRAB:
    {
      int edge=s_mode==APP_MODE_EDGE;
      u16 d=edge ? in->dF : s_crab_dir>0 ? in->dR : in->dL;
      u16 stop=edge ? ENTRY_STOP_MM : SIDE_CENTER_MM;
      u16 cm=(u16)(App_GetDistanceCm()-s_test_origin);
      if (!in->laser_fresh || !in->imu_ok || d==BSP_DIST_INVALID ||
          d<=stop || cm>(edge ? 490u : CRAB_TRAVEL_CM+20u) ||
          in->ms-s_test_ms>60000u) {
        s_mode=APP_MODE_IDLE; all_zero(); App_SetServoCenter();
      } else if (in->ms-s_test_ms<ROUTE_SETTLE_MS) {
        all_zero(); set_steer_all(edge ? 0 : 90*s_crab_dir);
        s_test_wall_last_ok=in->ms;
      } else if (edge) {
        EntryWallCommand wall=EntryWall_Update(&s_test_wall,in,cm);
        if (!wall.speed) {
          all_zero(); set_steer_all(0);
          if (in->ms-s_test_wall_last_ok>=400u) s_mode=APP_MODE_IDLE;
        } else {
          s_test_wall_last_ok=in->ms;
          longitudinal(in,d<stop+200u ? ROUTE_CREEP_COUNTS : wall.speed,wall.angle);
        }
      }
      else crab_step(in,s_crab_dir,d<stop+200u ? ROUTE_CREEP_COUNTS : SHIFT_SPD,1);
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
        s_diff_i += DIFF_KI * in->yaw_deg * ((float)APP_CTRL_MS / 1000.0f);
        if (s_diff_i >  DIFF_MAX) s_diff_i =  DIFF_MAX;
        if (s_diff_i < -DIFF_MAX) s_diff_i = -DIFF_MAX;
        /* plain drive keeps the constant trim so a straight run stays straight */
        for (w = 0u; w < 4u; w++)
        {
          float base = (float)((s_drive > 0) ? s_base_spd : -s_base_spd);
          float corr = DIFF_FF + DIFF_KP * in->yaw_deg + s_diff_i +
                       DIFF_KD * in->yaw_rate_dps;
          float v;
          if (corr >  DIFF_MAX) corr =  DIFF_MAX;
          if (corr < -DIFF_MAX) corr = -DIFF_MAX;
          v = (w == 1u || w == 3u) ? (base - corr) : (base + corr);
          s_tgt[w] = (s16)v;
        }
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
