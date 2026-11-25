/**
  * @file    centering.c
  * @brief   T3 / S2 - straight-line centering controller (4WD / 4WS).
  *
  *  Contract:  USER/app_if.h
  *    void Centering_Init(void);
  *    void Centering_Update(const LaserData *laser, float yaw_deg,
  *                          float yaw_rate_dps, s16 base_spd, s16 spd_tgt[4]);
  *  Call Centering_Update() from the 50 Hz control tick (every 20 ms), after
  *  the laser has been polled and before App_SetWheelSpeed().
  *
  *  Two decoupled loops:
  *
  *    lateral   e = (dL - dR) / 2  [cm]      e > 0 => car offset RIGHT
  *              e -> soft deadband -> LPF -> PD + I (anti-windup)
  *                -> delta [deg] on all four wheels (crab steer),
  *                   clamped to +-CENTER_DELTA_MAX (8 deg)
  *
  *    heading   yaw [deg], yaw_rate [deg/s]  yaw > 0 => nose LEFT
  *              yaw -> P + I , yaw_rate -> D (rate damping)
  *                -> Delta [deg] toe (front +Delta, rear -Delta),
  *                   clamped to +-CENTER_TOE_MAX (5 deg)
  *
  *    servos    App_SetSteer4(delta+Delta, delta+Delta, delta-Delta, delta-Delta)
  *
  *    speed     spd_tgt[0..3] = base_spd (the Lead runs a per-wheel loop);
  *              an optional bounded yaw-rate differential exists but is
  *              disabled by default (see CENTER_SPD_DIFF_KD in centering.h).
  *
  *  Safety behaviour:
  *    - laser->valid == 0, flags bit3 (timeout), or dL/dR equal to 0xFFFF,
  *      below 20 mm or above 4000 mm, on either side
  *        => delta = Delta = 0, all filters/integrators cleared,
  *           App_SetServoCenter() called once on the valid->invalid edge,
  *           spd_tgt[] = base_spd  (safe straight, no accumulation)
  *    - base_spd == 0 (car stopped by the FSM)
  *        => everything cleared and the servos are NOT touched, because the
  *           FSM / lane shift owns them while the car is standing still.
  *
  *  Reverse driving (base_spd < 0) is supported: the steering command is
  *  multiplied by -1 because the lateral/longitudinal coupling of a
  *  steered wheel reverses with the direction of travel.  Integrators are
  *  cleared when the direction of travel changes.
  *
  *  C90 / ASCII only (Keil ARMCC AC5, project is GBK).
  */

#include "centering.h"

/* sentinel value for "the servo angle we last sent is unknown" */
#define CF_SERVO_UNKNOWN   (999.0f)

/* ======================================================================= */
/*  module state                                                          */
/* ======================================================================= */
CenteringDbg g_centering;

static float s_e_filt;    /* low-pass filtered lateral error, cm            */
static float s_e_prev;    /* filtered error of the previous tick, cm        */
static float s_e_d;       /* filtered derivative of e, cm/s                 */
static float s_i_e;       /* lateral integrator, deg                        */
static float s_delta;     /* slew limited crab command, deg (forward frame) */
static float s_i_yaw;     /* heading integrator, deg                        */
static float s_toe;       /* slew limited toe command, deg (forward frame)  */

static u8  s_active;      /* 1 = we are driving and steering the servos     */
static s8  s_dir;         /* sign of base_spd of the previous tick          */

/* last command actually pushed onto the servo bus, deg per wheel */
static float s_out_fl;
static float s_out_fr;
static float s_out_rl;
static float s_out_rr;

/* ======================================================================= */
/*  small helpers (no libm dependency)                                    */
/* ======================================================================= */
static float cf_abs(float x)
{
  return (x < 0.0f) ? -x : x;
}

static float cf_limit(float x, float lim)
{
  if (x >  lim) return  lim;
  if (x < -lim) return -lim;
  return x;
}

/* fold an angle into -180..+180 (the IMU yaw is a free running integral) */
static float cf_wrap180(float d)
{
  while (d >  180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

/* 0xFFFF, 0, too close or too far all mean "no usable wall echo" */
static u8 cf_side_ok(u16 mm)
{
  if (mm == 0xFFFFu)              return 0u;
  if (mm <  (u16)CENTER_SIDE_MIN_MM) return 0u;
  if (mm >  (u16)CENTER_SIDE_MAX_MM) return 0u;
  return 1u;
}

static void cf_reset(void)
{
  s_e_filt = 0.0f;
  s_e_prev = 0.0f;
  s_e_d    = 0.0f;
  s_i_e    = 0.0f;
  s_delta  = 0.0f;
  s_i_yaw  = 0.0f;
  s_toe    = 0.0f;
}

/* mark the servo knowledge as unknown so the next command is always sent */
static void cf_forget(void)
{
  s_out_fl = CF_SERVO_UNKNOWN;
  s_out_fr = CF_SERVO_UNKNOWN;
  s_out_rl = CF_SERVO_UNKNOWN;
  s_out_rr = CF_SERVO_UNKNOWN;
}

/* Push the four wheel angles, but only when they really changed: a full
 * HTS-35H position frame is ~10 bytes and the servo UART is bit-banged, so
 * re-sending four frames every 20 ms would waste ~3 ms of CPU per tick. */
static void cf_steer(float delta, float toe)
{
  float fl = delta + toe;
  float fr = delta + toe;
  float rl = delta - toe;
  float rr = delta - toe;

  if ((cf_abs(fl - s_out_fl) < CENTER_STEER_EPS_DEG) &&
      (cf_abs(fr - s_out_fr) < CENTER_STEER_EPS_DEG) &&
      (cf_abs(rl - s_out_rl) < CENTER_STEER_EPS_DEG) &&
      (cf_abs(rr - s_out_rr) < CENTER_STEER_EPS_DEG))
  {
    return;
  }

  s_out_fl = fl;
  s_out_fr = fr;
  s_out_rl = rl;
  s_out_rr = rr;
  App_SetSteer4(fl, fr, rl, rr);
}

static void cf_straight(s16 base_spd, s16 spd_tgt[4])
{
  spd_tgt[0] = base_spd;
  spd_tgt[1] = base_spd;
  spd_tgt[2] = base_spd;
  spd_tgt[3] = base_spd;
}

/* ======================================================================= */
/*  public API                                                            */
/* ======================================================================= */
void Centering_Init(void)
{
  cf_reset();
  s_active = 0u;
  s_dir    = 0;

  s_out_fl = 0.0f;
  s_out_fr = 0.0f;
  s_out_rl = 0.0f;
  s_out_rr = 0.0f;

  g_centering.e_cm       = 0.0f;
  g_centering.delta_deg  = 0.0f;
  g_centering.toe_deg    = 0.0f;
  g_centering.valid      = 0u;
  g_centering.saturated  = 0u;

  /* requires the servo UART to be up already (call after uart2_init) */
  App_SetServoCenter();
}

void Centering_Update(const LaserData *laser, float yaw_deg, float yaw_rate_dps,
                      s16 base_spd, s16 spd_tgt[4])
{
  float e_raw;
  float e_use;
  float d_e;
  float p;
  float d;
  float out;
  float i_new;
  float yaw_err;
  float yaw_rate;
  float delta;
  float toe;
  s16   dif = 0;
  s16   dif_lim;
  u8    valid;
  u8    sat_e;
  u8    sat_y;
  s8    dir;

  if (spd_tgt == 0)
  {
    return;
  }

  /* ---- 0. direction of travel ------------------------------------- */
  dir = (base_spd > 0) ? (s8)1 : ((base_spd < 0) ? (s8)-1 : (s8)0);
  if (dir != s_dir)
  {
    /* steering geometry mirrors with the direction of travel: do not carry
     * an integrator or a slew-limited angle across the change */
    s_i_e   = 0.0f;
    s_i_yaw = 0.0f;
    s_delta = 0.0f;
    s_toe   = 0.0f;
    s_dir   = dir;
  }

  /* ---- 1. laser validity ------------------------------------------ */
  valid = 0u;
  if ((laser != 0) && (laser->valid != 0u) && ((laser->flags & 0x08u) == 0u))
  {
    if ((cf_side_ok(laser->dL) != 0u) && (cf_side_ok(laser->dR) != 0u))
    {
      valid = 1u;
    }
  }
  g_centering.valid = valid;

  /* ---- 2. car stopped: release the servos, accumulate nothing ----- */
  if (dir == 0)
  {
    cf_reset();
    s_active = 0u;

    /* we no longer know where the servos point (the FSM owns them); force a
     * refresh of all four frames when the car starts moving again */
    cf_forget();

    cf_straight(base_spd, spd_tgt);

    g_centering.delta_deg = 0.0f;
    g_centering.toe_deg   = 0.0f;
    g_centering.saturated = 0u;
    return;
  }

  /* ---- 3. no side lasers: straight ahead, do not accumulate ------- */
  if (valid == 0u)
  {
    cf_reset();
    if (s_active != 0u)
    {
      /* exactly once on the valid -> invalid edge */
      App_SetServoCenter();
      s_out_fl = 0.0f;
      s_out_fr = 0.0f;
      s_out_rl = 0.0f;
      s_out_rr = 0.0f;
    }
    s_active = 0u;

    cf_straight(base_spd, spd_tgt);

    g_centering.e_cm      = 0.0f;
    g_centering.delta_deg = 0.0f;
    g_centering.toe_deg   = 0.0f;
    g_centering.saturated = 0u;
    return;
  }

  /* ================================================================= */
  /*  4. lateral loop: e [cm] -> delta [deg]                           */
  /* ================================================================= */
  /* (dL - dR) mm / 10 = cm, / 2 = e  =>  / 20 */
  e_raw = ((float)laser->dL - (float)laser->dR) * 0.05f;
  e_raw = cf_limit(e_raw, CENTER_E_MAX_CM);

  /* first order low pass on the distance measurement */
  s_e_filt += CENTER_E_FILT_ALPHA * (e_raw - s_e_filt);

  /* derivative of the filtered error, cm/s, low passed again */
  d_e = (s_e_filt - s_e_prev) / CENTER_DT_S;
  s_e_prev = s_e_filt;
  s_e_d += CENTER_E_D_ALPHA * (d_e - s_e_d);

  /* soft deadband (continuous, no jump at the edge) */
  e_use = s_e_filt;
  if (e_use > CENTER_E_DEADBAND_CM)
  {
    e_use -= CENTER_E_DEADBAND_CM;
  }
  else if (e_use < -CENTER_E_DEADBAND_CM)
  {
    e_use += CENTER_E_DEADBAND_CM;
  }
  else
  {
    e_use = 0.0f;
  }

  p = CENTER_E_KP * e_use;
  d = (e_use != 0.0f) ? (CENTER_E_KD * s_e_d) : 0.0f;

  i_new = s_i_e + (CENTER_E_KI * e_use) * CENTER_DT_S;
  i_new = cf_limit(i_new, CENTER_E_I_MAX);

  out  = p + d + i_new;
  sat_e = 0u;
  if (out >  CENTER_DELTA_MAX) { out =  CENTER_DELTA_MAX; sat_e = 1u; }
  if (out < -CENTER_DELTA_MAX) { out = -CENTER_DELTA_MAX; sat_e = 1u; }
  if (sat_e != 0u)
  {
    /* back calculation anti windup: keep the state consistent with the
     * clamped output instead of letting it run away */
    i_new = cf_limit(out - p - d, CENTER_E_I_MAX);
  }
  s_i_e = i_new;

  /* slew limit - applied after the clamp, so the clamp always wins */
  if (out > s_delta + CENTER_DELTA_SLEW) out = s_delta + CENTER_DELTA_SLEW;
  if (out < s_delta - CENTER_DELTA_SLEW) out = s_delta - CENTER_DELTA_SLEW;
  s_delta = out;

  /* ================================================================= */
  /*  5. heading loop: yaw [deg] -> toe Delta [deg]                    */
  /* ================================================================= */
  yaw_err  = cf_wrap180(CENTER_YAW_SIGN * yaw_deg);
  yaw_rate = CENTER_YAW_SIGN * yaw_rate_dps;
  yaw_err  = cf_limit(yaw_err, CENTER_YAW_MAX_DEG);

  i_new = s_i_yaw + (CENTER_YAW_KI * yaw_err) * CENTER_DT_S;
  i_new = cf_limit(i_new, CENTER_YAW_I_MAX);

  /* yaw > 0 (nose left) must produce Delta < 0 (yaw right) */
  out  = -(CENTER_YAW_KP * yaw_err + CENTER_YAW_KD * yaw_rate + i_new);
  sat_y = 0u;
  if (out >  CENTER_TOE_MAX) { out =  CENTER_TOE_MAX; sat_y = 1u; }
  if (out < -CENTER_TOE_MAX) { out = -CENTER_TOE_MAX; sat_y = 1u; }
  if (sat_y != 0u)
  {
    i_new = cf_limit(-out - CENTER_YAW_KP * yaw_err
                          - CENTER_YAW_KD * yaw_rate, CENTER_YAW_I_MAX);
  }
  s_i_yaw = i_new;

  if (out > s_toe + CENTER_TOE_SLEW) out = s_toe + CENTER_TOE_SLEW;
  if (out < s_toe - CENTER_TOE_SLEW) out = s_toe - CENTER_TOE_SLEW;
  s_toe = out;

  /* ================================================================= */
  /*  6. wheel angles (mirrored when reversing)                        */
  /* ================================================================= */
  delta = (CENTER_STEER_SIGN * (float)dir) * s_delta;
  toe   = (CENTER_TOE_SIGN   * (float)dir) * s_toe;
  cf_steer(delta, toe);
  s_active = 1u;

  /* ================================================================= */
  /*  7. per-wheel speed targets                                       */
  /* ================================================================= */
  /* resist the yaw rate with a bounded differential: a positive yaw rate
   * (turning left) slows the right hand wheels down.  CENTER_SPD_DIFF_KD is
   * 0 by default, so dif stays 0 unless the Lead enables it. */
  dif = (s16)(-CENTER_SPD_DIFF_KD * yaw_rate);
  dif_lim = (s16)(CENTER_SPD_DIFF_MAX *
                  ((base_spd >= 0) ? (float)base_spd : -(float)base_spd));
  if (dif >  dif_lim) dif =  dif_lim;
  if (dif < -dif_lim) dif = -dif_lim;

  spd_tgt[0] = (s16)(base_spd - dif);   /* FL, left side  */
  spd_tgt[1] = (s16)(base_spd + dif);   /* FR, right side */
  spd_tgt[2] = (s16)(base_spd - dif);   /* RL, left side  */
  spd_tgt[3] = (s16)(base_spd + dif);   /* RR, right side */

  /* ================================================================= */
  /*  8. debug snapshot for the Lead's log                             */
  /* ================================================================= */
  g_centering.e_cm      = s_e_filt;
  g_centering.delta_deg = delta;
  g_centering.toe_deg   = toe;
  g_centering.saturated = (u8)(sat_e | sat_y);
}
