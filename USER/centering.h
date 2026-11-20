/**
  * @file    centering.h
  * @brief   T3 / S2 - straight-line centering for the C30D 4WD/4WS car.
  *
  *          4 wheel independent steering, 4 wheel drive.
  *          Lane: the car drives along a 40 cm furrow between two walls,
  *          one laser each side (dL, dR) plus one looking forward (dF).
  *
  *          The controller produces two steering angles per tick:
  *            delta : same angle on all four wheels (crab steer).
  *                    The whole car translates sideways, heading unchanged.
  *                    Driven by the lateral laser error e = (dL-dR)/2.
  *            Delta : front wheels +Delta, rear wheels -Delta (toe steer).
  *                    The two axles push opposite ways, the car yaws about
  *                    its centre, heading unchanged in translation.
  *                    Driven by the IMU yaw error and yaw rate.
  *
  *          Final servo command (exactly as specified in the task):
  *            App_SetSteer4(delta+Delta, delta+Delta, delta-Delta, delta-Delta)
  *
  * ------------------------------------------------------------------------
  *  SIGN CONVENTIONS - the three macros at the bottom of this block are the
  *  only things to flip if the hardware disagrees (one-line change each).
  *    positive steering angle : wheel turned LEFT (CCW seen from above)
  *    positive Delta          : car yaws LEFT
  *    positive yaw_deg input  : nose is LEFT of the reference heading
  *    positive e              : left wall farther => car is RIGHT of centre
  *                              => must steer LEFT => delta > 0
  * ------------------------------------------------------------------------
  *
  *  Timing: Centering_Update() must be called every 20 ms (50 Hz).
  *          The integrators use a fixed CENTER_DT_S of 0.02 s and ignore
  *          real elapsed time, so do not call it at another rate.
  *
  *  C90 / ASCII only (Keil ARMCC AC5, project is GBK).
  */
#ifndef __CENTERING_H
#define __CENTERING_H

#include "app_if.h"

/* ======================================================================= */
/*  Tunable parameters - all distances cm, all angles deg, time in s       */
/* ======================================================================= */

/* ---- lateral loop: e [cm] -> delta [deg] ----------------------------- */
#define CENTER_E_DEADBAND_CM   0.30f  /* soft deadband half width, +-cm    */
#define CENTER_E_FILT_ALPHA    0.30f  /* LPF on e, 1.0 = no filtering      */
#define CENTER_E_D_ALPHA       0.25f  /* LPF on the derivative of e        */
#define CENTER_E_KP            2.20f  /* deg per cm                        */
#define CENTER_E_KI            0.10f  /* deg per (cm * s)                  */
#define CENTER_E_KD            0.20f  /* deg per (cm/s)                    */
#define CENTER_E_I_MAX         1.50f  /* integrator clamp, deg             */
#define CENTER_E_MAX_CM       20.00f  /* plausibility clamp on e, cm       */
#define CENTER_DELTA_MAX       8.00f  /* hard steering clamp, deg          */
#define CENTER_DELTA_SLEW      2.50f  /* max change per 20 ms tick, deg    */

/* ---- heading loop: yaw [deg] -> toe Delta [deg] ---------------------- */
#define CENTER_YAW_KP          1.20f  /* deg toe per deg yaw               */
#define CENTER_YAW_KI          0.12f  /* deg per (deg * s)                 */
#define CENTER_YAW_KD          0.35f  /* deg toe per (deg/s) rate damping  */
#define CENTER_YAW_I_MAX       2.00f  /* integrator clamp, deg             */
#define CENTER_YAW_MAX_DEG    60.00f  /* yaw error clamp before the PID,deg*/
#define CENTER_TOE_MAX         5.00f  /* hard toe clamp, deg               */
#define CENTER_TOE_SLEW        1.50f  /* max change per 20 ms tick, deg    */

/* ---- misc ------------------------------------------------------------ */
#define CENTER_DT_S            0.02f  /* control period, fixed 50 Hz       */
#define CENTER_SIDE_MIN_MM       20u  /* closer than 2 cm = bad reading    */
#define CENTER_SIDE_MAX_MM     4000u  /* farther than 4 m = out of range   */
#define CENTER_STEER_EPS_DEG    0.10f /* servo bus update threshold, deg   */

/*  Optional left/right speed differential on yaw rate.
 *  DEFAULT 0 (disabled): the Lead's speed loop already applies its own
 *  yaw differential (YAW_KP/YAW_KD, clamp +-150 counts) and toe steering is
 *  the primary heading actuator.  Set CENTER_SPD_DIFF_KD != 0 only after
 *  that differential has been removed from main.c, otherwise the two fight.
 *  Units: counts/20ms per (deg/s); result bounded to CENTER_SPD_DIFF_MAX of
 *  the base speed (so 0.10 = +-10 % at most). */
#define CENTER_SPD_DIFF_KD     0.00f
#define CENTER_SPD_DIFF_MAX    0.10f

/* ======================================================================= */
/*  Sign / polarity switches - flip to -1.0f if the hardware disagrees     */
/* ======================================================================= */

/* +1: increasing yaw_deg / yaw_rate_dps means the nose turned LEFT.
 *     Bench check: hold the car, rotate it by hand nose-LEFT, watch the
 *     YAW= field printed by main.c (yaw*10).  If YAW goes NEGATIVE the IMU
 *     convention is "positive = right" => set this to -1.0f. */
#define CENTER_YAW_SIGN        (+1.0f)

/* +1: a positive delta applied to App_SetSteer4() turns a wheel to the LEFT.
 *     Applies to all four wheels identically (crab steer must be parallel). */
#define CENTER_STEER_SIGN      (+1.0f)

/* +1: (FL,FR)=+Delta and (RL,RR)=-Delta makes the car yaw LEFT.
 *     If the car yaws the wrong way on the first field test, use -1.0f. */
#define CENTER_TOE_SIGN        (+1.0f)

/* ======================================================================= */
/*  Debug snapshot for the Lead's 1 Hz log line (optional, read-only)      */
/* ======================================================================= */
typedef struct
{
  float e_cm;        /* filtered lateral error, + = car right of centre   */
  float delta_deg;   /* crab steering command actually issued, deg        */
  float toe_deg;     /* toe command actually issued, deg                  */
  u8    valid;       /* 1 = both side lasers usable this tick             */
  u8    saturated;   /* 1 = delta or Delta hit its clamp this tick        */
} CenteringDbg;

extern CenteringDbg g_centering;

/* ======================================================================= */
/*  Contract (USER/app_if.h)                                              */
/* ======================================================================= */
void Centering_Init(void);

/* laser   : latest frame, may be stale/invalid (checked here)
   yaw_deg : IMU heading deviation, deg (0 = straight down the furrow)
   yaw_rate: IMU yaw rate, deg/s
   base_spd: base wheel speed target, encoder counts per 20 ms (>= 0 forward)
   spd_tgt : out, FL, FR, RL, RR encoder counts per 20 ms               */
void Centering_Update(const LaserData *laser, float yaw_deg, float yaw_rate_dps,
                      s16 base_spd, s16 spd_tgt[4]);

#endif
