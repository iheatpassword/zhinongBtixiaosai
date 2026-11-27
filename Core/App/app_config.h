/**
  * @file    app_config.h
  * @brief   All tunables in one place.  Values carried over from the old
  *          USER/main.c (see C30D_MotorTest/docs/CODE_AUDIT.md).
  */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ---- control cadence -------------------------------------------------- */
#define APP_CTRL_MS        20u      /* 50 Hz control block                */

/* ---- odometry --------------------------------------------------------- */
#define WHEEL_DIAMETER_MM  71.2f
#define ENCODER_CPR        60000.0f

/* ---- speed loop ------------------------------------------------------- */
#define SPD_DEFAULT        150      /* counts per 20 ms                   */
#define SPD_MAX            2000
#define KP_NUM             10       /* PWM = KP_NUM*err/10 + integral     */
#define KI_DIV             8        /* integral += err/KI_DIV             */
#define PWM_SLEW           400      /* max PWM change per 20 ms tick      */
#define PWM_FF             250      /* feed-forward base (dead zone)      */
#define INTEG_MAX          10000
#define PWM_LIM            12000

/* ---- 4WS edge following (S1 / ENTRY) ---------------------------------- */
#define EDGE_SPD           5000     /* forward speed counts/20 ms         */
#define EDGE_KP            0.10f    /* deg per mm                         */
#define EDGE_MAX           5.0f     /* steering clamp, deg                */

/* Forward entry along the left wall (A entry and the E test).
   These are the FACTORY defaults.  They are what the car runs on whenever the
   saved record is absent or invalid - which happens every time the parameter
   struct changes - so they are kept at the values the car was actually tuned to,
   not at the historical ones.  A power cycle must never silently drop the car
   back to a 22 cm/s crawl. */
#define ENTRY_CRUISE_COUNTS     5000  /* standalone E test cruise, ~93 cm/s */
#define ROUTE_ENTRY_SEARCH      5000  /* automatic entry before rail acquisition */
#define ROUTE_ENTRY_CRUISE      5000  /* centred automatic entry */
#define ENTRY_WALL_ACQUIRE_MM    120u /* rail always reads <100 mm when first met */
#define ENTRY_WALL_TRACK_MM      300u /* once locked, a lost sample must not brake */
#define ENTRY_SEARCH_LIMIT_CM    100u /* startup opening */
#define ENTRY_WALL_BAND_MM        15  /* either side of SIDE_CENTER_MM */
#define ENTRY_WALL_HYST_VOTES      8u /* consecutive off-band samples before cruise->creep */
#define ENTRY_WALL_KP           0.20f /* wheel angle in degrees/mm */
#define ENTRY_WALL_MAX_DEG      15.0f
/* The steering linkage has a few degrees of slack.  A 10 mm error asks for 1 deg,
   the linkage swallows it, and the offset never closes - so the loop needs
   integral action to walk the wheel out of the dead band. */
#define ENTRY_STEER_KI          0.06f /* deg per (mm * s) */
#define ENTRY_STEER_INTEG_MAX_DEG 5.0f
/* Dead-zone feed-forward.  Most of what looked like 5.5 deg of linkage slack
   turned out to be stale per-wheel centre trims (see servo.c), so after those
   were corrected the feed-forward only has to cover the real backlash and must
   not fire on errors inside the tolerance band - at 3 mm / 3.5 deg a 4 mm error
   produced a 6.5 deg steering command and the wheels visibly swung at rest. */
#define ENTRY_STEER_DEADZONE_DEG 6.0f
#define ENTRY_STEER_DEADZONE_MM  3.0f
#define ENTRY_RANGE_FILTER_MS   100.0f /* time constant for lateral range rate */
#define ENTRY_SPEED_FILTER_MS    1200u /* slow average of the lateral error, diagnostics only */
/* Body-angle trim: a sustained range rate at speed means the body is angled to
   the wall, which translating cannot fix.  Front and rear wheels are split by
   this much to rotate it back parallel. */
#define ENTRY_YAW_TRIM_GAIN      30.0f /* deg of split per sin(heading error) */
#define ENTRY_YAW_TRIM_MAX_DEG    6.0f
#define ENTRY_YAW_TRIM_FILTER_MS 800u  /* only a sustained drift counts as heading */
/* Yaw-rate damping applied to the front/rear split.  The geometric pair trim
   removes a static body angle; the gyro rate damps the oscillation itself,
   which is the part the position loop cannot see.  Gain 0 disables it: the
   sign must be measured on the car before any feedback gain is trusted. */
#define ENTRY_YAW_DAMP_GAIN      0.0f  /* deg of split per (deg/s) */
#define ENTRY_YAW_DAMP_MAX_DEG   3.0f
/* Yaw-rate closed loop (reference-route style).  A PD on the gyro heading gives
   a yaw-rate command in deg/s, and the inverse kinematics turn it into a
   front/rear split: omega = 2*v*delta/L  =>  delta = omega*L/(2v).  Dividing by
   the wheel speed is what makes the authority speed independent - a fixed split
   buys only ~1 deg/s at creeping speed, which is why the rate-based trim could
   never hold a heading. */
#define ENTRY_YAW_WZ_KP          2.0f  /* deg/s per deg of heading error */
#define ENTRY_YAW_WZ_KD          0.5f  /* deg/s per (deg/s) of measured rate */
#define ENTRY_YAW_WZ_MAX        60.0f  /* clamp on the rate command, deg/s */
#define ENTRY_YAW_WB_MM        500.0f  /* front/rear axle spacing for the split */
/* Sign-calibration knobs, both 0 in normal use.  Set one, drive E once, and
   read the direction out of the log; that settles the convention. */
#define ENTRY_DEBUG_ANGLE_DEG    0.0f  /* added to the common four-wheel angle */
#define ENTRY_DEBUG_SPLIT_DEG    0.0f  /* added to the front/rear split        */
#define ENTRY_STEER_LEAD_MS     150u   /* look ahead for steering response */
#define ENTRY_FULL_ERROR_MM      50.0f /* diagnostics only; the speed no longer follows the error */
#define ENTRY_CREEP_ERROR_MM    100.0f /* diagnostics only; the speed no longer follows the error */
#define ENTRY_COAST_MARGIN_MM     6u   /* above the independent near-obstacle trip */
#define ENTRY_ACCEL_COUNTS      200    /* max rise per 20 ms; at 1 m/s the ramp must not dominate */
/* One speed count is this many mm/s: pi*D/CPR per 20 ms. */
#define MM_S_PER_COUNT          0.1864f
/* Slew limit on the common crab angle.  Without it a sign change in the error
   orders the servos to swing the full angle in a single 20 ms tick, which reads
   as a snatch; the reference route commands a lateral velocity and only ever
   asks for ~2.6 deg of crab, so its body just eases sideways. */
#define ENTRY_STEER_SLEW_DPS    150.0f /* deg per second */

/* Reverse return along the left wall (R command, used by the tuning loop). */
#define REV_EDGE_SPD          900    /* counts/20 ms, ~17 cm/s            */
#define REV_EDGE_DEFAULT_MM   3700   /* bare R reverses this far          */

/* A near-obstacle trip stops the drive but must not freeze the car: backing
   away is always allowed, clipped to this PWM so it can only creep out. */
#define SAFE_ESCAPE_PWM       4000
/* Side-only near-obstacle on a wall-following mode: keep the task alive and let
   the car slide away at this PWM, because zeroing the drive removed the only
   motion that could recover it. */
#define SAFE_SIDE_ESCAPE_PWM  2200
#define SIDE_BLOCK_GIVEUP_MS   3000u

/* ---- differential heading trim (left/right speed split) --------------- */
#define DIFF_KP            0.0f
#define DIFF_KI            0.0f
#define DIFF_KD            0.0f
#define DIFF_MAX           600.0f
#define DIFF_FF            (-55.0f) /* proven 2600 pure-diff trim, counts */

/* ---- crab / ridge shift ----------------------------------------------- */
#define SHIFT_SPD          5000
#define REV_FIND_PLATE_COUNTS   1800 /* counts/20 ms, before near plate confirmation */
#define ROUTE_TURN_COAST_MM_S   80u /* max individual wheel speed for steering under zero drive */
#define REV_SPD             SHIFT_SPD     /* reverse ridge-change cruise, counts/20 ms */
#define SHIFT_YAW_SPD_KP   40.0f
#define SHIFT_YAW_SPD_KD   9.0f
#define SHIFT_YAW_SPD_MAX  150.0f
#define SHIFT_YAW_SPD_SIGN (-1.0f)

/* Course dimensions and initial calibration. Distances refer to beam origins. */
#define ENTRY_STOP_MM           220u
#define SIDE_CENTER_MM           65u
/* Entry wall target, calibrated by hand on 2026-11-27 with the car in the
   correct entry pose: tail-left read 56.7 mm (10 samples, 56-57 spread).  The
   entry now follows the tail-left beam because the nose-left one is dead, so it
   must hold the tail beam's own calibrated reading - 65 was the old nose-beam
   number and would have parked the car 8 mm off. */
#define ENTRY_TARGET_MM          65u
#define REAR_TARGET_MM          137u  /* 100 mm geometric + old head's 37 mm offset */
#define RIDGE_FRONT_MM          206   /* six-laser centre pose, not the entry stop */
#define RIDGE_TAIL_MM           216   /* rear board, backward-facing VB02A */
#define RIDGE_SUM_TOL_MM         65u
/* Nominal side-beam spacing: 700 mm body, stations 60 mm in from each end. */
#define WALL_SENSOR_BASELINE_MM 580.0f
#define WALL_PAIR_MAX_DIFF_MM    70
#define WALL_PAIR_MAX_SKEW_MS    40
#ifndef ROUTE_TEST_STOP_AFTER_ROW2
#define ROUTE_TEST_STOP_AFTER_ROW2 0u
#endif
/* Approach envelope in mm and seconds. Final coast was measured at 77-79 mm/s. */
#define ROUTE_DECEL_MM_S2      1200.0f
#define ROUTE_DRIVE_LAG_MS      150u /* margin for the speed loop / PWM slew */
#define ROUTE_FINAL_COAST_MM    9u
#define ROUTE_FINAL_SPEED_MM_S  80u
#define ROUTE_TURN_TIMEOUT_MS   4000u
#define ROUTE_SETTLE_MS         600u
#define ROUTE_CREEP_COUNTS      450
#define CRAB_TRAVEL_CM          260u
#define ROW_PITCH_CM             70u
#define EDGE_TO_CENTER_CM        26u  /* side beam 29 cm ahead, half ridge width 15 cm */
#define PARK_TRAVEL_CM          100u
#define CRAB_TRIM_MAX_DEG       6.0f
#define YAW_DRIVE_KP            10.0f
#define YAW_DRIVE_KD            2.0f

/* ---- battery ---------------------------------------------------------- */
#define BATT_LOW_V         10.5f

/* ---- serial command ports -------------------------------------------- */
#define CMD_LINE_MAX       72u

#endif /* APP_CONFIG_H */
