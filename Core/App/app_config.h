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
#define EDGE_SPD           2600     /* forward speed counts/20 ms         */
#define EDGE_KP            0.10f    /* deg per mm                         */
#define EDGE_MAX           5.0f     /* steering clamp, deg                */

/* ---- differential heading trim (left/right speed split) --------------- */
#define DIFF_KP            0.0f
#define DIFF_KI            0.0f
#define DIFF_KD            0.0f
#define DIFF_MAX           600.0f
#define DIFF_FF            (-55.0f) /* proven 2600 pure-diff trim, counts */

/* ---- crab / ridge shift ----------------------------------------------- */
#define SHIFT_SPD          2400
#define SHIFT_YAW_SPD_KP   40.0f
#define SHIFT_YAW_SPD_KD   8.0f
#define SHIFT_YAW_SPD_MAX  150.0f
#define SHIFT_YAW_SPD_SIGN (-1.0f)

/* Course dimensions and initial calibration. Distances refer to beam origins. */
#define ENTRY_STOP_MM           220u
#define SIDE_CENTER_MM           55u
#define REAR_TARGET_MM          137u  /* 100 mm geometric + old head's 37 mm offset */
#define RIDGE_SUM_TOL_MM         65u
#define ROUTE_SETTLE_MS         600u
#define ROUTE_CREEP_COUNTS      450
#define CRAB_TRAVEL_CM          260u
#define ROW_PITCH_CM             70u
#define EDGE_TO_CENTER_CM        26u  /* side beam 29 cm ahead, half ridge width 15 cm */
#define PARK_TRAVEL_CM          100u
#define CRAB_TRIM_MAX_DEG       6.0f
#define YAW_DRIVE_KP            8.0f
#define YAW_DRIVE_KD            1.5f

/* ---- battery ---------------------------------------------------------- */
#define BATT_LOW_V         10.5f

/* ---- serial command ports -------------------------------------------- */
#define CMD_LINE_MAX       40u

#endif /* APP_CONFIG_H */
