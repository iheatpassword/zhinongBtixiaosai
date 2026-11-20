/**
  * @file    fsm.h
  * @brief   Competition state machine + 70 cm lateral lane shift (T4 / T5).
  *
  *  Implements the app_if.h contract (USER/app_if.h is authoritative):
  *
  *      void     FSM_Init(void);
  *      void     FSM_Start(void);
  *      void     FSM_Tick(const FsmInput *in);
  *      CarState FSM_GetState(void);
  *
  *  Field model
  *  -----------
  *  Six ridges run along the driving direction with a 70 cm pitch
  *  (30 cm ridge + 40 cm furrow).  Ridges 1..5 are 220 cm long, the last
  *  ridge is 180 cm.  The car enters through the 40 cm door, drives ridge 1
  *  forward, shifts 70 cm sideways with all four servos at 90 deg, drives
  *  ridge 2 backwards (serpentine), shifts again ... after ridge 6 it drives
  *  out through the door and parks in the start area.
  *
  *  State table
  *  -----------
  *    IDLE   motors stopped, servos centred; waits for FSM_Start()
  *    ENTRY  servos centred, forward at FSM_SPEED_ENTRY until the entry
  *           door is detected (laser edge) or FSM_ENTRY_CM has been driven
  *    RIDGE  laser centering active, signed base speed (+ forward on odd
  *           ridges, - backward on even ridges) until the ridge length of
  *           the current ridge is reached
  *    SHIFT  motors stopped, all four servos to 90 deg, creep at
  *           FSM_SPEED_SHIFT until FSM_SHIFT_CM, then servos centred and a
  *           clean stop before the next ridge
  *    EXIT   servos centred, forward at FSM_SPEED_EXIT until the exit door
  *           is detected or FSM_EXIT_CM has been driven
  *    PARK   motors off, stays there until power cycle
  *    FAULT  latched: entered from RIDGE/SHIFT when the laser is invalid for
  *           more than FSM_LASER_FAULT_MS, or from any active state when the
  *           run exceeds FSM_RUN_TIMEOUT_MS; FSM_Start() clears it
  *
  *  Sequence: IDLE -> ENTRY -> RIDGE(1) -> SHIFT(1) -> RIDGE(2) -> SHIFT(2)
  *            -> RIDGE(3) -> ... -> SHIFT(5) -> RIDGE(6) -> EXIT -> PARK
  *            (6 ridges, 5 lateral shifts)
  *
  *  Timing: FSM_Tick() must be called every FSM_TICK_MS (20 ms) from the
  *  Lead's main loop; FsmInput::ms is a 1 ms time stamp.
  *
  *  Distances: every phase keeps its own baseline of App_GetDistanceCm() -
  *  the FSM never asks for a reset (App_ResetDistance is not in the contract).
  *  App_GetDistanceCm() is expected to grow with travelled distance with a
  *  1 cm resolution; a decreasing (reverse) or wrapped reading is handled.
  *
  *  ASCII only (Keil ARMCC AC5 / GBK project), C90 declarations.
  */
#ifndef __FSM_H
#define __FSM_H

#include "app_if.h"

/* ======================================================================= */
/*  Field and competition configuration                                    */
/*  Distances in cm, laser distances in mm, speeds in encoder counts per   */
/*  20 ms control period (about 0.0004 cm per count).                      */
/* ======================================================================= */

/* Number of ridges in the field.                                          */
#define FSM_RIDGE_CNT               6u

/* Ridge length of each ridge, cm.  MUST contain FSM_RIDGE_CNT entries.
   Ridges 1..5: 220 cm, last ridge: 180 cm.                                */
#define FSM_RIDGE_LEN_CM_INIT       { 220u, 220u, 220u, 220u, 220u, 180u }

/* Ridge pitch (ridge + furrow) and the end-of-ridge lateral shift, cm.    */
#define FSM_RIDGE_PITCH_CM          70u
#define FSM_SHIFT_CM                70u

/* Lateral shift: all four steering servos to this angle, deg.             */
#define FSM_SHIFT_STEER_DEG         90.0f

/* Drive speeds, encoder counts per 20 ms; the sign selects the direction. */
#define FSM_SPEED_ENTRY             400
#define FSM_SPEED_RIDGE             700
#define FSM_SPEED_SHIFT             250
#define FSM_SPEED_EXIT              400

/* ENTRY: finish when the entry door is detected OR this distance is driven.
   EXIT : finish when the exit door is detected OR this distance is driven
          (fallback only - normally the door edge fires first; tune on the
          real field so the car parks fully inside the start area).         */
#define FSM_ENTRY_CM                40u
#define FSM_EXIT_CM                 100u

/* Doorway detector: the front laser is "blocked" below FSM_BLOCK_MM (gate
   frame / fence) and "clear" above FSM_DOOR_MM (open area).  A doorway is
   reported after FSM_DOOR_TICKS consecutive clear readings that follow at
   least one blocked reading.                                              */
#define FSM_BLOCK_MM                600u
#define FSM_DOOR_MM                 1500u
#define FSM_DOOR_TICKS              3u

/* Timing, ms.                                                            */
#define FSM_TICK_MS                 20u       /* FSM_Tick() call period     */
#define FSM_STEER_MS                600u      /* steering servo travel time */
#define FSM_LASER_FAULT_MS          200u      /* laser invalid limit        */
#define FSM_RUN_TIMEOUT_MS          270000u   /* 4.5 min competition limit  */

/* Odometry sanity: a per-tick jump above this value is treated as a u16
   wrap or a glitch and is discarded for one tick (cm).                    */
#define FSM_MAX_INC_CM              10u

/* ======================================================================= */
/*  Contract API - implemented in fsm.c, declared in app_if.h              */
/*                                                                         */
/*    void     FSM_Init(void);                                             */
/*    void     FSM_Start(void);          accepted from IDLE and FAULT only */
/*    void     FSM_Tick(const FsmInput *in);    call every 20 ms           */
/*    CarState FSM_GetState(void);                                         */
/* ======================================================================= */

/* Optional diagnostics for the Lead's log / OLED (not used by the FSM).   */
u8          FSM_GetRidge(void);      /* 1..6 while on a ridge, 0 otherwise */
u8          FSM_GetShiftCount(void); /* completed 70 cm shifts, 0..5       */
u16         FSM_GetPhaseCm(void);    /* distance in the current phase, cm  */
u8          FSM_GetFaultCode(void);  /* 0 none, 1 laser, 2 run time        */
const char *FSM_StateName(CarState st);  /* "IDLE".."FAULT", ASCII          */

#endif
