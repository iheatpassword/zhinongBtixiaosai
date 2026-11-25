/**
  * @file    app_if.h
  * @brief   Contract between the application logic and the board layer.
  *
  *  The pure-logic modules (fsm2.c / safety.c / centering.c) only ever include
  *  this header, so they stay independent of HAL and FreeRTOS.  The board
  *  layer (Core/BSP) does the hardware side and is called through the App_*
  *  hooks below.
  */
#ifndef APP_IF_H
#define APP_IF_H

#include "bsp_types.h"
#include "laser.h"      /* LaserData */

/* ----------------------------------------------------------------------- */
/*  Commands posted from the UI / serial tasks, consumed by the control task */
/* ----------------------------------------------------------------------- */
typedef enum
{
  APP_CMD_NONE = 0,
  APP_CMD_STOP,          /* stop everything, clear modes                 */
  APP_CMD_FWD,           /* plain speed-loop forward at the base speed   */
  APP_CMD_BACK,          /* plain speed-loop reverse                     */
  APP_CMD_EDGE,          /* 4WS edge following (S1 / ENTRY test)         */
  APP_CMD_FSM2,          /* full 5-ridge competition sequence            */
  APP_CMD_CRAB_R,        /* crab right test                              */
  APP_CMD_CRAB_L,        /* crab left test                               */
  APP_CMD_SERVO_CENTER,  /* all servos to 0 deg                          */
  APP_CMD_SERVO_P90,     /* all servos to +90 deg                        */
  APP_CMD_SERVO_M90,     /* all servos to -90 deg                        */
  APP_CMD_WHEELTEST      /* single-wheel identification                  */
} AppCmd;

/* Current top-level mode, owned by the control task. */
typedef enum
{
  APP_MODE_IDLE = 0,
  APP_MODE_DRIVE,        /* plain F/B speed loop                         */
  APP_MODE_EDGE,         /* 4WS edge following                           */
  APP_MODE_FSM2,         /* Fsm2_Tick() is driving the output            */
  APP_MODE_CRAB,         /* standalone crab / ridge test                 */
  APP_MODE_WHEELTEST
} AppMode;

/* Post a command from any task.  Pending STOP takes precedence; otherwise the last command
   wins. The control task consumes it atomically. */
void App_PostCmd(AppCmd c);

/* Base speed for plain F/B and for the modes, counts per 20 ms. */
void App_SetBaseSpeed(s16 counts);
s16  App_GetBaseSpeed(void);

/* ----------------------------------------------------------------------- */
/*  Hooks the control task provides for fsm2 / safety / centering           */
/* ----------------------------------------------------------------------- */
void App_SetWheelSpeed(const s16 spd_tgt[4]);   /* FL,FR,RL,RR, counts/20ms */
void App_DriveStop(void);
void App_SetSteerAll(float deg);                /* all four, same angle     */
void App_SetSteer4(float fl, float fr, float rl, float rr);
void App_SetServoCenter(void);
u16  App_GetDistanceCm(void);                   /* averaged odometry, cm    */
u32  App_MsNow(void);

/* ----------------------------------------------------------------------- */
/*  Control task interface                                                  */
/* ----------------------------------------------------------------------- */
struct AppInputs;   /* defined in app_tasks.h */
struct AppOutputs;

void    AppControl_Init(void);
void    AppControl_Tick(const struct AppInputs *in, struct AppOutputs *out);
AppMode AppControl_Mode(void);

/* Safety layer state, bit0 = distance stop, bit1 = stall. The trip is not
   latched: it drops the chassis to IDLE and a new command may move again
   once the obstruction is gone. */
u8      AppControl_SafetyFlags(void);

/* Last PWM handed to the motors (FL,FR,RL,RR, signed). 0 everywhere means the
   firmware is commanding a stop - if the wheels still turn in that state the
   cause is downstream (bridge enable, wiring), not the control loop. */
void    AppControl_GetPwm(s16 out[4]);

/* Motor driver enable (PD3) as sampled on the last control tick. */
u8      AppControl_Enabled(void);

#endif /* APP_IF_H */
