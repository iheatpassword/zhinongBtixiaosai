/**
  * @file    app_if.h
  * @brief   Sprint-1 interface contract between the Lead-owned integration
  *          (USER/main.c) and the delegated modules:
  *            - USER/laser.c        (agent: laser)      UART4 laser link
  *            - USER/centering.c    (agent: centering)  4WS lateral/heading control
  *            - USER/fsm.c          (agent: fsm)        lane shift + competition FSM
  *
  *  Only the Lead edits main.c / WHEELTEC.uvprojx.  Module authors implement the
  *  functions declared here in their own files.  C sources must stay ASCII-only
  *  (Keil ARMCC AC5 / GBK project).
  */
#ifndef __APP_IF_H
#define __APP_IF_H

#include "system.h"

/* ======================================================================= */
/*  Laser range finder link (UART4: PC10 = TX, PC11 = RX, 115200 8N1)       */
/*  Frame: 0xAA 0x55 | dL(2) dR(2) dF(2) | flags | CRC8                     */
/*         distances mm little endian, 0xFFFF = invalid                    */
/*         CRC8 poly 0x07 init 0x00 over 0xAA,0x55 + 7 payload bytes       */
/* ======================================================================= */
typedef struct
{
  u16 dL;              /* left  distance, mm */
  u16 dR;              /* right distance, mm */
  u16 dF;              /* front distance, mm */
  u8  flags;           /* bit0 front ok, bit1 left ok, bit2 right ok, bit3 timeout */
  u8  valid;           /* frame received within the last 100 ms */
  u32 last_ms;

  /* raw byte monitor for wiring/protocol bring-up */
  u16 raw_count;       /* total bytes received on UART4 */
  u8  raw_last[8];     /* last 8 received bytes, rolling */
  u8  raw_idx;
} LaserData;

extern volatile LaserData g_laser;

void Laser_Init(void);        /* UART4 + DMA1 Stream2/Channel4 circular RX */
void Laser_Poll(void);        /* call every 5..10 ms: assemble frames, timeout */
u8   Laser_IsFresh(void);     /* 1 if a valid frame arrived within 100 ms */

/* ======================================================================= */
/*  Centering controller (lateral via steer angle delta, heading via toe)  */
/* ======================================================================= */
void Centering_Init(void);

/* Inputs:  laser (may be invalid), IMU yaw in deg, yaw rate in deg/s,
            base wheel speed target in encoder counts per 20 ms
   Output:  per-wheel speed targets, order FL, FR, RL, RR (counts / 20 ms) */
void Centering_Update(const LaserData *laser, float yaw_deg, float yaw_rate_dps,
                      s16 base_spd, s16 spd_tgt[4]);

/* ======================================================================= */
/*  Hooks implemented by the Lead in main.c (available to fsm.c)           */
/* ======================================================================= */
void App_SetWheelSpeed(const s16 spd_tgt[4]);      /* speed-loop targets, FL,FR,RL,RR */
void App_DriveStop(void);                          /* stop motors, reset PID        */
void App_SetSteerAll(float deg);                   /* all 4 servos to same angle    */
void App_SetSteerRotate(float deg, float rot);     /* front +deg*rot, rear -deg*rot */
void App_SetServoCenter(void);                     /* all servos 0 deg              */
u16  App_GetDistanceCm(void);                      /* averaged odometry, cm         */

/* ======================================================================= */
/*  Competition state machine                                              */
/* ======================================================================= */
typedef enum
{
  ST_IDLE = 0,
  ST_ENTRY,       /* enter field through the 40 cm door                */
  ST_RIDGE,       /* drive along a ridge, laser centering              */
  ST_SHIFT,       /* shift sideways 70 cm to the next ridge            */
  ST_EXIT,        /* find and drive through the exit door              */
  ST_PARK,        /* stop inside the start area                        */
  ST_FAULT        /* sensor/link failure, motors stopped               */
} CarState;

typedef struct
{
  LaserData laser;
  float yaw_deg;
  float yaw_rate_dps;
  u32   ms;           /* millisecond timestamp */
} FsmInput;

void     FSM_Init(void);
void     FSM_Tick(const FsmInput *in);
CarState FSM_GetState(void);

#endif
