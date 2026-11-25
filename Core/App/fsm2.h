#ifndef FSM2_H
#define FSM2_H
#include "app_tasks.h"

typedef enum {
  F2_IDLE, F2_ENTRY, F2_CRAB_R1, F2_REV_12, F2_CRAB_L2, F2_REV_23,
  F2_CRAB_R3, F2_REV_34, F2_CRAB_L4, F2_REV_45, F2_CRAB_R5, F2_EXIT_L,
  F2_REV_PARK, F2_DONE, F2_FAULT
} Fsm2State;
typedef enum {F2A_STOP,F2A_FWD,F2A_REV,F2A_CRAB_R,F2A_CRAB_L} Fsm2Action;
typedef enum {F2_OK,F2_LOST_LINK,F2_LOST_IMU,F2_MISSING_RANGE,F2_BAD_LANDMARK,
              F2_TIMEOUT,F2_ABORTED,F2_OBSTRUCTION,F2_STALLED} Fsm2Fault;
typedef struct {
  Fsm2Action action;
  float steer_deg;
  s16 forward_speed;
  u8 slow, hold, track_ridge, wall_left;
} Fsm2Output;
void Fsm2_Init(void);
void Fsm2_Start(void);
void Fsm2_Abort(void);
void Fsm2_Fail(Fsm2Fault reason);
void Fsm2_Tick(const AppInputs *in, u16 odom_cm, Fsm2Output *out);
Fsm2State Fsm2_GetState(void);
u8 Fsm2_GetFaultCode(void);
#endif
