/**
  * @file    app_tasks.h
  * @brief   FreeRTOS task set and the shared input/output snapshots.
  *
  *  Task model (native FreeRTOS API, no CMSIS wrapper):
  *
  *      task     period    prio  job
  *      ctrl     20 ms     30    sample -> control -> actuate, must not block
  *      laser     5 ms     28    UART4 DMA drain + frame parse
  *      mon     100 ms     26    task liveness + IWDG refresh
  *      cmd      10 ms     20    serial command ports + user key
  *      ui      200 ms     12    OLED refresh
  *      log    1000 ms     10    telemetry to the three command ports
  */
#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "bsp_types.h"

/* Published once per control tick. */
typedef struct AppInputs
{
  u16 dF, dB, dL, dR;         /* mm, 0xFFFF = invalid                  */
  u8  vF, vB, vL, vR;         /* 1 = that channel had a valid frame     */
  u8  laser_fresh, laser_flags;
  u8  sample_seq[4];
  u8  imu_ok;
  s16 wheel_cnt[4];           /* signed counts this period, FL FR RL RR */
  float yaw_deg;
  float yaw_rate_dps;
  u16 batt_adc;
  u32 ms;
  u32 seq;
} AppInputs;

/* Produced by the control task, applied by it right after. */
typedef struct AppOutputs
{
  s16 pwm[4];                 /* signed PWM, FL FR RL RR                */
  s16 steer_deg10[4];         /* per-wheel angle in 0.1 deg             */
  u8  steer_dirty;            /* 1 = send a new servo frame             */
} AppOutputs;

void App_ReadInputs(AppInputs *out);

/* Incremented by the control task every tick; the monitor task uses it to
   decide whether the system is still meeting its deadline. */
extern volatile u32 g_ctl_ticks;

/* Creates all tasks with static allocation. Call from MX_FREERTOS_Init(). */
void App_InitRtos(void);

#endif /* APP_TASKS_H */
