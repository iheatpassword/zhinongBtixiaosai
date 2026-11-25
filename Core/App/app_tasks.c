/**
  * @file    app_tasks.c
  * @brief   FreeRTOS task set, native API, static allocation.
  */
#include "app_tasks.h"

#include "FreeRTOS.h"
#include "task.h"

#include "board.h"
#include "motor/motor.h"
#include "encoder/encoder.h"
#include "servo/servo.h"
#include "imu/imu.h"
#include "laser/laser.h"
#include "battery/battery.h"
#include "key/key.h"
#include "iwdg.h"       /* hiwdg */

#include "app_if.h"
#include "app_cmd.h"
#include "app_report.h"

#define CTRL_PERIOD_MS      20u
#define LASER_PERIOD_MS      5u
#define CMD_PERIOD_MS       10u
#define MON_PERIOD_MS      100u
#define UI_PERIOD_MS       200u
#define LOG_PERIOD_MS     1000u

#define PRIO_CTRL           30
#define PRIO_LASER          28
#define PRIO_MON            26
#define PRIO_CMD            20
#define PRIO_UI             12
#define PRIO_LOG            10

#define STK_CTRL           384
#define STK_LASER          256
#define STK_CMD            384
#define STK_MON            192
#define STK_UI             320
#define STK_LOG            320

#define MON_STALL_WINDOWS    3u

static StackType_t  s_stk_ctrl[STK_CTRL];   static StaticTask_t s_tcb_ctrl;
static StackType_t  s_stk_laser[STK_LASER]; static StaticTask_t s_tcb_laser;
static StackType_t  s_stk_cmd[STK_CMD];     static StaticTask_t s_tcb_cmd;
static StackType_t  s_stk_mon[STK_MON];     static StaticTask_t s_tcb_mon;
static StackType_t  s_stk_ui[STK_UI];       static StaticTask_t s_tcb_ui;
static StackType_t  s_stk_log[STK_LOG];     static StaticTask_t s_tcb_log;

volatile u32 g_ctl_ticks;

static AppInputs  s_in[2];
static volatile u8 s_in_idx;
static u32 s_in_seq;
static u32 s_prev_ms;

void App_ReadInputs(AppInputs *out)
{
  taskENTER_CRITICAL();
  *out=s_in[s_in_idx];
  taskEXIT_CRITICAL();
}

/* ----------------------------------------------------------------------- */
/*  sampling / actuation                                                   */
/* ----------------------------------------------------------------------- */

static void sample_inputs(AppInputs *in)
{
  u32   now = BSP_Ms();
  float dt;
  u8    w;

  LaserData laser;
  Laser_Snapshot(&laser);
  in->dF = laser.dF; in->dL = laser.dL;
  in->dR = laser.dR; in->dB = laser.dRidge;
  in->vF = in->dF < BSP_DIST_OVERRANGE;
  in->vL = in->dL < BSP_DIST_OVERRANGE;
  in->vR = in->dR < BSP_DIST_OVERRANGE;
  in->vB = in->dB < BSP_DIST_OVERRANGE;
  in->laser_fresh = laser.valid;
  in->laser_flags = laser.flags;
  for (w=0; w<4; w++) in->sample_seq[w] = laser.sample_seq[w];

  for (w = 0u; w < 4u; w++)
  {
    in->wheel_cnt[w] = (s16)Encoder_Read(w);
  }

  /* Measured period, not a hard-coded 20 ms: the old firmware integrated the
     heading with a constant 0.02 s and therefore produced a heading that
     drifted away from real time whenever the loop slipped. */
  dt = (float)(now - s_prev_ms) / 1000.0f;
  if (dt < 0.005f) dt = 0.005f;
  if (dt > 0.100f) dt = 0.100f;
  s_prev_ms = now;

  in->imu_ok = IMU_Update(dt);

  in->yaw_deg      = IMU_YawDeg();
  in->yaw_rate_dps = IMU_RateDps();
  in->batt_adc     = Battery_Raw();
  in->ms           = now;
}

static void apply_outputs(AppOutputs *out)
{
  Motor_SetPwm(out->pwm[0], out->pwm[1], out->pwm[2], out->pwm[3]);

  if (out->steer_dirty != 0u)
  {
    Servo_Set4Deg((float)out->steer_deg10[0] / 10.0f,
                  (float)out->steer_deg10[1] / 10.0f,
                  (float)out->steer_deg10[2] / 10.0f,
                  (float)out->steer_deg10[3] / 10.0f);
    out->steer_dirty = 0u;
  }
}

static void publish_inputs(AppInputs *src)
{
  u8 next = (u8)(s_in_idx ^ 1u);

  src->seq   = ++s_in_seq;
  s_in[next] = *src;
  s_in_idx   = next;          /* single byte store, atomic on Cortex-M */
}

/* ----------------------------------------------------------------------- */
/*  tasks                                                                  */
/* ----------------------------------------------------------------------- */

static void ctrl_task(void *arg)
{
  TickType_t last = xTaskGetTickCount();
  AppInputs  in;
  AppOutputs out;

  (void)arg;
  s_prev_ms = BSP_Ms();

  for (;;)
  {
    sample_inputs(&in);
    Servo_ReadFeedback(in.steer_feedback10);
    AppControl_Tick(&in, &out);
    apply_outputs(&out);
    for (u8 w=0; w<4; ++w) in.steer_target10[w]=out.steer_deg10[w];
    Servo_QueryNext();
    publish_inputs(&in);

    g_ctl_ticks++;
    vTaskDelayUntil(&last, pdMS_TO_TICKS(CTRL_PERIOD_MS));
  }
}

static void laser_task(void *arg)
{
  TickType_t last = xTaskGetTickCount();

  (void)arg;
  for (;;)
  {
    Laser_Poll();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(LASER_PERIOD_MS));
  }
}

static void cmd_task(void *arg)
{
  TickType_t last = xTaskGetTickCount();

  (void)arg;
  for (;;)
  {
    AppCmd_Poll();
    Key_Tick();
    AppCmd_HandleKey(Key_Event());
    vTaskDelayUntil(&last, pdMS_TO_TICKS(CMD_PERIOD_MS));
  }
}

static void ui_task(void *arg)
{
  TickType_t last = xTaskGetTickCount();

  (void)arg;
  for (;;)
  {
    Report_Ui();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(UI_PERIOD_MS));
  }
}

static void log_task(void *arg)
{
  TickType_t last = xTaskGetTickCount();

  (void)arg;
  for (;;)
  {
    Report_Log();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(LOG_PERIOD_MS));
  }
}

static void mon_task(void *arg)
{
  u32 last_ctl = 0u;
  u32 next_led;
  u8  stall    = 0u;

  (void)arg;
  next_led = BSP_Ms() + 250u;

  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(MON_PERIOD_MS));

    if (g_ctl_ticks == last_ctl)
    {
      if (stall < 0xFFu) stall++;
    }
    else
    {
      stall    = 0u;
      last_ctl = g_ctl_ticks;
    }

    if (stall >= MON_STALL_WINDOWS)
    {
      /* The control task stopped meeting its deadline: stop feeding the dog
         and let the 1 s IWDG reset the board. */
      taskDISABLE_INTERRUPTS();
      for (;;) { }
    }

    /* Heartbeat at 4 Hz, but only while the control task is alive: if it
       stalls the LED freezes, which is a visible symptom before the reset. */
    if ((s32)(BSP_Ms() - next_led) >= 0)
    {
      next_led += 250u;
      Board_HeartbeatToggle();
    }

    (void)HAL_IWDG_Refresh(&hiwdg);
  }
}

/* ----------------------------------------------------------------------- */
/*  hooks                                                                  */
/* ----------------------------------------------------------------------- */

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;
  taskDISABLE_INTERRUPTS();
  for (;;) { }
}

void App_InitRtos(void)
{
  AppControl_Init();
  AppCmd_Init();

  (void)xTaskCreateStatic(ctrl_task,  "ctrl",  STK_CTRL,  NULL, PRIO_CTRL,
                          s_stk_ctrl,  &s_tcb_ctrl);
  (void)xTaskCreateStatic(laser_task, "laser", STK_LASER, NULL, PRIO_LASER,
                          s_stk_laser, &s_tcb_laser);
  (void)xTaskCreateStatic(mon_task,   "mon",   STK_MON,   NULL, PRIO_MON,
                          s_stk_mon,   &s_tcb_mon);
  (void)xTaskCreateStatic(cmd_task,   "cmd",   STK_CMD,   NULL, PRIO_CMD,
                          s_stk_cmd,   &s_tcb_cmd);
  (void)xTaskCreateStatic(ui_task,    "ui",    STK_UI,    NULL, PRIO_UI,
                          s_stk_ui,    &s_tcb_ui);
  (void)xTaskCreateStatic(log_task,   "log",   STK_LOG,   NULL, PRIO_LOG,
                          s_stk_log,   &s_tcb_log);
}
