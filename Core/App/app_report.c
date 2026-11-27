/**
  * @file    app_report.c
  * @brief   1 Hz telemetry and OLED screen.
  *
  *  The 1 Hz line used to be ~282 characters times three ports, i.e. ~74 ms of
  *  blocking UART inside the 20 ms control loop.  It now lives in the lowest
  *  priority task, so it can take as long as it likes.
  */
#include "app_report.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "tune_params.h"
#include "app_config.h"
#include "app_if.h"
#include "app_tasks.h"
#include "fsm2.h"
#include "app_cmd.h"

#include "board.h"
#include "main.h"
#include "usart.h"
#include "oled/oled.h"
#include "servo/servo.h"

static StaticSemaphore_t tx_storage;
static SemaphoreHandle_t tx_lock;
void Report_Init(void) { tx_lock=xSemaphoreCreateMutexStatic(&tx_storage); }

/* Console port.  USART1 and USART3 no longer exist in the project: USART6 took
   over the rear F1 laser uplink and UART4 carries the head board. */
static UART_HandleTypeDef *const s_ports[1] = { &huart5 };

static void put_raw(UART_HandleTypeDef *h, const char *s)
{
  size_t n=strlen(s);
  if (n) (void)HAL_UART_Transmit(h,(uint8_t *)s,(uint16_t)n,10u+(uint32_t)n/10u);
}

void App_Puts(const char *s)
{
  /* Console on UART5 only: USART3 and UART4 now carry the two F1 laser
     uplinks, so the console must not share them. */
  xSemaphoreTake(tx_lock,portMAX_DELAY);
  put_raw(s_ports[0], s);
  xSemaphoreGive(tx_lock);
}

/* High-rate traces (MOT/HDG) share UART5 with the console.  One port at 25 Hz
   is ~7.5 kB/s against an 11.5 kB/s link; broadcasting to three tripled that and
   truncated lines mid-field, which made every diagnosis unreliable. */
void App_PutsTrace(const char *s)
{
  xSemaphoreTake(tx_lock,portMAX_DELAY);
  put_raw(s_ports[0], s);
  xSemaphoreGive(tx_lock);
}

void App_PutInt(int v)
{
  char b[12];
  u8   n = 0u;

  if (v < 0) { b[n++] = '-'; v = -v; }
  if (v == 0) { b[n++] = '0'; }
  else
  {
    char t[10];
    u8   m = 0u;
    while ((v > 0) && (m < 10u)) { t[m++] = (char)('0' + (v % 10)); v /= 10; }
    while (m > 0u) b[n++] = t[--m];
  }
  b[n] = '\0';

  App_Puts(b);
}

/* ----------------------------------------------------------------------- */
/*  1 Hz telemetry                                                         */
/* ----------------------------------------------------------------------- */
static void format_angles(char out[32], const s16 angles[4])
{
  size_t used=0;
  for (unsigned w=0; w<4; ++w) {
    if (w) out[used++]='/';
    used += angles[w]==SERVO_ANGLE_NA ? snprintf(out+used,32-used,"NA")
                                      : snprintf(out+used,32-used,"%d",angles[w]);
  }
}

void Report_Heading(void)
{
  AppInputs in;
  char line[192];
  App_ReadInputs(&in);
  const HeadingLog *h = &in.heading;
  if (h->mode == APP_MODE_IDLE || h->mode == APP_MODE_WHEELTEST) return;
  int n = snprintf(line, sizeof line,
      "HDG MS=%lu MODE=%u F2=%u IMU=%u YAW10=%d RATE10=%d RAW=%d COR=%d"
      " TGT=%d/%d/%d/%d MEAS=%d/%d/%d/%d PV=%lu\r\n",
      (unsigned long)in.ms, (unsigned)h->mode, (unsigned)h->stage, (unsigned)in.imu_ok,
      (int)(in.yaw_deg*10), (int)(in.yaw_rate_dps*10), h->raw, h->correction,
      h->target[0], h->target[1], h->target[2], h->target[3],
      h->measured[0], h->measured[1], h->measured[2], h->measured[3], (unsigned long)h->param_revision);
  if (n > 0 && (size_t)n < sizeof line) App_Puts(line);
}

void Report_Log(void)
{
  AppInputs sample;
  App_ReadInputs(&sample);
  const AppInputs *in = &sample;

  /* First line after reset: say WHY we rebooted.  RCC->CSR survives a reset,
     so "IWDG" here means the board is still reset-looping even though the
     OLED keeps showing the previous frame (the panel is not cleared by a
     reset, only the GRAM is). */
  {
    static u8 done = 0u;

    if (done == 0u)
    {
      u32 csr = RCC->CSR;

      done = 1u;
      App_Puts("RST:");
      if ((csr & RCC_CSR_IWDGRSTF) != 0u) App_Puts(" IWDG");
      if ((csr & RCC_CSR_WWDGRSTF) != 0u) App_Puts(" WWDG");
      if ((csr & RCC_CSR_SFTRSTF)  != 0u) App_Puts(" SW");
      if ((csr & RCC_CSR_PORRSTF)  != 0u) App_Puts(" POR");
      if ((csr & RCC_CSR_PINRSTF)  != 0u) App_Puts(" PIN");
      if ((csr & RCC_CSR_BORRSTF)  != 0u) App_Puts(" BOR");
      if ((csr & RCC_CSR_LPWRRSTF) != 0u) App_Puts(" LPWR");
      App_Puts("\r\n");

      __HAL_RCC_CLEAR_RESET_FLAGS();
    }
  }

  char line[240], target[32], actual[32];
  s16 pwm[4];
  AppControl_GetPwm(pwm);
  int n=snprintf(line,sizeof line,
    "SPD %d MODE=%d SF=%u CNT=%d/%d/%d/%d PWM=%d/%d/%d/%d YAW=%d RATE=%d"
    " VB=%d LS=%d,%d,%d,%u RG=%d,%u F2=%u FA=%u LZ=%s DIST=%u PV=%lu\r\n",
    App_GetBaseSpeed(),(int)AppControl_Mode(),AppControl_SafetyFlags(),
    in->wheel_cnt[0],in->wheel_cnt[1],in->wheel_cnt[2],in->wheel_cnt[3],
    pwm[0],pwm[1],pwm[2],pwm[3],(int)(in->yaw_deg*10),(int)(in->yaw_rate_dps*10),
    (int)(in->batt_adc*3.3f*11.0f*0.99f/4096.0f*10),
    in->dL==BSP_DIST_INVALID?-1:in->dL,in->dR==BSP_DIST_INVALID?-1:in->dR,
    in->dF==BSP_DIST_INVALID?-1:in->dF,in->laser_flags,
    in->dB==BSP_DIST_INVALID?-1:in->dB,in->vB,(unsigned)Fsm2_GetState(),
    Fsm2_GetFaultCode(),in->laser_fresh?"OK":"LOST",App_GetDistanceCm(),
    (unsigned long)in->heading.param_revision);
  if (n>0 && (size_t)n<sizeof line) App_Puts(line);
  snprintf(line,sizeof line,"LASER MS=%lu CFG=%u HEAD=%u TAIL=%u FLAGS=%u TLRB=%u/%u/%u\r\n",
           (unsigned long)in->ms, in->use_tail ? 6u : 3u, in->laser_fresh, in->tail_fresh,
           in->tail_flags, in->tail_left, in->tail_right, in->tail_back);
  App_Puts(line);
  format_angles(target,in->steer_target10);
  format_angles(actual,in->steer_feedback10);
  snprintf(line,sizeof line,"STEER TGT10=%s ACT10=%s\r\n",target,actual);
  App_Puts(line);
}

/* ----------------------------------------------------------------------- */
/*  5 Hz OLED                                                              */
/* ----------------------------------------------------------------------- */

/* Signed value in tenths, printed with an explicit decimal point ("-3.0").
   OLED_ShowNumber() takes a u32 and blanks the leading zeros, so the sign and
   the point are drawn by hand.  A bare integer here was easy to misread as
   whole degrees. */
static void show_signed_tenths(u8 x, u8 y, int v10, u8 size)
{
  u8  cw  = (u8)(size / 2u);
  u8  neg = (u8)(v10 < 0);
  u32 mag = (neg != 0u) ? (u32)(-v10) : (u32)v10;
  u32 ip  = mag / 10u;
  u8  fd  = (u8)(mag % 10u);
  u8  d   = 1u;
  u32 lim = 10u;
  u8  col = 0u;

  while ((ip >= lim) && (d < 4u)) { lim *= 10u; d++; }

  if (neg != 0u) { OLED_ShowChar(x, y, (u8)'-', size, 1); col = 1u; }

  OLED_ShowNumber((u8)(x + (u8)(col * cw)), y, ip, d, size);
  col = (u8)(col + d);
  OLED_ShowChar((u8)(x + (u8)(col * cw)), y, (u8)'.', size, 1);
  col++;
  OLED_ShowChar((u8)(x + (u8)(col * cw)), y, (u8)('0' + (char)fd), size, 1);
}

void Report_Ui(void)
{
  AppInputs sample;
  App_ReadInputs(&sample);
  const AppInputs *in = &sample;
  AppMode m = AppControl_Mode();

  OLED_Clear();
  OLED_ShowString(0, 0, (const u8 *)"C30D 4WS");

  switch (m)
  {
    case APP_MODE_EDGE:      OLED_ShowString(64, 0, (const u8 *)"EDGE"); break;
    case APP_MODE_FSM2:      OLED_ShowString(64, 0, (const u8 *)"FSM2"); break;
    case APP_MODE_CRAB:      OLED_ShowString(64, 0, (const u8 *)"CRAB"); break;
    case APP_MODE_DRIVE:     OLED_ShowString(64, 0, (const u8 *)"DRIVE"); break;
    case APP_MODE_WHEELTEST: OLED_ShowString(64, 0, (const u8 *)"WHEEL"); break;
    case APP_MODE_REV_EDGE:  OLED_ShowString(64, 0, (const u8 *)"REV");   break;
    default:                 OLED_ShowString(64, 0, (const u8 *)"IDLE"); break;
  }

  /* Last command letter accepted from any port: if a letter appears here
     while nobody is typing, something is feeding the command parser. */
  OLED_ShowChar(112, 0, (u8)AppCmd_LastChar(), 12, 1);
  /* PD3 motor-enable state: "E" enabled, "e" disabled. */
  OLED_ShowChar(100, 0, (u8)(AppControl_Enabled() ? (u8)'E' : (u8)'e'), 12, 1);

  OLED_ShowString(0, 12, (const u8 *)"L");
  OLED_ShowNumber(10, 12, (u32)((in->dL == BSP_DIST_INVALID) ? 0u : in->dL), 4, 12);
  OLED_ShowString(56, 12, (const u8 *)"R");
  OLED_ShowNumber(66, 12, (u32)((in->dR == BSP_DIST_INVALID) ? 0u : in->dR), 4, 12);

  /* What the firmware is actually commanding (max |PWM| over the four
     wheels).  If this reads 0 while the wheels still turn, the bridge is
     being driven from outside the MCU - it is a hardware problem, not the
     control loop. */
  {
    s16 pwm[4];
    u32 mx = 0u;
    u8  k;

    AppControl_GetPwm(pwm);
    for (k = 0u; k < 4u; k++)
    {
      u32 a = (u32)((pwm[k] < 0) ? -pwm[k] : pwm[k]);
      if (a > mx) mx = a;
    }

    OLED_ShowString(0, 24, (const u8 *)"PWM");
    OLED_ShowNumber(30, 24, mx, 5, 12);
    OLED_ShowString(76, 24, (const u8 *)"SF");
    OLED_ShowNumber(96, 24, (u32)AppControl_SafetyFlags(), 1, 12);
  }

  OLED_ShowString(0, 36, (const u8 *)"YAW");
  {
    /* Tenths of a degree, wrapped into (-180, 180] so the field never needs
       more digits than it has.  The old code clamped a negative heading to 0,
       which hid every left-hand turn. */
    int y10 = (int)(in->yaw_deg * 10.0f) % 3600;
    if (y10 >  1800) y10 -= 3600;
    if (y10 < -1800) y10 += 3600;
    show_signed_tenths(28, 36, y10, 12u);
  }

  OLED_ShowString(0, 48, (const u8 *)"F2");
  OLED_ShowNumber(24, 48, (u32)Fsm2_GetState(), 2, 12);
  OLED_ShowString(56, 48, (const u8 *)"VB");
  OLED_ShowNumber(76, 48, (u32)((float)in->batt_adc * 3.3f * 11.0f * 0.99f / 4096.0f * 10.0f), 3, 12);

  OLED_Refresh_Gram();
}
