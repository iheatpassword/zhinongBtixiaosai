/**
  * @file    app_report.c
  * @brief   1 Hz telemetry and OLED screen.
  *
  *  The 1 Hz line used to be ~282 characters times three ports, i.e. ~74 ms of
  *  blocking UART inside the 20 ms control loop.  It now lives in the lowest
  *  priority task, so it can take as long as it likes.
  */
#include "app_report.h"
#include "app_config.h"
#include "app_if.h"
#include "app_tasks.h"
#include "fsm2.h"
#include "app_cmd.h"

#include "board.h"
#include "main.h"
#include "usart.h"
#include "oled/oled.h"

#define TX_TIMEOUT_MS  20u

static UART_HandleTypeDef *const s_ports[3] = { &huart1, &huart3, &huart5 };

static void put_raw(UART_HandleTypeDef *h, const char *s)
{
  uint16_t n = 0u;

  while ((s[n] != '\0') && (n < 200u)) n++;
  if (n == 0u) return;

  (void)HAL_UART_Transmit(h, (uint8_t *)s, n, TX_TIMEOUT_MS);
}

void App_Puts(const char *s)
{
  u8 i;

  for (i = 0u; i < 3u; i++) put_raw(s_ports[i], s);
}

void App_PutInt(int v)
{
  char b[12];
  u8   n = 0u;
  u8   i;

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

  for (i = 0u; i < 3u; i++) put_raw(s_ports[i], b);
}

/* ----------------------------------------------------------------------- */
/*  1 Hz telemetry                                                         */
/* ----------------------------------------------------------------------- */
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

  App_Puts("SPD ");
  App_PutInt((int)App_GetBaseSpeed());
  App_Puts(" MODE=");
  App_PutInt((int)AppControl_Mode());
  App_Puts(" SF=");
  App_PutInt((int)AppControl_SafetyFlags());

  App_Puts(" CNT=");
  App_PutInt((int)in->wheel_cnt[0]); App_Puts("/");
  App_PutInt((int)in->wheel_cnt[1]); App_Puts("/");
  App_PutInt((int)in->wheel_cnt[2]); App_Puts("/");
  App_PutInt((int)in->wheel_cnt[3]);

  /* What the firmware is actually commanding. All zeros while the wheels are
     spinning => the bridge is being driven from somewhere else. */
  {
    s16 pwm[4];
    AppControl_GetPwm(pwm);
    App_Puts(" PWM=");
    App_PutInt((int)pwm[0]); App_Puts("/");
    App_PutInt((int)pwm[1]); App_Puts("/");
    App_PutInt((int)pwm[2]); App_Puts("/");
    App_PutInt((int)pwm[3]);
  }

  App_Puts(" YAW=");
  App_PutInt((int)(in->yaw_deg * 10.0f));
  App_Puts(" RATE=");
  App_PutInt((int)(in->yaw_rate_dps * 10.0f));

  App_Puts(" VB=");
  App_PutInt((int)((float)in->batt_adc * 3.3f * 11.0f * 0.99f / 4096.0f * 10.0f));

  App_Puts(" LS=");
  App_PutInt((in->dL == BSP_DIST_INVALID) ? -1 : (int)in->dL); App_Puts(",");
  App_PutInt((in->dR == BSP_DIST_INVALID) ? -1 : (int)in->dR); App_Puts(",");
  App_PutInt((in->dF == BSP_DIST_INVALID) ? -1 : (int)in->dF); App_Puts(",");
  App_PutInt((int)in->laser_flags);

  App_Puts(" RG=");
  App_PutInt((in->dB == BSP_DIST_INVALID) ? -1 : (int)in->dB); App_Puts(",");
  App_PutInt((int)in->vB);

  App_Puts(" F2=");
  App_PutInt((int)Fsm2_GetState());
  App_Puts(" FA=");
  App_PutInt((int)Fsm2_GetFaultCode());

  App_Puts(" LZ=");
  App_Puts(in->laser_fresh ? "OK" : "LOST");

  App_Puts(" DIST=");
  App_PutInt((int)App_GetDistanceCm());

  App_Puts("\r\n");
}

/* ----------------------------------------------------------------------- */
/*  5 Hz OLED                                                              */
/* ----------------------------------------------------------------------- */
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
  OLED_ShowNumber(28, 36, (u32)((int)(in->yaw_deg * 10.0f) < 0 ? 0 : (int)(in->yaw_deg * 10.0f)), 4, 12);

  OLED_ShowString(0, 48, (const u8 *)"F2");
  OLED_ShowNumber(24, 48, (u32)Fsm2_GetState(), 2, 12);
  OLED_ShowString(56, 48, (const u8 *)"VB");
  OLED_ShowNumber(76, 48, (u32)((float)in->batt_adc * 3.3f * 11.0f * 0.99f / 4096.0f * 10.0f), 3, 12);

  OLED_Refresh_Gram();
}
