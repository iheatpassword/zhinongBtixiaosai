/**
  * @file    main.c
  * @brief   C30D 4WD/4WS car - STRAIGHT RUN TEST (after direction calibration)
  *
  * Measured wheel directions (left two were reversed, now all "forward" = +):
  *     FL = +   FR = +   RL = +   RR = +
  *
  * Cycle:  3 s countdown -> forward 2.0 s -> stop 1.5 s -> forward ... (loop)
  * Wheels on the ground, car aimed at open space.  KEY (PE0):
  *     short press : select next wheel (FL -> FR -> RL -> RR)
  *     long  press : invert the direction of the selected wheel (if needed)
  *
  * OLED: per wheel the current forward sign and the encoder counts per 100 ms.
  * USART1 (115200): "dir: + + + +  spd: ..." once per second.
  *
  * Based on the WHEELTEC C30D V2.0 official source; only USER/main.c replaced,
  * the FreeRTOS scheduler is NOT started.
  */
#include "system.h"

/* ----------------------------------------------------------------------- */
/*  configuration                                                          */
/* ----------------------------------------------------------------------- */
#define RUN_PWM        2200     /* straight drive PWM                     */
#define RUN_MS         2000u    /* forward time per cycle                 */
#define STOP_MS        1500u    /* pause time per cycle                   */
#define SERVO_CENTER   500
#define KEY_LONG_MS    800u
#define KEY_SHORT_MIN  30u

/* wheels: FL=0 FR=1 RL=2 RR=3 ; servos: FL=4 FR=3 RL=2 RR=1 */
static const u8 WHEEL_SERVO[4] = { 4u, 3u, 2u, 1u };
static const u8 WHEEL_MOTOR[4] = { 0u, 1u, 2u, 3u };
static const char *WHEEL_NAME[4] = { "FL", "FR", "RL", "RR" };
static const char *MOTOR_NAME[4] = { "A", "B", "C", "D" };

/* +1 / -1 : PWM sign used as "drive forward" for that wheel.
   Calibrated on the car: left wheels were reversed, so all are +1 now.    */
static s8 motor_dir[4] = { 1, 1, 1, 1 };

static u8  wheel_sel;
static int16_t wheel_speed[4];
static u8  run_active;

/* ======================================================================= */
/*  USART1 debug                                                          */
/* ======================================================================= */
static void u1_putc(u8 b)
{
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) { }
  USART_SendData(USART1, b);
}

static void u1_puts(const char *s)
{
  while (*s != '\0') u1_putc((u8)*s++);
}

static void u1_num(int v)
{
  char b[10];
  u8 n = 0;
  if (v < 0) { u1_putc('-'); v = -v; }
  if (v == 0) { u1_putc('0'); return; }
  while (v > 0) { b[n++] = (char)('0' + (v % 10)); v /= 10; }
  while (n > 0) u1_putc((u8)b[--n]);
}

/* ======================================================================= */
/*  USART2 : HTS-35H bus servo                                            */
/* ======================================================================= */
static void u2_putc(u8 b)
{
  while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) { }
  USART_SendData(USART2, b);
}

static void servo_send(u8 id, u8 cmd, const u8 *p, u8 n)
{
  u8  i;
  u8  len = (u8)(n + 3u);
  u16 sum = (u16)(id + len + cmd);

  for (i = 0; i < n; i++) sum = (u16)(sum + p[i]);

  u2_putc(0x55); u2_putc(0x55); u2_putc(id); u2_putc(len); u2_putc(cmd);
  for (i = 0; i < n; i++) u2_putc(p[i]);
  u2_putc((u8)(255u - (sum & 0xFFu)));
}

static void servo_move(u8 id, u16 pos, u16 move_ms)
{
  u8 p[4];
  p[0] = (u8)(pos & 0xFFu);
  p[1] = (u8)(pos >> 8);
  p[2] = (u8)(move_ms & 0xFFu);
  p[3] = (u8)(move_ms >> 8);
  servo_send(id, 1u, p, 4u);
}

static void steer_center(void)
{
  u8 w;
  for (w = 0; w < 4u; w++) servo_move(WHEEL_SERVO[w], SERVO_CENTER, 400u);
}

/* ======================================================================= */
/*  motors                                                                 */
/* ======================================================================= */
static void drive(int pwm)
{
  u8 w;
  int m[4];
  for (w = 0; w < 4u; w++) m[WHEEL_MOTOR[w]] = pwm * (int)motor_dir[w];
  Set_Pwm(m[0], m[1], m[2], m[3], 1500);
}

static void drive_stop(void)
{
  Set_Pwm(0, 0, 0, 0, 1500);
}

/* ======================================================================= */
/*  OLED                                                                  */
/* ======================================================================= */
static void screen(void)
{
  u8 i, y;
  int16_t sp;

  OLED_Clear();
  OLED_ShowString(0, 0, (u8 *)"STRAIGHT TEST");

  for (i = 0; i < 4u; i++)
  {
    y = (u8)(10u + 10u * i);

    if (i == wheel_sel) OLED_ShowString(0, y, (u8 *)">");
    else                OLED_ShowString(0, y, (u8 *)" ");

    OLED_ShowString(8, y, (u8 *)WHEEL_NAME[i]);
    OLED_ShowString(26, y, (u8 *)MOTOR_NAME[i]);
    OLED_ShowString(40, y, (u8 *)((motor_dir[i] > 0) ? "+" : "-"));

    sp = wheel_speed[i];
    if (sp < 0) sp = (int16_t)(-sp);
    OLED_ShowNumber(48, y, (u32)sp, 4, 12);
  }

  OLED_ShowString(0, 50, (u8 *)(run_active ? "RUN " : "STOP"));
  OLED_ShowString(32, 50, (u8 *)"CLK/HOLD");
  OLED_Refresh_Gram();
}

/* ======================================================================= */
/*  key (PE0, active low)                                                 */
/* ======================================================================= */
static void key_scan(void)
{
  static u8  key_last = 0u;
  static u32 t_down = 0u;
  u32 now = getSysTickCnt();
  u8  cur = (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0) == Bit_RESET) ? 1u : 0u;

  if ((cur != 0u) && (key_last == 0u))
  {
    t_down = now;
  }
  else if ((cur == 0u) && (key_last != 0u))
  {
    u32 dur = now - t_down;

    if (dur >= KEY_LONG_MS)
    {
      motor_dir[wheel_sel] = (s8)(-motor_dir[wheel_sel]);
      u1_puts("FLIP ");
      u1_puts(WHEEL_NAME[wheel_sel]);
      u1_puts(" -> ");
      u1_puts((motor_dir[wheel_sel] > 0) ? "+" : "-");
      u1_puts("\r\n");
    }
    else if (dur >= KEY_SHORT_MIN)
    {
      wheel_sel = (u8)((wheel_sel + 1u) & 3u);
      u1_puts("SEL ");
      u1_puts(WHEEL_NAME[wheel_sel]);
      u1_puts("\r\n");
    }
  }
  key_last = cur;
}

/* ======================================================================= */
/*  main                                                                  */
/* ======================================================================= */
int main(void)
{
  u32 cycle_start;
  u32 next_sample;
  u32 next_screen;
  u32 next_log;
  u32 t;
  u8  w;
  u8  was_run = 0u;

  systemInit();

  uart2_init(115200u);
  NVIC_DisableIRQ(USART2_IRQn);
  NVIC_ClearPendingIRQ(USART2_IRQn);

  u1_puts("\r\nC30D straight run test\r\n");
  u1_puts("dir default: + + + + (left wheels were reversed)\r\n");

  steer_center();
  drive_stop();
  delay_ms(1200u);

  /* 3 s countdown so the car can be aimed */
  OLED_Clear();
  OLED_ShowString(0, 0, (u8 *)"STRAIGHT TEST");
  OLED_ShowString(0, 10, (u8 *)"DIR: + + + +");
  OLED_ShowString(0, 20, (u8 *)"FORWARD 2.0s");
  OLED_ShowString(0, 30, (u8 *)"STOP    1.5s");
  OLED_ShowString(0, 40, (u8 *)"KEY: CLK/HOLD");
  OLED_ShowString(0, 50, (u8 *)"START IN 3s");
  OLED_Refresh_Gram();
  delay_ms(3000u);

  for (w = 0; w < 4u; w++) (void)Read_Encoder((u8)(2u + w));

  cycle_start = getSysTickCnt();
  next_sample = cycle_start + 100u;
  next_screen = cycle_start;
  next_log    = cycle_start + 1000u;

  while (1)
  {
    u32 now = getSysTickCnt();
    t = now - cycle_start;

    run_active = (t < RUN_MS) ? 1u : 0u;
    if (run_active != 0u) drive(RUN_PWM);
    else                  drive_stop();

    if ((was_run != 0u) && (run_active == 0u))
    {
      u1_puts("run cycle done\r\n");
    }
    was_run = run_active;

    if (t >= (RUN_MS + STOP_MS))
    {
      cycle_start = now;
      run_active = 0u;
    }

    delay_ms(10u);
    key_scan();

    if (getSysTickCnt() >= next_sample)
    {
      next_sample += 100u;
      wheel_speed[0] = (int16_t)Read_Encoder(2);
      wheel_speed[1] = (int16_t)Read_Encoder(3);
      wheel_speed[2] = (int16_t)Read_Encoder(4);
      wheel_speed[3] = (int16_t)Read_Encoder(5);
    }

    if (getSysTickCnt() >= next_screen)
    {
      next_screen += 100u;
      screen();
    }

    if (getSysTickCnt() >= next_log)
    {
      next_log += 1000u;
      u1_puts("dir: ");
      for (w = 0; w < 4u; w++)
      {
        u1_puts((motor_dir[w] > 0) ? "+" : "-");
        u1_putc(' ');
      }
      u1_puts(" spd: ");
      for (w = 0; w < 4u; w++)
      {
        u1_num((int)wheel_speed[w]);
        u1_putc(' ');
      }
      u1_puts(run_active ? " RUN\r\n" : " STOP\r\n");
    }
  }
}
