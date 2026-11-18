/**
  * @file    main.c
  * @brief   C30D 4WD/4WS car - ENCODER SPEED CLOSED LOOP + serial commands
  *
  * Four drive motors are speed-controlled (PID, 50 Hz) from the four encoders:
  *     MotorA -> E1 (TIM2)   FL
  *     MotorB -> E2 (TIM3)   FR
  *     MotorC -> E3 (TIM4)   RL
  *     MotorD -> E4 (TIM5)   RR
  * Speed is measured as encoder counts per 20 ms control period.
  *
  * Serial (USART1 "serial1" CH9102 / COM7, USART3 "serial3", 115200 8N1):
  *   F            drive forward, speed closed loop at target
  *   B            drive backward
  *   S            stop (also resets PID)
  *   V<n>         set target speed, counts per 20 ms, e.g. V120
  *   T / T-       steering servos +90 / -90 deg (broadcast)
  *   C            steering servos centre (0 deg)
  *   H / ?        help
  *
  * Safety: 5 s without a movement command -> stop.
  * Motor polarity (calibrated): FL=-1 FR=-1 RL=+1 RR=+1.
  *
  * Based on the WHEELTEC C30D V2.0 official source; only USER/main.c replaced,
  * the FreeRTOS scheduler is NOT started.
  */
#include "system.h"
#include "laser.h"

/* ----------------------------------------------------------------------- */
/*  configuration                                                          */
/* ----------------------------------------------------------------------- */
#define CTRL_MS            20u      /* speed loop period                    */
#define SPD_DEFAULT        150      /* counts per CTRL_MS                   */
#define SPD_MAX            600
#define KP_NUM             10       /* PWM = KP_NUM*err/10 + integral       */
#define KI_DIV             8        /* integral += err/KI_DIV               */
#define PWM_SLEW           400      /* max PWM change per 20 ms tick        */
#define PWM_FF             250      /* feed-forward base (dead zone)        */
#define INTEG_MAX          10000
#define PWM_LIM            12000
#define CMD_TIMEOUT_MS     5000u

#define SERVO_CENTER       500
#define SERVO_UNIT_PER_DEG 4.1667f
#define SERVO_MOVE_MS      1000u
#define BROADCAST_ID       0xFEu

/* wheels: FL=0 FR=1 RL=2 RR=3 ; servos: FL=4 FR=3 RL=2 RR=1 */
static const u8 WHEEL_MOTOR[4] = { 0u, 1u, 2u, 3u };

/* +1 / -1 : PWM sign used as "drive forward" for that wheel (calibrated) */
static s8 motor_dir[4] = { -1, -1, 1, 1 };

static s8  motor_run;        /* 0 stop, +1 forward, -1 backward */
static s16 spd_target;       /* counts per control period */
static u8  open_loop;        /* 1 = fixed PWM mode (diagnostic) */
static s16 open_pwm;
static u16 servo_angle;
static u32 last_cmd_ms;
static char last_cmd[8];
static u8  last_port;

/* speed loop state */
static s16 meas_spd[4];      /* filtered absolute counts per period */
static s16 prev_raw[4];
static s16 pwm_out[4];
static s32 integral[4];

/* ======================================================================= */
/*  1 ms time base (TIM6) - the official ms_now() does not run in    */
/*  bare metal because the FreeRTOS tick counter is only incremented by the */
/*  scheduler, so provide our own millisecond counter.                      */
/* ======================================================================= */
static volatile u32 ms_cnt;

void TIM6_DAC_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    ms_cnt++;
  }
}

static u32 ms_now(void)
{
  return ms_cnt;
}

static void ms_init(void)
{
  TIM_TimeBaseInitTypeDef tb;
  NVIC_InitTypeDef nv;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

  tb.TIM_Prescaler     = 84u - 1u;      /* 84 MHz / 84 = 1 MHz          */
  tb.TIM_Period        = 1000u - 1u;    /* 1 MHz / 1000 = 1 kHz (1 ms)  */
  tb.TIM_CounterMode   = TIM_CounterMode_Up;
  tb.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM6, &tb);
  TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

  nv.NVIC_IRQChannel                   = TIM6_DAC_IRQn;
  nv.NVIC_IRQChannelPreemptionPriority = 2u;
  nv.NVIC_IRQChannelSubPriority        = 0u;
  nv.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&nv);

  TIM_Cmd(TIM6, ENABLE);
}

/* ======================================================================= */
/*  generic UART transmit / receive                                        */
/* ======================================================================= */
static void tx_putc(USART_TypeDef *u, u8 b)
{
  while (USART_GetFlagStatus(u, USART_FLAG_TXE) == RESET) { }
  USART_SendData(u, b);
}

static void tx_puts(USART_TypeDef *u, const char *s)
{
  while (*s != '\0') tx_putc(u, (u8)*s++);
}

static void tx_num(USART_TypeDef *u, int v)
{
  char b[10];
  u8 n = 0;
  if (v < 0) { tx_putc(u, '-'); v = -v; }
  if (v == 0) { tx_putc(u, '0'); return; }
  while (v > 0) { b[n++] = (char)('0' + (v % 10)); v /= 10; }
  while (n > 0) tx_putc(u, (u8)b[--n]);
}

static void tx_hexb(USART_TypeDef *u, u8 v)
{
  const char *h = "0123456789ABCDEF";
  tx_putc(u, (u8)h[v >> 4]);
  tx_putc(u, (u8)h[v & 0x0Fu]);
}

typedef struct
{
  USART_TypeDef      *usart;
  DMA_Stream_TypeDef *stream;
  u8   buf[128];
  u16  last;
  char line[32];
  u8   len;
} CmdPort;

static CmdPort port1;
static CmdPort port3;

static void cmd_port_init(CmdPort *p, USART_TypeDef *u, DMA_Stream_TypeDef *s, u32 ch)
{
  DMA_InitTypeDef dma;

  p->usart  = u;
  p->stream = s;
  p->last   = 0u;
  p->len    = 0u;

  USART_ITConfig(u, USART_IT_RXNE, DISABLE);

  DMA_DeInit(s);
  dma.DMA_Channel            = ch;
  dma.DMA_PeripheralBaseAddr = (u32)&u->DR;
  dma.DMA_Memory0BaseAddr    = (u32)p->buf;
  dma.DMA_DIR                = DMA_DIR_PeripheralToMemory;
  dma.DMA_BufferSize         = 128u;
  dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
  dma.DMA_Mode               = DMA_Mode_Circular;
  dma.DMA_Priority           = DMA_Priority_High;
  dma.DMA_FIFOMode           = DMA_FIFOMode_Disable;
  dma.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
  dma.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  dma.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
  DMA_Init(s, &dma);
  DMA_Cmd(s, ENABLE);
  USART_DMACmd(u, USART_DMAReq_Rx, ENABLE);
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

static void servo_broadcast_deg(int deg)
{
  s16 pos = (s16)((float)SERVO_CENTER + (float)deg * SERVO_UNIT_PER_DEG);
  if (pos < 0)    pos = 0;
  if (pos > 1000) pos = 1000;
  servo_move(BROADCAST_ID, (u16)pos, SERVO_MOVE_MS);
  servo_angle = (u16)(deg < 0 ? -deg : deg);
}

/* ======================================================================= */
/*  speed loop                                                            */
/* ======================================================================= */
static void speed_reset(void)
{
  u8 w;
  for (w = 0; w < 4u; w++)
  {
    (void)Read_Encoder((u8)(2u + w));   /* clear counter */
    meas_spd[w] = 0;
    prev_raw[w] = 0;
    pwm_out[w]  = 0;
    integral[w] = 0;
  }
  Set_Pwm(0, 0, 0, 0, 1500);
}

static void speed_apply(void)
{
  int m[4];
  u8  w;
  int p;

  for (w = 0; w < 4u; w++)
  {
    if (motor_run == 0) { m[WHEEL_MOTOR[w]] = 0; continue; }
    p = pwm_out[w];
    p = (motor_run > 0) ? p : -p;
    m[WHEEL_MOTOR[w]] = p * (int)motor_dir[w];
  }
  Set_Pwm(m[0], m[1], m[2], m[3], 1500);
}

/* one speed-loop tick: measure encoders, PID, apply PWM (50 Hz) */
static void speed_tick(void)
{
  u8  w;
  int d = 0, e, p;
  int raw;

  for (w = 0; w < 4u; w++)
  {
    raw = Read_Encoder((u8)(2u + w));
    if (raw < 0) raw = -raw;
    if (raw > 32000) raw = 32000;
    d = raw;

    if (motor_run == 0)
    {
      integral[w] = 0;
      pwm_out[w]  = 0;
      continue;
    }

    if (open_loop != 0u)
    {
      prev_raw[w] = (s16)d;
      meas_spd[w] = (s16)d;
      pwm_out[w]  = open_pwm;
      continue;
    }

    /* simple 2-point average to reduce encoder quantisation noise */
    meas_spd[w] = (s16)((d + prev_raw[w]) / 2);
    prev_raw[w] = (s16)d;

    e = (int)spd_target - (int)meas_spd[w];
    integral[w] += (s32)e / KI_DIV;
    if (integral[w] > INTEG_MAX)  integral[w] = INTEG_MAX;
    if (integral[w] < -INTEG_MAX) integral[w] = -INTEG_MAX;

    /* feed-forward: measured speed ~ 0.4*PWM - 100  =>  PWM ~ 2.5*spd + 250 */
    p = (KP_NUM * e) / 10 + (int)integral[w] + PWM_FF + ((int)spd_target * 25) / 10;
    if (p > PWM_LIM) p = PWM_LIM;
    if (p < 0)       p = 0;

    /* slew-rate limit to avoid oscillation from wheel inertia */
    if (p > (int)pwm_out[w] + PWM_SLEW) p = (int)pwm_out[w] + PWM_SLEW;
    if (p < (int)pwm_out[w] - PWM_SLEW) p = (int)pwm_out[w] - PWM_SLEW;

    pwm_out[w] = (s16)p;
  }
  speed_apply();
}

/* ======================================================================= */
/*  OLED                                                                  */
/* ======================================================================= */
static void screen(void)
{
  u8 i, y;

  OLED_Clear();
  OLED_ShowString(0, 0, (u8 *)"SPEED LOOP 50Hz");

  for (i = 0; i < 4u; i++)
  {
    y = (u8)(10u + 10u * i);
    OLED_ShowString(0, y, (u8 *)((i == 0u) ? "FL" : (i == 1u) ? "FR" : (i == 2u) ? "RL" : "RR"));
    OLED_ShowNumber(18, y, (u32)meas_spd[i], 3, 12);
    OLED_ShowString(38, y, (u8 *)"/");
    OLED_ShowNumber(44, y, (u32)spd_target, 3, 12);
    OLED_ShowNumber(70, y, (u32)pwm_out[i], 4, 12);
  }

  OLED_ShowString(0, 50, (u8 *)(motor_run > 0 ? "FWD " :
                                (motor_run < 0 ? "BACK" : "STOP")));
  OLED_ShowString(40, 50, (u8 *)"V");
  OLED_ShowNumber(48, 50, (u32)spd_target, 3, 12);
  OLED_Refresh_Gram();
}

/* ======================================================================= */
/*  commands                                                              */
/* ======================================================================= */
static void cmd_help(USART_TypeDef *u)
{
  tx_puts(u, "\r\n=== C30D speed-loop command ===\r\n");
  tx_puts(u, "F       : forward (speed closed loop)\r\n");
  tx_puts(u, "B       : backward\r\n");
  tx_puts(u, "S       : stop\r\n");
  tx_puts(u, "V<n>    : target speed counts/20ms, e.g. V120\r\n");
  tx_puts(u, "O<n>    : open-loop fixed PWM (diagnostic), e.g. O2500\r\n");
  tx_puts(u, "T / T-  : servos +90 / -90 deg (broadcast)\r\n");
  tx_puts(u, "C       : servos centre\r\n");
  tx_puts(u, "H / ?   : help\r\n");
  tx_puts(u, "===============================\r\n");
}

static int parse_num(const char *s)
{
  int v = 0;
  u8  i = 0u;
  while ((s[i] >= '0') && (s[i] <= '9')) { v = v * 10 + (s[i] - '0'); i++; }
  return v;
}

static void cmd_exec(USART_TypeDef *u, const char *line)
{
  char c[16];
  u8  i = 0u;
  u8  j;

  while ((line[i] != '\0') && (i < 15u))
  {
    c[i] = line[i];
    if ((c[i] >= 'a') && (c[i] <= 'z')) c[i] = (char)(c[i] - 'a' + 'A');
    i++;
  }
  c[i] = '\0';
  if (i == 0u) return;

  for (j = 0u; (j < 7u) && (c[j] != '\0'); j++) last_cmd[j] = c[j];
  last_cmd[j] = '\0';
  last_cmd_ms = ms_now();
  last_port = (u == USART1) ? 1u : 3u;

  if (c[0] == 'F')
  {
    open_loop = 0u;
    motor_run = 1;
    speed_reset();
    tx_puts(u, "OK FWD\r\n");
  }
  else if (c[0] == 'B')
  {
    open_loop = 0u;
    motor_run = -1;
    speed_reset();
    tx_puts(u, "OK BACK\r\n");
  }
  else if ((c[0] == 'S') || (c[0] == 'X'))
  {
    open_loop = 0u;
    motor_run = 0;
    speed_reset();
    tx_puts(u, "OK STOP\r\n");
  }
  else if (c[0] == 'O')
  {
    int v = parse_num(&c[1]);
    if ((v >= 0) && (v <= 12000))
    {
      open_loop = 1u;
      open_pwm = (s16)v;
      motor_run = 1;
      speed_reset();
      tx_puts(u, "OK OPEN PWM=");
      tx_num(u, v);
      tx_puts(u, "\r\n");
    }
    else tx_puts(u, "ERR: O 0..12000\r\n");
  }
  else if (c[0] == 'V')
  {
    int v = parse_num(&c[1]);
    if ((v >= 0) && (v <= SPD_MAX))
    {
      spd_target = (s16)v;
      tx_puts(u, "OK V=");
      tx_num(u, v);
      tx_puts(u, "\r\n");
    }
    else tx_puts(u, "ERR: V 0..600\r\n");
  }
  else if (c[0] == 'T')
  {
    if (c[1] == '-') { servo_broadcast_deg(-90); tx_puts(u, "OK SERVO -90\r\n"); }
    else             { servo_broadcast_deg(+90); tx_puts(u, "OK SERVO +90\r\n"); }
  }
  else if (c[0] == 'C')
  {
    servo_broadcast_deg(0);
    tx_puts(u, "OK SERVO 0\r\n");
  }
  else if ((c[0] == 'H') || (c[0] == '?'))
  {
    cmd_help(u);
  }
  else
  {
    tx_puts(u, "ERR: H for help\r\n");
  }
}

static void cmd_poll(CmdPort *p)
{
  u16 pos = (u16)(128u - DMA_GetCurrDataCounter(p->stream));
  u8  b;

  while (p->last != pos)
  {
    b = p->buf[p->last];
    p->last = (u16)((p->last + 1u) & 127u);

    if ((b == '\r') || (b == '\n'))
    {
      if (p->len > 0u)
      {
        p->line[p->len] = '\0';
        p->len = 0u;
        cmd_exec(p->usart, p->line);
      }
    }
    else if (p->len < (u8)(sizeof(p->line) - 1u))
    {
      p->line[p->len++] = (char)b;
    }
  }
}

/* ======================================================================= */
/*  main                                                                  */
/* ======================================================================= */
int main(void)
{
  u32 next_ctrl;
  u32 next_screen;
  u32 next_log;
  u32 now;

  ms_init();

  systemInit();

  Laser_Init();

  uart2_init(115200u);
  NVIC_DisableIRQ(USART2_IRQn);
  NVIC_ClearPendingIRQ(USART2_IRQn);

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1 | RCC_AHB1Periph_DMA2, ENABLE);
  NVIC_DisableIRQ(USART1_IRQn);
  NVIC_ClearPendingIRQ(USART1_IRQn);
  NVIC_DisableIRQ(USART3_IRQn);
  NVIC_ClearPendingIRQ(USART3_IRQn);
  cmd_port_init(&port1, USART1, DMA2_Stream2, DMA_Channel_4);
  cmd_port_init(&port3, USART3, DMA1_Stream1, DMA_Channel_4);

  tx_puts(USART1, "\r\nC30D speed-loop ready\r\n");
  tx_puts(USART3, "\r\nC30D speed-loop ready\r\n");
  cmd_help(USART1);
  cmd_help(USART3);

  servo_broadcast_deg(0);
  spd_target = SPD_DEFAULT;
  motor_run = 0;
  speed_reset();
  delay_ms(500u);

  next_ctrl   = ms_now();
  next_screen = ms_now();
  next_log    = ms_now() + 1000u;

  while (1)
  {
    now = ms_now();

    if (now >= next_ctrl)
    {
      next_ctrl += CTRL_MS;
      if ((s32)(now - next_ctrl) > 200) next_ctrl = now + CTRL_MS;  /* catch up */
      speed_tick();
    }

    delay_ms(5u);
    Laser_Poll();
    cmd_poll(&port1);
    cmd_poll(&port3);

    if ((motor_run != 0) && ((ms_now() - last_cmd_ms) > CMD_TIMEOUT_MS))
    {
      motor_run = 0;
      speed_reset();
      tx_puts(USART1, "TIMEOUT -> STOP\r\n");
      tx_puts(USART3, "TIMEOUT -> STOP\r\n");
    }

    if (ms_now() >= next_screen)
    {
      next_screen += 200u;
      screen();
    }

    if (ms_now() >= next_log)
    {
      u8 k, w;
      next_log += 1000u;
      for (k = 0u; k < 2u; k++)
      {
        USART_TypeDef *u = (k == 0u) ? USART1 : USART3;
        tx_puts(u, (open_loop != 0u) ? "OL PWM=" : (motor_run > 0 ? "SPD FWD " : (motor_run < 0 ? "SPD BACK " : "SPD STOP ")));
        if (open_loop != 0u) { tx_num(u, (int)open_pwm); tx_putc(u, ' '); }
        tx_puts(u, "tgt=");
        tx_num(u, (int)spd_target);
        tx_puts(u, " ");
        for (w = 0u; w < 4u; w++)
        {
          tx_num(u, (int)meas_spd[w]);
          tx_putc(u, '/');
          tx_num(u, (int)pwm_out[w]);
          tx_putc(u, ' ');
        }
        tx_puts(u, "SVO=");
        tx_num(u, (int)servo_angle);
        tx_puts(u, " P=");
        tx_num(u, (int)last_port);
        tx_puts(u, " CMD=");
        tx_puts(u, last_cmd);
        tx_puts(u, " LZ=");
        tx_num(u, (int)Laser.raw_count);
        tx_putc(u, ':');
        for (w = 0u; w < 8u; w++)
        {
          tx_hexb(u, Laser.raw_last[(u8)((Laser.raw_idx + w) & 7u)]);
          tx_putc(u, ' ');
        }
        tx_puts(u, "\r\n");
      }
    }
  }
}
