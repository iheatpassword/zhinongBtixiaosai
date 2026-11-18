/**
  * @file    main.c
  * @brief   C30D 4WD/4WS car - SERIAL COMMAND DEBUG INTERFACE
  *
  * Two USB serial ports accept the same commands:
  *   USART1 "serial1" Type-C (CH9102, PA9=TX / PA10=RX) 115200 8N1
  *   USART3 "serial3" Type-C (CH340,  PD8=TX / PD9 =RX) 115200 8N1
  * (whichever port receives a command replies on that same port)
  *
  * Commands (ASCII, CR or LF terminated, case insensitive):
  *   F / FWD / FORWARD   drive forward until S
  *   B / BACK            drive backward until S
  *   S / STOP            stop motors
  *   T / T90             all steering servos to +90 deg (broadcast ID 0xFE)
  *   T- / TM90           all steering servos to -90 deg (broadcast)
  *   C / CENTER          all steering servos to 0 deg (broadcast)
  *   H / ?               help
  *
  * Safety: if driving and no command arrives for 5 s -> STOP.
  * Motor polarity (calibrated): FL=+ FR=+ RL=+ RR=+.
  * Servo IDs: FL=4 FR=3 RL=2 RR=1, broadcast ID = 0xFE.
  *
  * Based on the WHEELTEC C30D V2.0 official source; only USER/main.c replaced,
  * the FreeRTOS scheduler is NOT started.
  */
#include "system.h"

/* ----------------------------------------------------------------------- */
/*  configuration                                                          */
/* ----------------------------------------------------------------------- */
#define SERVO_CENTER       500
#define SERVO_UNIT_PER_DEG 4.1667f
#define SERVO_MOVE_MS      1000u
#define BROADCAST_ID       0xFEu
#define CMD_PWM            2200
#define CMD_TIMEOUT_MS     5000u
#define RX_BUF_SIZE        128u

/* wheels: FL=0 FR=1 RL=2 RR=3 ; servos: FL=4 FR=3 RL=2 RR=1 */
static const u8 WHEEL_MOTOR[4] = { 0u, 1u, 2u, 3u };

/* +1 / -1 : PWM sign used as "drive forward" for that wheel (calibrated) */
static s8 motor_dir[4] = { 1, 1, 1, 1 };

static s8  motor_run;        /* 0 stop, +1 forward, -1 backward */
static u16 servo_angle;      /* last commanded steering angle magnitude */
static u32 last_cmd_ms;
static char last_cmd[8];
static u8  last_port;        /* 1 = USART1, 3 = USART3 */

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

typedef struct
{
  USART_TypeDef      *usart;
  DMA_Stream_TypeDef *stream;
  u8   buf[RX_BUF_SIZE];
  u16  last;
  char line[32];
  u8   len;
} CmdPort;

static CmdPort port1;   /* USART1 */
static CmdPort port3;   /* USART3 */

static void cmd_port_init(CmdPort *p, USART_TypeDef *u, DMA_Stream_TypeDef *s, u32 ch)
{
  DMA_InitTypeDef dma;

  p->usart  = u;
  p->stream = s;
  p->last   = 0u;
  p->len    = 0u;

  /* stop the official UART RX interrupt so the DMA can own the data */
  USART_ITConfig(u, USART_IT_RXNE, DISABLE);

  DMA_DeInit(s);
  dma.DMA_Channel            = ch;
  dma.DMA_PeripheralBaseAddr = (u32)&u->DR;
  dma.DMA_Memory0BaseAddr    = (u32)p->buf;
  dma.DMA_DIR                = DMA_DIR_PeripheralToMemory;
  dma.DMA_BufferSize         = RX_BUF_SIZE;
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
  motor_run = 0;
}

/* ======================================================================= */
/*  OLED                                                                  */
/* ======================================================================= */
static void screen(void)
{
  OLED_Clear();
  OLED_ShowString(0, 0, (u8 *)"SERIAL CMD 115200");
  OLED_ShowString(0, 10, (u8 *)"PORT:U");
  OLED_ShowNumber(48, 10, (u32)last_port, 1, 12);
  OLED_ShowString(0, 20, (u8 *)"LAST:");
  OLED_ShowString(48, 20, (u8 *)last_cmd);
  OLED_ShowString(0, 30, (u8 *)(motor_run > 0 ? "MOTOR: FWD" :
                                (motor_run < 0 ? "MOTOR: BACK" : "MOTOR: STOP")));
  OLED_ShowString(0, 40, (u8 *)"SERVO:");
  OLED_ShowNumber(48, 40, (u32)servo_angle, 3, 12);
  OLED_ShowString(80, 40, (u8 *)"deg");
  OLED_ShowString(0, 50, (u8 *)"F B S T T- C H");
  OLED_Refresh_Gram();
}

/* ======================================================================= */
/*  command execution                                                     */
/* ======================================================================= */
static void cmd_help(USART_TypeDef *u)
{
  tx_puts(u, "\r\n=== C30D serial command ===\r\n");
  tx_puts(u, "F/FWD/FORWARD : drive forward\r\n");
  tx_puts(u, "B/BACK        : drive backward\r\n");
  tx_puts(u, "S/STOP        : stop motors\r\n");
  tx_puts(u, "T/T90         : all servos +90 deg (broadcast)\r\n");
  tx_puts(u, "T-/TM90       : all servos -90 deg (broadcast)\r\n");
  tx_puts(u, "C/CENTER      : all servos 0 deg (broadcast)\r\n");
  tx_puts(u, "H/?           : this help\r\n");
  tx_puts(u, "===========================\r\n");
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

  if ((c[0] == 'F') || (c[0] == 'B') || (c[0] == 'S') ||
      (c[0] == 'T') || (c[0] == 'C') || (c[0] == 'H') || (c[0] == '?'))
  {
    last_cmd_ms = getSysTickCnt();
    last_port = (u == USART1) ? 1u : 3u;

    if (c[0] == 'F')
    {
      motor_run = 1;
      drive(CMD_PWM);
      tx_puts(u, "OK FWD\r\n");
    }
    else if (c[0] == 'B')
    {
      motor_run = -1;
      drive(-CMD_PWM);
      tx_puts(u, "OK BACK\r\n");
    }
    else if (c[0] == 'S')
    {
      drive_stop();
      tx_puts(u, "OK STOP\r\n");
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
    else
    {
      cmd_help(u);
    }
  }
  else
  {
    tx_puts(u, "ERR: H for help\r\n");
  }
}

static void cmd_poll(CmdPort *p)
{
  u16 pos = (u16)(RX_BUF_SIZE - DMA_GetCurrDataCounter(p->stream));
  u8  b;

  while (p->last != pos)
  {
    b = p->buf[p->last];
    p->last = (u16)((p->last + 1u) & (RX_BUF_SIZE - 1u));

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
  u32 next_screen;
  u32 next_log;
  u8  w;

  systemInit();

  uart2_init(115200u);
  NVIC_DisableIRQ(USART2_IRQn);
  NVIC_ClearPendingIRQ(USART2_IRQn);

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1 | RCC_AHB1Periph_DMA2, ENABLE);

  /* USART1 RX : DMA2 Stream2 / Channel4 ; USART3 RX : DMA1 Stream1 / Channel4 */
  NVIC_DisableIRQ(USART1_IRQn);
  NVIC_ClearPendingIRQ(USART1_IRQn);
  NVIC_DisableIRQ(USART3_IRQn);
  NVIC_ClearPendingIRQ(USART3_IRQn);
  cmd_port_init(&port1, USART1, DMA2_Stream2, DMA_Channel_4);
  cmd_port_init(&port3, USART3, DMA1_Stream1, DMA_Channel_4);

  tx_puts(USART1, "\r\nC30D serial command interface ready\r\n");
  tx_puts(USART3, "\r\nC30D serial command interface ready\r\n");
  cmd_help(USART1);
  cmd_help(USART3);

  servo_broadcast_deg(0);
  drive_stop();
  delay_ms(800u);

  for (w = 0; w < 4u; w++) (void)Read_Encoder((u8)(2u + w));

  last_cmd_ms = getSysTickCnt();
  next_screen = 0u;
  next_log = getSysTickCnt() + 3000u;

  while (1)
  {
    delay_ms(10u);
    cmd_poll(&port1);
    cmd_poll(&port3);

    if ((motor_run != 0) && ((getSysTickCnt() - last_cmd_ms) > CMD_TIMEOUT_MS))
    {
      drive_stop();
      tx_puts(USART1, "TIMEOUT -> STOP\r\n");
      tx_puts(USART3, "TIMEOUT -> STOP\r\n");
    }

    if (getSysTickCnt() >= next_screen)
    {
      next_screen += 200u;
      screen();
    }

    if (getSysTickCnt() >= next_log)
    {
      next_log += 3000u;
      tx_puts(USART1, motor_run > 0 ? "MOTOR: FWD\r\n" :
                      (motor_run < 0 ? "MOTOR: BACK\r\n" : "MOTOR: STOP\r\n"));
      tx_puts(USART3, motor_run > 0 ? "MOTOR: FWD\r\n" :
                      (motor_run < 0 ? "MOTOR: BACK\r\n" : "MOTOR: STOP\r\n"));
    }
  }
}
