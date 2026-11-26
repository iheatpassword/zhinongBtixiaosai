/**
  * @file    servo.c
  * @brief   HTS-35H bus servo frames on USART2.
  */
#include "servo.h"
#include "usart.h"

#define SERVO_TX_TIMEOUT_MS  5u
#define SERVO_POS_READ       0x1Cu
#define SERVO_REPLY_MAX_MS   35u
#define SERVO_FRESH_MS       200u

/* Wheel -> servo id.  FL=4, FR=3, RL=2, RR=1 (measured on the car). */
static const uint8_t s_servo_id[4] = { 4u, 3u, 2u, 1u };
static uint8_t wheel, pending;
/* One six-byte request echo plus its eight-byte response fit in this window. */
static volatile uint8_t rx[24], rx_count, rx_error;
static uint32_t request_ms, sample_ms[4];
static s16 feedback[4];

static HAL_StatusTypeDef servo_send(uint8_t id, uint8_t cmd, const uint8_t *data, uint8_t n)
{
  uint8_t  buf[10];
  uint8_t  len = (uint8_t)(n + 3u);
  uint16_t sum = (uint16_t)(id + len + cmd);
  uint8_t  i;

  for (i = 0u; i < n; i++) sum = (uint16_t)(sum + data[i]);

  buf[0] = 0x55u;
  buf[1] = 0x55u;
  buf[2] = id;
  buf[3] = len;
  buf[4] = cmd;
  for (i = 0u; i < n; i++) buf[5u + i] = data[i];
  buf[5u + n] = (uint8_t)(255u - (sum & 0xFFu));

  return HAL_UART_Transmit(&huart2, buf, (uint16_t)(n + 6u),
                          SERVO_TX_TIMEOUT_MS);
}

static void servo_move(uint8_t id, uint16_t pos, uint16_t move_ms)
{
  uint8_t p[4];

  if (pos > 1000u) pos = 1000u;

  p[0] = (uint8_t)(pos & 0xFFu);
  p[1] = (uint8_t)(pos >> 8);
  p[2] = (uint8_t)(move_ms & 0xFFu);
  p[3] = (uint8_t)(move_ms >> 8);

  servo_send(id, 1u, p, 4u);
}

void Servo_Init(void)
{
  for (unsigned i=0; i<4; ++i) feedback[i]=SERVO_ANGLE_NA;
  HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void Servo_RxIRQ(void)
{
  uint32_t status=USART2->SR;
  if (!(USART2->CR1 & USART_CR1_RXNEIE)) return;
  if (!(status & (USART_SR_RXNE | USART_SR_ORE))) return;
  uint8_t byte=(uint8_t)USART2->DR; /* SR then DR also clears the UART errors */
  if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))
    rx_error=1;
  if (rx_count < sizeof rx) rx[rx_count++]=byte;
  else {
    rx_error=1;
    CLEAR_BIT(USART2->CR1, USART_CR1_RXNEIE);
  }
}

static s16 decode_position(const volatile uint8_t *buf, unsigned count, uint8_t id)
{
  for (unsigned i=0; i+8u<=count; ++i) {
    const volatile uint8_t *f=&buf[i];
    if (f[0]!=0x55 || f[1]!=0x55 || f[2]!=id || f[3]!=5) continue;
    /* Report section 2 describes status 0; the captured ID_READ echoes CMD.
       Only one unicast POS_READ is outstanding, accept either reply form. */
    if (f[4]!=SERVO_POS_READ && f[4]!=0) continue;
    unsigned sum=f[2]+f[3]+f[4]+f[5]+f[6]+f[7];
    int pos=f[5]+256*f[6];
    if ((sum & 0xffu)!=0xffu || pos>1000) continue;
    int scaled=(pos-SERVO_CENTER)*12; /* 0.24 degrees per unit, in tenths */
    return (s16)((scaled+(scaled>=0 ? 2 : -2))/5);
  }
  return SERVO_ANGLE_NA;
}

void Servo_QueryNext(void)
{
  /* Single bus owner. Drop movement echoes and errors before opening this
     wheel's RX window. Arm before TX so a fast response is not lost. */
  __HAL_UART_CLEAR_OREFLAG(&huart2);
  rx_count=0;
  rx_error=0;
  request_ms=HAL_GetTick();
  pending=1;
  SET_BIT(USART2->CR1, USART_CR1_RXNEIE);
  if (servo_send(s_servo_id[wheel],SERVO_POS_READ,NULL,0)!=HAL_OK) rx_error=1;
}

void Servo_ReadFeedback(s16 angle10[4])
{
  uint32_t now=HAL_GetTick();
  if (pending) {
    CLEAR_BIT(USART2->CR1, USART_CR1_RXNEIE);
    feedback[wheel]=SERVO_ANGLE_NA;
    if (!rx_error && now-request_ms<=SERVO_REPLY_MAX_MS) {
      feedback[wheel]=decode_position(rx,rx_count,s_servo_id[wheel]);
      sample_ms[wheel]=request_ms;
    }
    pending=0;
    wheel=(wheel+1u)%4u;
  }
  for (unsigned i=0; i<4; ++i) {
    if (now-sample_ms[i]>SERVO_FRESH_MS) feedback[i]=SERVO_ANGLE_NA;
    angle10[i]=feedback[i];
  }
}

u32 Servo_FeedbackTime(unsigned wheel_index)
{
  return sample_ms[wheel_index];
}

void Servo_Center(void)
{
  servo_move(SERVO_BROADCAST_ID, (uint16_t)SERVO_CENTER, (uint16_t)SERVO_SLOW_MS);
}

void Servo_SetAllDeg(int deg)
{
  int pos = SERVO_CENTER + (int)((float)deg * SERVO_UNIT_PER_DEG);

  if (pos < 0)    pos = 0;
  if (pos > 1000) pos = 1000;

  servo_move(SERVO_BROADCAST_ID, (uint16_t)pos, (uint16_t)SERVO_FAST_MS);
}

void Servo_SetOneDeg(uint8_t id, float deg)
{
  float units = (float)SERVO_CENTER + deg * SERVO_UNIT_PER_DEG;
  int   pos;

  if (units < 0.0f)    units = 0.0f;
  if (units > 1000.0f) units = 1000.0f;

  pos = (int)units;
  servo_move(id, (uint16_t)pos, (uint16_t)SERVO_FAST_MS);
}

void Servo_Set4Deg(float fl, float fr, float rl, float rr)
{
  Servo_SetOneDeg(s_servo_id[0], fl);
  Servo_SetOneDeg(s_servo_id[1], fr);
  Servo_SetOneDeg(s_servo_id[2], rl);
  Servo_SetOneDeg(s_servo_id[3], rr);
}
