/**
  * @file    servo.c
  * @brief   HTS-35H bus servo frames on USART2.
  */
#include "servo.h"
#include "usart.h"

#define SERVO_TX_TIMEOUT_MS  5u

/* Wheel -> servo id.  FL=4, FR=3, RL=2, RR=1 (measured on the car). */
static const uint8_t s_servo_id[4] = { 4u, 3u, 2u, 1u };

static void servo_send(uint8_t id, uint8_t cmd, const uint8_t *data, uint8_t n)
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

  (void)HAL_UART_Transmit(&huart2, buf, (uint16_t)(n + 6u),
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
  /* CubeMX already ran MX_USART2_UART_Init(). Nothing to configure here;
     the bus is write-only from the MCU side. */
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
