/**
  * @file    servo.h
  * @brief   HTS-35H bus servos on USART2 (PD5/PD6, 115200, broadcast 0xFE).
  *
  *  Frame: 0x55 0x55 <id> <len> <cmd> <data...> <checksum>
  *         len      = data bytes + 3
  *         checksum = 255 - (id + len + cmd + sum(data))
  *  Position   : 500 + angle * 4.1667, centre 500, range 0..1000
  *  Polarity   : positive angle = clockwise seen from above (i.e. to the right)
  */
#ifndef SERVO_H
#define SERVO_H

#include "bsp_types.h"

#define SERVO_CENTER        500
#define SERVO_UNIT_PER_DEG  4.1667f
#define SERVO_BROADCAST_ID  0xFEu

/* Travel time written into every move command.  The old firmware used 150 ms;
   it dominates the steering latency budget (see docs/MIGRATION_PLAN.md 1.3),
   so lower it to 40..60 ms once the loops are re-tuned. */
#define SERVO_FAST_MS       150u
#define SERVO_SLOW_MS       300u

void Servo_Init(void);
void Servo_Center(void);
void Servo_SetAllDeg(int deg);                          /* broadcast */
void Servo_SetOneDeg(uint8_t id, float deg);
void Servo_Set4Deg(float fl, float fr, float rl, float rr);

#endif /* SERVO_H */
