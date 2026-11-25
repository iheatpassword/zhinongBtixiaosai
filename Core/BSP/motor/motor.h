/**
  * @file    motor.h
  * @brief   Four drive motors on TIM1/TIM9/TIM10/TIM11 (10 kHz PWM).
  *
  *  Channel map (from the WHEELTEC C30D V2.0 schematic):
  *      MotorA  forward TIM10_CH1 (PB8)   reverse TIM11_CH1 (PB9)
  *      MotorB  forward TIM9_CH1  (PE5)   reverse TIM9_CH2  (PE6)
  *      MotorC  forward TIM1_CH2  (PE11)  reverse TIM1_CH1  (PE9)
  *      MotorD  forward TIM1_CH4  (PE14)  reverse TIM1_CH3  (PE13)
  *
  *  Each channel drives one side of an H-bridge, so a signed value becomes a
  *  pair (full, full + v) exactly like the original Set_Pwm().
  */
#ifndef MOTOR_H
#define MOTOR_H

#include "bsp_types.h"

#define MOTOR_PWM_FULL  16799   /* htimX.Init.Period, verified against CubeMX */
#define MOTOR_NUM       4

void Motor_Init(void);
void Motor_Stop(void);

/* a..d: -16799..+16799, positive = that channel forward. */
void Motor_SetPwm(int a, int b, int c, int d);

#endif /* MOTOR_H */
