/**
  * @file    encoder.h
  * @brief   Quadrature encoders on TIM2..TIM5 (TI12, 65535 period).
  *
  *  Wiring verified against the original Encoder_Init_TIM2..5:
  *      wheel 0 / FL  TIM2  PA15 + PB3
  *      wheel 1 / FR  TIM3  PB4  + PB5
  *      wheel 2 / RL  TIM4  PB6  + PB7
  *      wheel 3 / RR  TIM5  PA0  + PA1
  */
#ifndef ENCODER_H
#define ENCODER_H

#include "bsp_types.h"

void Encoder_Init(void);

/* Signed counts accumulated since the previous call; the hardware counter is
   zeroed on read, exactly like the original Read_Encoder(). wheel = 0..3
   (FL, FR, RL, RR).  Returns 0 for an out-of-range index. */
int Encoder_Read(uint8_t wheel);

/* Read without clearing (diagnostics). */
int Encoder_Peek(uint8_t wheel);

#endif /* ENCODER_H */
