/**
  * @file    encoder.c
  * @brief   Quadrature encoder reads on TIM2..TIM5.
  */
#include "encoder.h"
#include "tim.h"

void Encoder_Init(void)
{
  (void)HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  __HAL_TIM_SET_COUNTER(&htim4, 0);
  __HAL_TIM_SET_COUNTER(&htim5, 0);
}

int Encoder_Peek(uint8_t wheel)
{
  switch (wheel)
  {
    case 0u: return (int)(int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    case 1u: return (int)(int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    case 2u: return (int)(int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    case 3u: return (int)(int16_t)__HAL_TIM_GET_COUNTER(&htim5);
    default: break;
  }
  return 0;
}

int Encoder_Read(uint8_t wheel)
{
  TIM_HandleTypeDef *h;

  switch (wheel)
  {
    case 0u: h = &htim2; break;
    case 1u: h = &htim3; break;
    case 2u: h = &htim4; break;
    case 3u: h = &htim5; break;
    default: return 0;
  }

  {
    int v = (int)(int16_t)__HAL_TIM_GET_COUNTER(h);
    __HAL_TIM_SET_COUNTER(h, 0);
    return v;
  }
}
