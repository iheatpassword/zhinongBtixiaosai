/**
  * @file    motor.c
  * @brief   Four drive motor PWM outputs.
  */
#include "motor.h"
#include "tim.h"

/* Split a signed demand into the (forward, reverse) CCR pair.
   Kept identical in behaviour to the original BALANCE/balance.c Set_Pwm(). */
static void hbridge(int v, uint32_t *fwd, uint32_t *rev)
{
  if (v < 0)
  {
    *fwd = (uint32_t)MOTOR_PWM_FULL;
    *rev = (uint32_t)(MOTOR_PWM_FULL + v);
  }
  else
  {
    *rev = (uint32_t)MOTOR_PWM_FULL;
    *fwd = (uint32_t)(MOTOR_PWM_FULL - v);
  }
}

void Motor_Init(void)
{
  /* Advanced timer TIM1 needs its main output enabled; HAL_TIM_PWM_Start
     does that for us (MOE). */
  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  (void)HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

  (void)HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);

  Motor_Stop();
}

void Motor_Stop(void)
{
  Motor_SetPwm(0, 0, 0, 0);
}

void Motor_SetPwm(int a, int b, int c, int d)
{
  uint32_t a1, a2, b1, b2, c1, c2, d1, d2;

  hbridge(a, &a1, &a2);   /* PWMA1 / PWMA2 */
  hbridge(b, &b1, &b2);   /* PWMB1 / PWMB2 */
  hbridge(c, &c1, &c2);   /* PWMC1 / PWMC2 */
  hbridge(d, &d1, &d2);   /* PWMD1 / PWMD2 */

  __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, a1);
  __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, a2);

  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, b1);
  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, b2);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, c1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, c2);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, d1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, d2);
}
