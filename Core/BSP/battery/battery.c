/**
  * @file    battery.c
  * @brief   Battery voltage on ADC1 channel 9 (PB1).
  */
#include "battery.h"
#include "adc.h"        /* hadc1 (CubeMX) */

#define BAT_ADC_TIMEOUT_MS  10u

void Battery_Init(void)
{
  /* No ADC calibration on STM32F4 (that is an F1/F3 concept); just discard
     the first conversion so the sample-and-hold has settled. */
  (void)Battery_Raw();
}

uint16_t Battery_Raw(void)
{
  uint16_t v = 0u;

  (void)HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, BAT_ADC_TIMEOUT_MS) == HAL_OK)
  {
    v = (uint16_t)HAL_ADC_GetValue(&hadc1);
  }
  (void)HAL_ADC_Stop(&hadc1);

  return v;
}

float Battery_Volt(void)
{
  /* Same expression the old firmware used for Get_Adc(9) in main.c. */
  return (float)Battery_Raw() * 3.3f * 11.0f * 0.99f / 4096.0f;
}
