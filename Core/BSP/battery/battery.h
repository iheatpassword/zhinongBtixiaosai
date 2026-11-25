/**
  * @file    battery.h
  * @brief   Battery voltage on ADC1 channel 9 (PB1).
  *
  *  Named "battery" rather than "adc" on purpose: Core/Inc/adc.h already exists
  *  (CubeMX) and a second adc.h on the include path would be ambiguous.
  *
  *  Divider ratio 11:1, 3.3 V reference, 12-bit, plus the 0.99 trim the old
  *  firmware used.  Low-battery threshold in the application is 10.5 V.
  */
#ifndef BATTERY_H
#define BATTERY_H

#include "bsp_types.h"

void     Battery_Init(void);
uint16_t Battery_Raw(void);     /* 0..4095 */
float    Battery_Volt(void);    /* volts  */

#endif /* BATTERY_H */
