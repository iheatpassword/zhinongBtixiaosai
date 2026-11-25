/**
  * @file    board.h
  * @brief   Board bring-up for the C30D V2.0 4WD/4WS chassis.
  *
  *  Board_Init() is the single entry point the application calls before the
  *  FreeRTOS scheduler starts.  Everything CubeMX already configured (clocks,
  *  GPIO modes, UART/TIM/ADC/DMA/IWDG init) is in Core/Src; what is added here
  *  is the part CubeMX never generates: actually STARTING the peripherals and
  *  wiring the drivers together.
  */
#ifndef BOARD_H
#define BOARD_H

#include "bsp_types.h"

/* Starts PWM, encoders, UART DMA reception, ADC, IMU and the OLED.
   Must be called after all MX_*_Init() calls and before the scheduler runs. */
void Board_Init(void);

/* PE8 heartbeat LED (push-pull, active high). */
void Board_HeartbeatToggle(void);

/* Cycle-counter based microsecond delay (DWT), used by the bit-banged I2C. */
void     BSP_DelayUs(uint32_t us);
uint32_t BSP_Ms(void);

/* Feed the IWDG.  Needed during Board_Init(): CubeMX starts the watchdog in
   MX_IWDG_Init() BEFORE MX_FREERTOS_Init(), and the bring-up below blocks for
   roughly a second (servo centring + OLED + the IMU zero-rate bias), so the
   monitor task is not there yet to feed it. */
void Board_DogFeed(void);

/* Motor driver enable on PD3 (input, active high): the hardware only drives
   the motors while this pin is high.  Must be sampled EVERY control tick -
   checking it once at boot is not enough, the line can drop mid-run. */
u8 Board_MotorEnabled(void);

#endif /* BOARD_H */
