/**
  * @file    board.c
  * @brief   Board bring-up: starts every peripheral and wires the drivers.
  */
#include "board.h"

#include "main.h"        /* LED_Pin / user labels from CubeMX */
#include "tim.h"         /* htim1..htim12 */
#include "usart.h"       /* huart1..huart5 */
#include "adc.h"         /* hadc1 */
#include "iwdg.h"        /* hiwdg */

#include "motor/motor.h"
#include "encoder/encoder.h"
#include "servo/servo.h"
#include "imu/imu.h"
#include "laser/laser.h"
#include "oled/oled.h"
#include "battery/battery.h"
#include "key/key.h"

/* ======================================================================= */
/*  timing                                                                 */
/* ======================================================================= */

static void dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT      = 0u;
  DWT->CTRL       |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t BSP_Ms(void)
{
  return HAL_GetTick();
}

void BSP_DelayUs(uint32_t us)
{
  uint32_t start;
  uint32_t ticks;
  uint32_t guard;

  if (us == 0u) return;

  start = DWT->CYCCNT;
  ticks = us * (SystemCoreClock / 1000000u);
  if (ticks < 8u) ticks = 8u;

  /* The guard makes this safe even if CYCCNT is not running: the loop would
     otherwise spin forever, which is exactly what a hung bring-up looks like
     from the debugger. */
  for (guard = 0u; guard < (ticks * 4u + 256u); guard++)
  {
    if ((uint32_t)(DWT->CYCCNT - start) >= ticks) break;
  }
}

void Board_DogFeed(void)
{
  (void)HAL_IWDG_Refresh(&hiwdg);
}

u8 Board_MotorEnabled(void)
{
  /* PD3 is configured as an input with pull-up by MX_GPIO_Init().
     HIGH = the motor driver is enabled. */
  return (HAL_GPIO_ReadPin(ENABLE_SW_GPIO_Port, ENABLE_SW_Pin) == GPIO_PIN_SET)
         ? 1u : 0u;
}

/* ======================================================================= */
/*  heartbeat                                                              */
/* ======================================================================= */

void Board_HeartbeatToggle(void)
{
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

/* ======================================================================= */
/*  bring-up                                                               */
/* ======================================================================= */

void Board_Init(void)
{
  uint8_t i;

  dwt_init();

  /* The IWDG is already armed at this point (MX_IWDG_Init() runs earlier in
     main.c) with a ~1 s window, and this whole function blocks for about the
     same amount of time.  Feed it at every stage boundary, otherwise the part
     resets in the middle of the IMU bias and never reaches the scheduler. */
  Board_DogFeed();

  /* Actuators first: PWM must be running before any CCR is written, and the
     servos are centred before anything can command an angle. */
  Motor_Init();
  Encoder_Init();
  Servo_Init();

  /* Repeat the centre command: if the previous boot reset mid-crab, the
     servos may still be holding +-90 deg. */
  for (i = 0u; i < 3u; i++)
  {
    Servo_Center();
    HAL_Delay(50u);
    Board_DogFeed();
  }

  /* Human interface */
  OLED_Init();
  Key_Init();
  Board_DogFeed();

  /* Sensors */
  Battery_Init();
  IMU_Init();          /* blocks ~0.7 s and feeds the dog while it does */
  Laser_Init();

  Board_DogFeed();
}
