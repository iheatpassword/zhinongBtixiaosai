/**
  * @file    imu.h
  * @brief   ICM-20948 gyro Z axis, heading integration.
  *
  *  The range MUST stay at +-500 dps with the DLPF on, because IMU_LSB_PER_DPS
  *  below assumes 32768 / 500 = 65.5 counts per deg/s.  The chip powers up at
  *  +-250 dps (131 LSB/dps), so IMU_Init() writes GYRO_CONFIG_1 = 0x33.
  *  Losing that write makes every heading reading 2x too large.
  */
#ifndef IMU_H
#define IMU_H

#include "bsp_types.h"

#define IMU_LSB_PER_DPS  65.5f

void  IMU_Init(void);          /* config + ~0.7 s zero-rate bias, blocking */

/* Read the gyro exactly once and integrate the heading.  dt_s is the real
   control period (0.02 for the 50 Hz task) - integrate with the measured
   period, never with a hard-coded one. */
uint8_t IMU_Update(float dt_s);

float IMU_RateDps(void);
float IMU_YawDeg(void);
void  IMU_ResetYaw(void);

#endif /* IMU_H */
