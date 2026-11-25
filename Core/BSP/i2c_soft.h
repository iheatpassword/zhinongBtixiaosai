/**
  * @file    i2c_soft.h
  * @brief   Bit-banged I2C master on PB10 (SCL) / PB11 (SDA) for the ICM-20948.
  *
  *  Kept as software I2C on purpose: the original firmware used it and the
  *  timing on this board is already proven.  No pull-up configuration is
  *  needed here - CubeMX sets both pins to open-drain-less push-pull with
  *  internal pull-ups (IMU_SCL_Pin / IMU_SDA_Pin in main.h).
  *
  *  Device addresses are the 8-bit form used by the sensor datasheets
  *  (ICM-20948 = 0xD0 for write), matching the original driver.
  */
#ifndef I2C_SOFT_H
#define I2C_SOFT_H

#include "bsp_types.h"

void I2C_Soft_Init(void);

/* All return 1 on ACK, 0 on NACK/timeout. */
uint8_t I2C_Soft_Write(uint8_t dev8, uint8_t reg, uint8_t val);
uint8_t I2C_Soft_Read(uint8_t dev8, uint8_t reg);

/* Sequential read of n bytes (register pointer auto-increments).  Used for
   the gyro so the high and low byte always come from the same sample. */
uint8_t I2C_Soft_ReadN(uint8_t dev8, uint8_t reg, uint8_t *buf, uint8_t n);

#endif /* I2C_SOFT_H */
