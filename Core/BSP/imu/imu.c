/**
  * @file    imu.c
  * @brief   ICM-20948 gyro Z axis.
  *
  *  The original firmware read the chip directly through the soft I2C (the
  *  stock ICM20948.c driver only did the one-time configuration).  This file
  *  keeps that arrangement but moves the configuration in here so the two
  *  halves cannot drift apart.
  */
#include "imu.h"
#include "i2c_soft.h"
#include "board.h"      /* Board_DogFeed */
#include "main.h"       /* HAL_Delay */

/* ICM-20948 bank 0 / bank 2 registers */
#define ICM_ADDR              0xD0u
#define ICM_REG_BANK_SEL      0x7Fu
#define ICM_BANK_0            0x00u
#define ICM_BANK_2            0x20u
#define ICM_REG_PWR_MGMT_1    0x06u
#define ICM_REG_GYRO_SMPLRT   0x00u
#define ICM_REG_GYRO_CONFIG_1 0x01u
#define ICM_REG_ACCEL_SMPLRT2 0x11u
#define ICM_REG_ACCEL_CONFIG  0x14u
#define ICM_REG_GYRO_ZOUT_H   0x37u

#define IMU_BIAS_SAMPLES      200u
/* GYRO_SMPLRT_DIV=7 puts the gyro output at 1125/8 = 140.6 Hz, i.e. one new
   sample every 7.1 ms.  Sampling every 3 ms returned the same sample ~2.4 times
   and cut the averaging to ~84 independent readings; 8 ms gives 200 genuine
   ones for a bias that is three times tighter. */
#define IMU_BIAS_PERIOD_MS    8u

static float s_bias;
static float s_rate_dps;
static float s_yaw_deg;

/* The bit-banged I2C occasionally returns a corrupted byte.  One bad sample is
   integrated over a whole 20 ms tick, so a single flipped high bit is worth
   several degrees of heading and shows up as the car "jumping" while standing
   still.  Integrate the median of the last three rates instead of the newest
   one: an isolated glitch can never win the vote. */
static float s_hist[3];
static uint8_t s_hist_n, s_hist_i;

static uint8_t configured;
static uint8_t gyro_z_raw(int16_t *raw)
{
  uint8_t b[2];

  /* Sequential H+L read so both bytes come from the same sample. */
  if (I2C_Soft_ReadN(ICM_ADDR, ICM_REG_GYRO_ZOUT_H, b, 2u) == 0u)
  {
    return 0;
  }
  *raw = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
  return 1u;
}

void IMU_Init(void)
{
  uint16_t good = 0;
  int16_t raw;
  uint8_t config = 0;
  int32_t sum = 0;
  uint16_t i;

  I2C_Soft_Init();

  /* bank 0: reset, then run with the auto clock source */
  (void)I2C_Soft_Write(ICM_ADDR, ICM_REG_BANK_SEL, ICM_BANK_0);
  (void)I2C_Soft_Write(ICM_ADDR, ICM_REG_PWR_MGMT_1, 0x01u);
  HAL_Delay(10u);
  (void)I2C_Soft_Write(ICM_ADDR, ICM_REG_PWR_MGMT_1, 0x01u);

  /* bank 2: sample-rate divider, +-500 dps + DLPF, accel 2g + DLPF */
  (void)I2C_Soft_Write(ICM_ADDR, ICM_REG_BANK_SEL, ICM_BANK_2);
  (void)I2C_Soft_Write(ICM_ADDR, ICM_REG_GYRO_SMPLRT, 0x07u);
  (void)I2C_Soft_Write(ICM_ADDR, ICM_REG_GYRO_CONFIG_1,
                       0x30u | 0x02u | 0x01u);   /* DLPF_CFG=6 | 500dps | DLPF */
  (void)I2C_Soft_Write(ICM_ADDR, ICM_REG_ACCEL_SMPLRT2, 0x07u);
  (void)I2C_Soft_Write(ICM_ADDR, ICM_REG_ACCEL_CONFIG,
                       0x30u | 0x00u | 0x01u);  /* DLPF_CFG=6 | 2g | DLPF */

  configured = I2C_Soft_ReadN(ICM_ADDR, ICM_REG_GYRO_CONFIG_1, &config, 1u) && config == 0x33u;
  /* back to bank 0 for the data registers */
  (void)I2C_Soft_Write(ICM_ADDR, ICM_REG_BANK_SEL, ICM_BANK_0);
  HAL_Delay(100u);

  /* Zero-rate bias: the car must be standing still during this window.
     The IWDG is already running at this point, so keep it fed: the window is
     ~0.6 s and the watchdog window can be as short as ~0.7 s. */
  for (i = 0u; i < IMU_BIAS_SAMPLES; i++)
  {
    if (gyro_z_raw(&raw)) { sum += raw; good++; }
    HAL_Delay(IMU_BIAS_PERIOD_MS);

    if ((i & 0x1Fu) == 0x1Fu) Board_DogFeed();   /* every 32 samples */
  }

  configured = configured && good >= 180u;
  s_bias = good ? (float)sum / good : 0.0f;
  s_rate_dps = 0.0f;
  s_yaw_deg  = 0.0f;
  s_hist[0] = s_hist[1] = s_hist[2] = 0.0f;
  s_hist_n  = 0u;
  s_hist_i  = 0u;
}

uint8_t IMU_Update(float dt_s)
{
  int16_t raw;
  float r, m, a, b, c;

  if (!configured || !gyro_z_raw(&raw)) { s_rate_dps = 0; return 0; }

  r = ((float)raw - s_bias) / IMU_LSB_PER_DPS;

  s_hist[s_hist_i] = r;
  s_hist_i = (uint8_t)((s_hist_i + 1u) % 3u);
  if (s_hist_n < 3u) s_hist_n++;

  m = r;
  if (s_hist_n == 3u)
  {
    a = s_hist[0]; b = s_hist[1]; c = s_hist[2];
    m = (a < b) ? ((b < c) ? b : ((a < c) ? c : a))
                : ((a < c) ? a : ((b < c) ? c : b));
  }

  s_rate_dps  = m;
  s_yaw_deg  += m * dt_s;
  return 1;
}

float IMU_RateDps(void)
{
  return s_rate_dps;
}

float IMU_YawDeg(void)
{
  return s_yaw_deg;
}

void IMU_ResetYaw(void)
{
  s_yaw_deg = 0.0f;
  /* Drop the spike history too, so a sample taken before the reset cannot
     become the median seed of the new run. */
  s_hist[0] = s_hist[1] = s_hist[2] = 0.0f;
  s_hist_n  = 0u;
  s_hist_i  = 0u;
}
