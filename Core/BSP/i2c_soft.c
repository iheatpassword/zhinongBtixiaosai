/**
  * @file    i2c_soft.c
  * @brief   Bit-banged I2C master on PB10/PB11.
  */
#include "i2c_soft.h"
#include "board.h"
#include "main.h"       /* IMU_SCL_Pin / IMU_SDA_Pin */

#define IIC_PORT    GPIOB
#define IIC_SCL     IMU_SCL_Pin
#define IIC_SDA     IMU_SDA_Pin

#define SCL_H()     HAL_GPIO_WritePin(IIC_PORT, IIC_SCL, GPIO_PIN_SET)
#define SCL_L()     HAL_GPIO_WritePin(IIC_PORT, IIC_SCL, GPIO_PIN_RESET)
#define SDA_H()     HAL_GPIO_WritePin(IIC_PORT, IIC_SDA, GPIO_PIN_SET)
#define SDA_L()     HAL_GPIO_WritePin(IIC_PORT, IIC_SDA, GPIO_PIN_RESET)
#define SDA_READ()  (HAL_GPIO_ReadPin(IIC_PORT, IIC_SDA) == GPIO_PIN_SET)

#define IIC_DELAY() BSP_DelayUs(1)

static void sda_out(void)
{
  GPIO_InitTypeDef g = {0};

  g.Pin   = IIC_SDA;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(IIC_PORT, &g);
}

static void sda_in(void)
{
  GPIO_InitTypeDef g = {0};

  g.Pin   = IIC_SDA;
  g.Mode  = GPIO_MODE_INPUT;
  g.Pull  = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(IIC_PORT, &g);
}

static void iic_start(void)
{
  sda_out();
  SDA_H();
  IIC_DELAY();
  SCL_H();
  IIC_DELAY();
  SDA_L();          /* SDA falls while SCL is high */
  IIC_DELAY();
  SCL_L();
  IIC_DELAY();
}

static void iic_stop(void)
{
  sda_out();
  SCL_L();
  IIC_DELAY();
  SDA_L();
  IIC_DELAY();
  SCL_H();
  IIC_DELAY();
  SDA_H();          /* SDA rises while SCL is high */
  IIC_DELAY();
}

static uint8_t iic_write_byte(uint8_t b)
{
  uint8_t i;
  uint8_t ack;

  for (i = 0u; i < 8u; i++)
  {
    SCL_L();
    if ((b & 0x80u) != 0u) SDA_H(); else SDA_L();
    b = (uint8_t)(b << 1);
    IIC_DELAY();
    SCL_H();
    IIC_DELAY();
  }

  /* 9th clock: release SDA and sample the slave's ACK */
  SCL_L();
  sda_in();
  SDA_H();
  IIC_DELAY();
  SCL_H();
  IIC_DELAY();
  ack = SDA_READ() ? 0u : 1u;
  SCL_L();
  sda_out();

  return ack;
}

static uint8_t iic_read_byte(uint8_t ack)
{
  uint8_t i;
  uint8_t b = 0u;

  sda_in();
  for (i = 0u; i < 8u; i++)
  {
    SCL_L();
    IIC_DELAY();
    SCL_H();
    IIC_DELAY();
    b = (uint8_t)((b << 1) | (SDA_READ() ? 1u : 0u));
  }

  /* ACK(0) / NACK(1) from the master */
  SCL_L();
  sda_out();
  if (ack != 0u) SDA_H(); else SDA_L();
  IIC_DELAY();
  SCL_H();
  IIC_DELAY();
  SCL_L();

  return b;
}

void I2C_Soft_Init(void)
{
  sda_out();
  SCL_H();
  SDA_H();
  IIC_DELAY();
}

uint8_t I2C_Soft_Write(uint8_t dev8, uint8_t reg, uint8_t val)
{
  iic_start();
  if (iic_write_byte(dev8 & 0xFEu) == 0u) { iic_stop(); return 0u; }
  if (iic_write_byte(reg)        == 0u) { iic_stop(); return 0u; }
  if (iic_write_byte(val)        == 0u) { iic_stop(); return 0u; }
  iic_stop();
  return 1u;
}

uint8_t I2C_Soft_Read(uint8_t dev8, uint8_t reg)
{
  uint8_t v;

  iic_start();
  if (iic_write_byte(dev8 & 0xFEu) == 0u) { iic_stop(); return 0xFFu; }
  if (iic_write_byte(reg)          == 0u) { iic_stop(); return 0xFFu; }

  iic_start();
  if (iic_write_byte(dev8 | 0x01u) == 0u) { iic_stop(); return 0xFFu; }

  v = iic_read_byte(1u);      /* NACK the single byte */
  iic_stop();

  return v;
}

uint8_t I2C_Soft_ReadN(uint8_t dev8, uint8_t reg, uint8_t *buf, uint8_t n)
{
  uint8_t i;

  if ((buf == 0) || (n == 0u)) return 0u;

  iic_start();
  if (iic_write_byte(dev8 & 0xFEu) == 0u) { iic_stop(); return 0u; }
  if (iic_write_byte(reg)          == 0u) { iic_stop(); return 0u; }

  iic_start();
  if (iic_write_byte(dev8 | 0x01u) == 0u) { iic_stop(); return 0u; }

  for (i = 0u; i < n; i++)
  {
    buf[i] = iic_read_byte((uint8_t)((i + 1u < n) ? 0u : 1u));
  }
  iic_stop();

  return 1u;
}
