/**
  * @file    oled.h
  * @brief   0.96" 128x64 OLED, software SPI on GPIOD.
  *
  *  Pin map (identical to the original HARDWARE/oled.c):
  *      PD12 = RST   PD11 = DC
  *      PD14 = SCLK  PD13 = SDIN (MOSI)
  *
  *  Only the pin macros changed: the original used bit-band writes
  *  (PDout(n) = 1), which depended on the StdPeriph sys.h.  The drawing code
  *  in oled.c is untouched.
  */
#ifndef __OLED_H
#define __OLED_H

#include "bsp_types.h"
#include "main.h"       /* OLED_*_Pin / OLED_*_GPIO_Port from CubeMX */

extern u8 OLED_GRAM[128][8];

#define OLED_RST_Clr()   HAL_GPIO_WritePin(OLED_RST_GPIO_Port,  OLED_RST_Pin,  GPIO_PIN_RESET)
#define OLED_RST_Set()   HAL_GPIO_WritePin(OLED_RST_GPIO_Port,  OLED_RST_Pin,  GPIO_PIN_SET)
#define OLED_RS_Clr()    HAL_GPIO_WritePin(OLED_DC_GPIO_Port,   OLED_DC_Pin,   GPIO_PIN_RESET)
#define OLED_RS_Set()    HAL_GPIO_WritePin(OLED_DC_GPIO_Port,   OLED_DC_Pin,   GPIO_PIN_SET)
#define OLED_SCLK_Clr()  HAL_GPIO_WritePin(OLED_SCK_GPIO_Port,  OLED_SCK_Pin,  GPIO_PIN_RESET)
#define OLED_SCLK_Set()  HAL_GPIO_WritePin(OLED_SCK_GPIO_Port,  OLED_SCK_Pin,  GPIO_PIN_SET)
#define OLED_SDIN_Clr()  HAL_GPIO_WritePin(OLED_MOSI_GPIO_Port, OLED_MOSI_Pin, GPIO_PIN_RESET)
#define OLED_SDIN_Set()  HAL_GPIO_WritePin(OLED_MOSI_GPIO_Port, OLED_MOSI_Pin, GPIO_PIN_SET)

#define OLED_CMD   0
#define OLED_DATA  1

#define OLED_Height  64
#define OLED_Width   128

#define CNSizeWidth   16
#define CNSizeHeight  16

void OLED_WR_Byte(u8 dat, u8 cmd);
void OLED_Refresh_Gram(void);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x, u8 y, u8 t);
void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 size, u8 mode);
void OLED_ShowNumber(u8 x, u8 y, u32 num, u8 len, u8 size);
void OLED_ShowString(u8 x, u8 y, const u8 *p);

#endif /* __OLED_H */
