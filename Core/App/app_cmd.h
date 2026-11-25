/**
  * @file    app_cmd.h
  * @brief   Serial command ports (USART1 / USART3 / UART5) and user key.
  *
  *  CubeMX already configured the three UARTs and their circular RX DMA
  *  (USART1 -> DMA2 Stream2, USART3 -> DMA1 Stream1, UART5 -> DMA1 Stream0).
  *  This module starts the transfers and turns bytes into AppCmd requests.
  */
#ifndef APP_CMD_H
#define APP_CMD_H

#include "bsp_types.h"

void AppCmd_Init(void);
void AppCmd_Poll(void);                 /* drain + parse, cmd task */

/* Map a key event (KEY_EV_*) to an App_PostCmd() request. */
void AppCmd_HandleKey(u8 ev);

/* Last accepted command letter and the port it came from (0 = none). */
char AppCmd_LastChar(void);
u8   AppCmd_LastPort(void);

#endif /* APP_CMD_H */
