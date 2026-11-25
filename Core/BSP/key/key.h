/**
  * @file    key.h
  * @brief   PE0 user key: single / double / triple click and long press.
  *
  *  Ported from the old main.c key driver.  Key_Tick() is self-timed: call it
  *  as often as you like (every 5..20 ms), it samples the pin every 20 ms.
  */
#ifndef KEY_H
#define KEY_H

#include "bsp_types.h"

#define KEY_EV_NONE    0u
#define KEY_EV_SINGLE  1u
#define KEY_EV_DOUBLE  2u
#define KEY_EV_TRIPLE  3u
#define KEY_EV_LONG    4u

void    Key_Init(void);
uint8_t Key_Raw(void);      /* 1 = pressed (PE0 is active low) */
void    Key_Tick(void);     /* debounce + click state machine */
uint8_t Key_Event(void);    /* returns and clears the pending event */

#endif /* KEY_H */
