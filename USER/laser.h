/**
  * @file    laser.h
  * @brief   UART5 laser link (F103 -> C30D), with raw byte monitor
  *          Expected frame: AA 55 | dL(2) dR(2) dF(2) | flags | CRC8
  */
#ifndef __LASER_H
#define __LASER_H

#include "system.h"

typedef struct
{
  u16 dL;
  u16 dR;
  u16 dF;
  u8  flags;
  u8  valid;
  u32 last_ms;
  /* raw monitor (for wiring / protocol debugging) */
  u16 raw_count;      /* total bytes received on UART5                 */
  u8  raw_last[8];    /* last 8 received bytes (rolling)               */
  u8  raw_idx;
} Laser_Data_t;

extern volatile Laser_Data_t Laser;

void Laser_Init(void);
void Laser_Poll(void);        /* call every 5..10 ms */
u8   LaserIsFresh(void);

#endif
