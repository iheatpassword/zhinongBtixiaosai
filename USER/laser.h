/**
  * @file    laser.h
  * @brief   UART5 laser range finder link (F103 -> C30D)
  *          Frame: AA 55 | dL(2) dR(2) dF(2) | flags | CRC8
  *          distances in mm, little endian, 0xFFFF = invalid
  *          flags: bit0 front valid, bit1 left valid, bit2 right valid, bit3 comm timeout
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
  u8  valid;      /* frame received within the last 100 ms */
  u32 last_ms;
} Laser_Data_t;

extern volatile Laser_Data_t Laser;

void Laser_Init(void);
void Laser_Poll(void);        /* call every 5..10 ms */
u8   LaserIsFresh(void);

#endif
