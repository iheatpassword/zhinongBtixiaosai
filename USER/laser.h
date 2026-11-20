/**
  * @file    laser.h
  * @brief   Laser range finder link on UART4 (PC10 = TX, PC11 = RX, 115200 8N1).
  *
  *  The authoritative interface lives in USER/app_if.h:
  *      extern volatile LaserData g_laser;
  *      void Laser_Init(void);
  *      void Laser_Poll(void);
  *      u8   Laser_IsFresh(void);
  *
  *  Wire format (10 bytes, little endian distances in mm):
  *      0xAA 0x55 | dL(2) dR(2) dF(2) | flags | CRC8
  *      0xFFFF marks an invalid distance.
  *      CRC8 polynomial 0x07, init 0x00, computed over
  *      0xAA 0x55 + the 7 payload bytes.
  *
  *  This header deliberately does NOT define the LaserData struct; it comes
  *  from app_if.h so that every module shares one definition.
  */
#ifndef __LASER_H
#define __LASER_H

#include "app_if.h"

#endif /* __LASER_H */
