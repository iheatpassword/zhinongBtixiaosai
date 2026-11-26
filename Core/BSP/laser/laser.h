#ifndef LASER_H
#define LASER_H

#include "bsp_types.h"
#include "laser_wire.h"

typedef struct
{
  u16 dL;              /* left  distance, mm */
  u16 dR;              /* right distance, mm */
  u16 dF;              /* front distance, mm */
  u16 dRidge;          /* aux sensor: distance to the straddled ridge side, mm */
  u8  ridge_valid;     /* aux frame received within the last 100 ms */
  u8  flags;           /* bit0/1/2 has data, bit3 timeout, bit4 ridge present */
  u8  valid;           /* a CRC-good frame arrived within the last 100 ms */
  u32 last_ms;

  u8 sample_seq[LW_CHANNELS];
  u8 sample_age[LW_CHANNELS];
} LaserData;

void Laser_Snapshot(LaserData *head, LaserData *tail);
void Laser_Init(void);      /* starts UART4 DMA circular reception */
void Laser_Poll(void);      /* call every 5..10 ms: drain, parse, timeout */


#endif /* LASER_H */
