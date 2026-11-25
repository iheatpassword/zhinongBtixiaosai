/**
  * @file    bsp_types.h
  * @brief   Common integer aliases shared by the BSP and APP layers.
  */
#ifndef BSP_TYPES_H
#define BSP_TYPES_H

#include <stdint.h>

typedef int8_t   s8;
typedef int16_t  s16;
typedef int32_t  s32;
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;

/* Laser / TOF sentinel: 0xFFFF means "no data on this channel". */
#define BSP_DIST_INVALID    0xFFFFu
/* VB02A over-range / no echo, treated as invalid by the consumers. */
#define BSP_DIST_OVERRANGE  2000u

#endif /* BSP_TYPES_H */
