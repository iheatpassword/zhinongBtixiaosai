#ifndef LASER_WIRE_H
#define LASER_WIRE_H
#include <stdint.h>

/* F103 -> F407, 115200 8N1. Fixed versioned frame. Each project is standalone.
 * mm[4] little endian, flags, sample counters[4], sample ages in ms[4], CRC8.
 * The channel order in all three arrays is left, right, forward, rear-facing.
 * 2000 is an observed no-echo on VB02A; 0xffff denotes missing data.
 */
enum { LW_LEFT, LW_RIGHT, LW_FRONT, LW_BACK, LW_CHANNELS };
enum { LW_SYNC0=0xaa, LW_SYNC1=0x57, LW_MM=2, LW_FLAGS=10, LW_SEQ=11,
       LW_AGE=15, LW_CRC=19, LW_LEN=20, LW_MAX_AGE_MS=100, LW_MISSING=0xffff };
static inline uint8_t lw_channel_bit(unsigned i)
{
  static const uint8_t bit[LW_CHANNELS] = {2,4,1,16};
  return bit[i];
}
static inline uint8_t lw_crc(const uint8_t *p, unsigned n)
{
  uint8_t c=0;
  while (n--) {
    c ^= *p++;
    for (unsigned b=0; b<8; b++) c = (uint8_t)((c<<1) ^ ((c&0x80u) ? 7u : 0u));
  }
  return c;
}
#endif
