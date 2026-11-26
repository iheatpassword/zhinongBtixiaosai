/**
  * @file    tune_store.c
  * @brief   Persist the tuning table as an append-only log in flash.
  *
  * Layout: sector 7 (0x08060000), the last 128 KB sector of the 512 KB
  * STM32F407VE, split into 512 slots of 256 bytes.  A save programs the next
  * blank slot instead of erasing, because a sector erase on this single-bank
  * part stalls every instruction fetch: the CPU cannot run, so it cannot feed
  * the 1 s IWDG, and IWDG_RLR cannot be widened while the watchdog is running.
  * 512 saves is far more than a tuning session ever needs, and a full sector is
  * reported (RC=-7) rather than silently losing the write.
  *
  * The linker script stops the image at 384 KB so it can never grow into the
  * sector.  A record that is interrupted anywhere fails its CRC, so the loader
  * skips it and falls back to the previous one instead of running on half a
  * record.
  */
#include "tune_store.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#define TUNE_STORE_ADDR     0x08060000u   /* sector 7, past the 384 KB image */
#define TUNE_STORE_MAGIC    0x54554E45u   /* "TUNE" */
#define TUNE_STORE_LAYOUT   1u
#define TUNE_SLOT_SIZE      320u
#define TUNE_SLOT_COUNT     409u          /* 409 * 320 = 130,880 of 131,072 */
#define TUNE_BLANK_WORD     0xFFFFFFFFu

typedef struct {
  uint32_t magic;
  uint16_t layout;
  uint16_t bytes;
  uint32_t revision;
  uint32_t crc;                           /* over the values that follow */
} StoreHeader;

static uint8_t s_rec[TUNE_SLOT_SIZE];
static TuneValues s_check;

/* Adding parameters must never overflow a slot. */
typedef char tune_slot_fits[
  (sizeof(StoreHeader) + sizeof(TuneValues) <= TUNE_SLOT_SIZE) ? 1 : -1];

static uint32_t crc32(const uint8_t *p, uint32_t n)
{
  uint32_t c = 0xFFFFFFFFu;

  while (n--)
  {
    c ^= *p++;
    for (unsigned b = 0u; b < 8u; b++)
      c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
  }
  return ~c;
}

static uint32_t slot_addr(uint32_t index)
{
  return TUNE_STORE_ADDR + index * TUNE_SLOT_SIZE;
}

static int slot_valid(uint32_t index)
{
  const StoreHeader *h    = (const StoreHeader *)slot_addr(index);
  const uint8_t     *body = (const uint8_t *)slot_addr(index) + sizeof(StoreHeader);

  if (h->magic  != TUNE_STORE_MAGIC) return 0;
  if (h->layout != TUNE_STORE_LAYOUT) return 0;
  if (h->bytes  != (uint16_t)sizeof(TuneValues)) return 0;
  return crc32(body, sizeof(TuneValues)) == h->crc;
}

int Tune_Store_Load(TuneValues *out)
{
  /* Newest slot wins; a corrupted tail (interrupted save) is skipped. */
  for (uint32_t i = TUNE_SLOT_COUNT; i-- > 0u; )
  {
    if (!slot_valid(i)) continue;
    if (out != 0)
      memcpy(out, (const void *)(slot_addr(i) + sizeof(StoreHeader)), sizeof(TuneValues));
    return 1;
  }
  return 0;
}

int Tune_Store_Save(const TuneValues *in, uint32_t revision)
{
  StoreHeader *h;
  uint32_t index;

  if (in == 0) return -1;

  for (index = 0u; index < TUNE_SLOT_COUNT; index++)
  {
    if (*(const volatile uint32_t *)slot_addr(index) == TUNE_BLANK_WORD) break;
  }
  if (index >= TUNE_SLOT_COUNT) return -7;     /* sector full: needs an erase */

  memset(s_rec, 0, sizeof s_rec);
  h = (StoreHeader *)s_rec;
  h->magic    = TUNE_STORE_MAGIC;
  h->layout   = (uint16_t)TUNE_STORE_LAYOUT;
  h->bytes    = (uint16_t)sizeof(TuneValues);
  h->revision = revision;
  memcpy(s_rec + sizeof(StoreHeader), in, sizeof(TuneValues));
  h->crc = crc32(s_rec + sizeof(StoreHeader), sizeof(TuneValues));

  if (HAL_FLASH_Unlock() != HAL_OK) return -2;

  for (uint32_t off = 0u; off < TUNE_SLOT_SIZE; off += 4u)
  {
    uint32_t w;
    memcpy(&w, s_rec + off, sizeof w);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, slot_addr(index) + off, w) != HAL_OK)
    {
      (void)HAL_FLASH_Lock();
      return -4;
    }
  }

  (void)HAL_FLASH_Lock();

  /* Verify through the same path the loader uses, then compare contents. */
  if (!slot_valid(index)) return -5;
  memcpy(&s_check, (const void *)(slot_addr(index) + sizeof(StoreHeader)), sizeof(TuneValues));
  if (memcmp(&s_check, in, sizeof(TuneValues)) != 0) return -6;

  return 0;
}
