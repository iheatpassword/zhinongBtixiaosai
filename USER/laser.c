/**
  * @file    laser.c
  * @brief   Laser range finder link on UART4: DMA1 Stream2 / Channel4
  *          circular receive + CRC8 frame parser (replaces the legacy
  *          UART5 / DMA1 Stream0 implementation).
  *
  *  Hardware
  *    PC10 = UART4_TX (AF8), PC11 = UART4_RX (AF8), 115200 8N1, no flow ctl.
  *    UART4 was not used by the legacy firmware, so nothing else owns it.
  *    The RXNE interrupt and the NVIC UART4 IRQ are explicitly disabled:
  *    the DMA stream owns all received data.
  *
  *  Frame (10 bytes)
  *      0xAA 0x55 | dL(2) dR(2) dF(2) | flags | CRC8
  *      distances little endian, mm; 0xFFFF = invalid distance
  *      CRC8 polynomial 0x07, init 0x00, over 0xAA,0x55 + 7 payload bytes
  *
  *  Frame validity
  *      g_laser.valid is set on every CRC-good frame and cleared when no good
  *      frame arrived for LASER_TIMEOUT_MS.  The sender's flags byte is copied
  *      verbatim; bit3 (timeout) is additionally set while the link is stale,
  *      matching the bit assignment documented in app_if.h.
  */
#include "laser.h"

#define LASER_BUF_SIZE        256u   /* DMA circular buffer, power of two   */
#define LASER_TIMEOUT_MS      100u   /* link considered lost after this     */

#define LASER_HDR0            0xAAu
#define LASER_HDR1            0x55u
#define LASER_V2_LEN          12u    /* AA55 + 4 distances + flags + CRC    */
#define LASER_V1_LEN          10u    /* legacy: 3 distances + flags + CRC   */
#define LASER_PAYLOAD_LEN     7u     /* dL(2) dR(2) dF(2) flags             */
#define LASER_FRAME_TAIL      8u     /* 7 payload bytes + 1 CRC byte        */

#define LASER_FLAG_TIMEOUT    0x08u  /* bit3: no frame within timeout       */
#define LASER_INVALID_MM      0xFFFFu

/* Public data, one instance for the whole firmware (declared in app_if.h). */
volatile LaserData g_laser;

/* ----------------------------------------------------------------------- */
/*  module state                                                           */
/* ----------------------------------------------------------------------- */
static u8  dma_buf[LASER_BUF_SIZE]; /* written by DMA1 Stream2              */
static u16 dma_last;                /* next buffer index to consume (0..255)*/

static u8  win[LASER_V2_LEN];
static u8  win_len;

/* ----------------------------------------------------------------------- */
/*  CRC8: polynomial 0x07, init 0x00 (same as the F103 sender)             */
/* ----------------------------------------------------------------------- */
static u8 laser_crc8(const u8 *p, u16 n)
{
  u8  crc = 0u;
  u16 i;
  u8  b;

  for (i = 0u; i < n; i++)
  {
    crc ^= p[i];
    for (b = 0u; b < 8u; b++)
    {
      crc = (crc & 0x80u) ? (u8)((crc << 1) ^ 0x07u) : (u8)(crc << 1);
    }
  }
  return crc;
}

/* ----------------------------------------------------------------------- */
/*  Consume one byte: raw monitor + 0xAA55 frame hunt + CRC check           */
/* ----------------------------------------------------------------------- */
static void win_drop(u8 n)
{
  u8 i;
  for (i = 0u; (u8)(i + n) < win_len; i++)
  {
    win[i] = win[i + n];
  }
  win_len = (u8)(win_len - n);
}

/*  Parse one complete v2 frame (12 bytes): AA55 dL dR dF dB flags crc       */
static void laser_emit_v2(const u8 *f)
{
  g_laser.dL    = (u16)f[2]  | ((u16)f[3]  << 8);
  g_laser.dR    = (u16)f[4]  | ((u16)f[5]  << 8);
  g_laser.dF    = (u16)f[6]  | ((u16)f[7]  << 8);
  g_laser.dRidge = (u16)f[8] | ((u16)f[9]  << 8);
  g_laser.flags = f[10];
  g_laser.ridge_flags = (u8)((f[10] & 0x10u) ? 1u : 0u);
  g_laser.ridge_valid = (u8)((f[10] & 0x10u) ? 1u : 0u);
  g_laser.ridge_ms = App_MsNow();
  g_laser.last_ms  = App_MsNow();
  g_laser.valid    = 1u;
}

/*  Legacy v1 frame (10 bytes): AA55 dL dR dF flags crc                     */
static void laser_emit_v1(const u8 *f)
{
  g_laser.dL    = (u16)f[2] | ((u16)f[3] << 8);
  g_laser.dR    = (u16)f[4] | ((u16)f[5] << 8);
  g_laser.dF    = (u16)f[6] | ((u16)f[7] << 8);
  g_laser.flags = f[8];
  g_laser.ridge_valid = 0u;
  g_laser.last_ms = App_MsNow();
  g_laser.valid   = 1u;
}

/* ----------------------------------------------------------------------- */
/*  Consume one byte: raw monitor + AA55 window hunt (v2 first, then v1)    */
/* ----------------------------------------------------------------------- */
static void laser_parse_byte(u8 b)
{
  /* raw monitor: count every byte, keep the last 8 in a rolling ring. */
  g_laser.raw_count++;
  g_laser.raw_last[g_laser.raw_idx & 7u] = b;
  g_laser.raw_idx = (u8)((g_laser.raw_idx + 1u) & 7u);

  if (win_len < LASER_V2_LEN)
  {
    win[win_len] = b;
    win_len++;
  }
  else
  {
    win_drop(1u);
    win[LASER_V2_LEN - 1u] = b;
  }

  while ((win_len >= 2u) && !((win[0] == LASER_HDR0) && (win[1] == LASER_HDR1)))
  {
    win_drop(1u);
  }

  if (win_len >= LASER_V1_LEN)
  {
    if (laser_crc8(win, 9u) == win[9])
    {
      laser_emit_v1(win);
      win_len = 0u;
      return;
    }
  }

  if (win_len >= LASER_V2_LEN)
  {
    if (laser_crc8(win, 11u) == win[11])
    {
      laser_emit_v2(win);
      win_len = 0u;
      return;
    }
    win_drop(1u);
  }
}
/*  Initialise GPIOC + UART4 + DMA1 Stream2/Channel4 (UART4_RX)            */
/* ----------------------------------------------------------------------- */
void Laser_Init(void)
{
  GPIO_InitTypeDef  gpio;
  USART_InitTypeDef usart;
  DMA_InitTypeDef   dma;
  u8 i;

  /* --- clocks: GPIOC + UART4 (APB1) ----------------------------------- */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

  /* --- PC10 / PC11 as UART4 alternate function (AF8) ------------------ */
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);

  gpio.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
  gpio.GPIO_Mode  = GPIO_Mode_AF;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  gpio.GPIO_PuPd  = GPIO_PuPd_UP;   /* keep an unconnected RX idle high    */
  GPIO_Init(GPIOC, &gpio);

  /* --- UART4: 115200 8N1, RX + TX ------------------------------------- */
  usart.USART_BaudRate            = 115200u;
  usart.USART_WordLength          = USART_WordLength_8b;
  usart.USART_StopBits            = USART_StopBits_1;
  usart.USART_Parity              = USART_Parity_No;
  usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(UART4, &usart);
  USART_Cmd(UART4, ENABLE);

  /* --- DMA owns the RX data: keep the UART interrupt path quiet ------- */
  USART_ITConfig(UART4, USART_IT_RXNE, DISABLE);
  NVIC_DisableIRQ(UART4_IRQn);
  NVIC_ClearPendingIRQ(UART4_IRQn);

  /* --- DMA1 Stream2 / Channel4 = UART4_RX, circular, byte/byte -------- */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
  DMA_DeInit(DMA1_Stream2);

  dma.DMA_Channel            = DMA_Channel_4;
  dma.DMA_PeripheralBaseAddr = (u32)&UART4->DR;
  dma.DMA_Memory0BaseAddr    = (u32)dma_buf;
  dma.DMA_DIR                = DMA_DIR_PeripheralToMemory;
  dma.DMA_BufferSize         = LASER_BUF_SIZE;
  dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
  dma.DMA_Mode               = DMA_Mode_Circular;
  dma.DMA_Priority           = DMA_Priority_Medium;
  dma.DMA_FIFOMode           = DMA_FIFOMode_Disable;
  dma.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
  dma.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  dma.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream2, &dma);
  DMA_Cmd(DMA1_Stream2, ENABLE);
  USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);

  /* --- parser + public state ------------------------------------------ */
  dma_last = 0u;
  win_len = 0u;

  g_laser.dL    = LASER_INVALID_MM;
  g_laser.dR    = LASER_INVALID_MM;
  g_laser.dF    = LASER_INVALID_MM;
  g_laser.flags = LASER_FLAG_TIMEOUT;
  g_laser.valid = 0u;
  g_laser.last_ms = 0u;
  g_laser.raw_count = 0u;
  g_laser.raw_idx = 0u;
  for (i = 0u; i < 8u; i++)
  {
    g_laser.raw_last[i] = 0u;
  }
}

/* ----------------------------------------------------------------------- */
/*  Poll: drain new DMA bytes, then apply the 100 ms freshness timeout.     */
/*  Call every 5..10 ms.  At 115200 baud the 256-byte buffer wraps in       */
/*  ~22 ms, so a 10 ms poll never loses a whole frame in practice.          */
/* ----------------------------------------------------------------------- */
void Laser_Poll(void)
{
  u16 pos;
  u8  b;

  /* number of bytes written since DMA start, modulo the circular length.
     The mask maps a counter value of 0 (exactly 256 bytes written) to 0. */
  pos = (u16)((LASER_BUF_SIZE - DMA_GetCurrDataCounter(DMA1_Stream2))
              & (LASER_BUF_SIZE - 1u));

  while (dma_last != pos)
  {
    b = dma_buf[dma_last];
    dma_last = (u16)((dma_last + 1u) & (LASER_BUF_SIZE - 1u));
    laser_parse_byte(b);
  }

  if ((u32)(App_MsNow() - g_laser.last_ms) > LASER_TIMEOUT_MS)
  {
    g_laser.valid = 0u;
    g_laser.flags |= LASER_FLAG_TIMEOUT;
  }

  if ((u32)(App_MsNow() - g_laser.ridge_ms) > LASER_TIMEOUT_MS)
  {
    g_laser.ridge_valid = 0u;
  }
}

/* ----------------------------------------------------------------------- */
/*  1 while a CRC-good frame arrived within the last 100 ms                 */
/* ----------------------------------------------------------------------- */
u8 Laser_IsFresh(void)
{
  return g_laser.valid;
}
