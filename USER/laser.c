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
#define LASER_AUX_HDR1        0x56u
#define LASER_AUX_PAYLOAD     4u     /* id(1) d(2) flags(1)                 */
#define LASER_AUX_TAIL        5u     /* 4 payload + 1 CRC                   */
#define LASER_AUX_ID_RIDGE    1u     /* id of the ridge-side aux sensor     */
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

static u8  rx_state;                /* 0 hunt AA, 1 expect 55, 2 payload     */
static u8  rx_idx;                  /* bytes collected for the current frame */
static u8  rx_frame[LASER_FRAME_TAIL];
static u8  aux_frame[LASER_AUX_TAIL];
static u8  aux_idx;

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
static void laser_parse_byte(u8 b)
{
  u8 tmp[12];
  u8 i;

  /* raw monitor: count every byte, keep the last 8 in a rolling ring.
     raw_last[raw_idx] is the oldest byte, raw_idx is the next write slot. */
  g_laser.raw_count++;
  g_laser.raw_last[g_laser.raw_idx & 7u] = b;
  g_laser.raw_idx = (u8)((g_laser.raw_idx + 1u) & 7u);

  switch (rx_state)
  {
    case 0u:                                  /* hunting for header byte 0   */
      if (b == LASER_HDR0)
      {
        rx_state = 1u;
      }
      break;

    case 1u:                                  /* hunting for header byte 1   */
      if (b == LASER_HDR1)
      {
        rx_state = 2u;
        rx_idx   = 0u;
      }
      else if (b == LASER_AUX_HDR1)
      {
        rx_state = 3u;
        aux_idx  = 0u;
      }
      else if (b != LASER_HDR0)               /* AA AA .. restarts the hunt  */
      {
        rx_state = 0u;
      }
      break;

    case 3u:                                  /* auxiliary frame payload+CRC */
      aux_frame[aux_idx] = b;
      aux_idx++;
      if (aux_idx >= LASER_AUX_TAIL)
      {
        tmp[0] = LASER_HDR0;
        tmp[1] = LASER_AUX_HDR1;
        for (i = 0u; i < LASER_AUX_PAYLOAD; i++)
        {
          tmp[2u + i] = aux_frame[i];
        }
        if (laser_crc8(tmp, 6u) == aux_frame[LASER_AUX_PAYLOAD])
        {
          if (aux_frame[0] == LASER_AUX_ID_RIDGE)
          {
            g_laser.dRidge     = (u16)aux_frame[1] | ((u16)aux_frame[2] << 8);
            g_laser.ridge_flags = aux_frame[3];
            g_laser.ridge_ms   = App_MsNow();
            g_laser.ridge_valid = 1u;
          }
        }
        rx_state = 0u;
        aux_idx  = 0u;
      }
      break;

    default:                                  /* main frame payload + CRC    */
      rx_frame[rx_idx] = b;
      rx_idx++;
      if (rx_idx >= LASER_FRAME_TAIL)
      {
        tmp[0] = LASER_HDR0;
        tmp[1] = LASER_HDR1;
        for (i = 0u; i < LASER_PAYLOAD_LEN; i++)
        {
          tmp[2u + i] = rx_frame[i];
        }

        if (laser_crc8(tmp, 9u) == rx_frame[LASER_PAYLOAD_LEN])
        {
          g_laser.dL    = (u16)rx_frame[0] | ((u16)rx_frame[1] << 8);
          g_laser.dR    = (u16)rx_frame[2] | ((u16)rx_frame[3] << 8);
          g_laser.dF    = (u16)rx_frame[4] | ((u16)rx_frame[5] << 8);
          g_laser.flags = rx_frame[6];
          g_laser.last_ms = App_MsNow();
          g_laser.valid = 1u;
        }

        rx_state = 0u;
        rx_idx   = 0u;
      }
      break;
  }
}

/* ----------------------------------------------------------------------- */
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
  rx_state = 0u;
  rx_idx   = 0u;

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
