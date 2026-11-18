/**
  * @file    laser.c
  * @brief   UART5 laser link: DMA1 Stream0 / Channel4 circular receive,
  *          frame parser with CRC8 check.
  *
  *  Frame (10 bytes):
  *    0xAA 0x55 dL_L dL_H dR_L dR_H dF_L dF_H flags crc8
  *  CRC8: polynomial 0x07, init 0x00, over 0xAA 0x55 + 7 payload bytes.
  */
#include "laser.h"

#define LASER_BUF_SIZE   256u
#define LASER_TIMEOUT_MS 100u

volatile Laser_Data_t Laser;

static u8  dma_buf[LASER_BUF_SIZE];
static u16 dma_last;
static u8  rx_state;
static u8  rx_idx;
static u8  rx_frame[8];

static u8 crc8(const u8 *p, u16 n)
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

static void laser_parse_byte(u8 b)
{
  u8 tmp[9];
  u8 i;

  /* raw monitor: count every byte and keep the last 8 */
  Laser.raw_count++;
  Laser.raw_last[Laser.raw_idx & 7u] = b;
  Laser.raw_idx = (u8)((Laser.raw_idx + 1u) & 7u);

  switch (rx_state)
  {
    case 0u:
      if (b == 0xAAu) rx_state = 1u;
      break;

    case 1u:
      if (b == 0x55u)      rx_state = 2u;
      else if (b == 0xAAu) rx_state = 1u;
      else                 rx_state = 0u;
      break;

    default:
      rx_frame[rx_idx++] = b;
      if (rx_idx >= 8u)
      {
        tmp[0] = 0xAAu;
        tmp[1] = 0x55u;
        for (i = 0u; i < 7u; i++) tmp[2u + i] = rx_frame[i];

        if (crc8(tmp, 9u) == rx_frame[7])
        {
          Laser.dL = (u16)rx_frame[0] | ((u16)rx_frame[1] << 8);
          Laser.dR = (u16)rx_frame[2] | ((u16)rx_frame[3] << 8);
          Laser.dF = (u16)rx_frame[4] | ((u16)rx_frame[5] << 8);
          Laser.flags = rx_frame[6];
          Laser.last_ms = getSysTickCnt();
          Laser.valid = 1u;
        }
        rx_state = 0u;
        rx_idx = 0u;
      }
      break;
  }
}

void Laser_Init(void)
{
  DMA_InitTypeDef dma;
  u8 i;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

  /* stop the official USART5 RX interrupt so the DMA can own the data */
  USART_ITConfig(UART5, USART_IT_RXNE, DISABLE);
  NVIC_DisableIRQ(UART5_IRQn);
  NVIC_ClearPendingIRQ(UART5_IRQn);

  DMA_DeInit(DMA1_Stream0);
  dma.DMA_Channel            = DMA_Channel_4;
  dma.DMA_PeripheralBaseAddr = (u32)&UART5->DR;
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
  DMA_Init(DMA1_Stream0, &dma);
  DMA_Cmd(DMA1_Stream0, ENABLE);
  USART_DMACmd(UART5, USART_DMAReq_Rx, ENABLE);

  dma_last = 0u;
  rx_state = 0u;
  rx_idx   = 0u;

  Laser.dL = 0xFFFFu;
  Laser.dR = 0xFFFFu;
  Laser.dF = 0xFFFFu;
  Laser.flags = 0u;
  Laser.valid = 0u;
  Laser.last_ms = 0u;
  Laser.raw_count = 0u;
  Laser.raw_idx = 0u;
  for (i = 0u; i < 8u; i++) Laser.raw_last[i] = 0u;
}

void Laser_Poll(void)
{
  u16 pos = (u16)(LASER_BUF_SIZE - DMA_GetCurrDataCounter(DMA1_Stream0));

  while (dma_last != pos)
  {
    laser_parse_byte(dma_buf[dma_last]);
    dma_last = (u16)((dma_last + 1u) & (LASER_BUF_SIZE - 1u));
  }

  if ((getSysTickCnt() - Laser.last_ms) > LASER_TIMEOUT_MS)
  {
    Laser.valid = 0u;
  }
}

u8 LaserIsFresh(void)
{
  return Laser.valid;
}
