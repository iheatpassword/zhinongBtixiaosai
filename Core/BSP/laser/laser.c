#include "laser.h"
#include "board.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define RX_SIZE 256u
static u8 dma_buf[RX_SIZE], frame[LW_LEN];
static u16 cursor;
static u8 used;
static LaserData latest;

/* Publish only whole CRC-checked frames. The control task can preempt polling. */
static void parse(u8 byte)
{
  frame[used++] = byte;
  while (used && (frame[0] != LW_SYNC0 || (used >= 2 && frame[1] != LW_SYNC1))) {
    --used;
    memmove(frame, frame+1, used);
  }
  if (used != LW_LEN) return;
  if (lw_crc(frame,LW_CRC) != frame[LW_CRC]) {
    --used;
    memmove(frame, frame+1, used);
    return;
  }
  LaserData data = {0};
  u16 *mm[] = {&data.dL,&data.dR,&data.dF,&data.dRidge};
  for (unsigned i=0; i<LW_CHANNELS; ++i) {
    *mm[i] = frame[LW_MM+2*i] | ((u16)frame[LW_MM+2*i+1] << 8);
    data.sample_seq[i] = frame[LW_SEQ+i];
    data.sample_age[i] = frame[LW_AGE+i];
  }
  data.flags = frame[LW_FLAGS];
  data.valid = 1;
  data.last_ms = BSP_Ms();
  taskENTER_CRITICAL();
  latest = data;
  taskEXIT_CRITICAL();
  used = 0;
}

void Laser_Snapshot(LaserData *data)
{
  taskENTER_CRITICAL();
  *data = latest;
  taskEXIT_CRITICAL();
  u32 lag = BSP_Ms() - data->last_ms;
  data->valid = data->valid && lag <= LW_MAX_AGE_MS;
  u16 *mm[] = {&data->dL,&data->dR,&data->dF,&data->dRidge};
  for (unsigned i=0; i<LW_CHANNELS; ++i) {
    if (!data->valid || lag + data->sample_age[i] > LW_MAX_AGE_MS ||
        !(data->flags & lw_channel_bit(i))) {
      *mm[i] = LW_MISSING;
      data->flags &= (u8)~lw_channel_bit(i);
    }
  }
  data->ridge_valid = (data->flags & lw_channel_bit(LW_BACK)) != 0;
}

void Laser_Init(void)
{
  used = 0;
  cursor = 0;
  (void)HAL_UART_Receive_DMA(&huart4,dma_buf,RX_SIZE);
}

void Laser_Poll(void)
{
  static u32 last_poll;
  u32 now = BSP_Ms();
  if (huart4.RxState != HAL_UART_STATE_BUSY_RX || huart4.ErrorCode != HAL_UART_ERROR_NONE) {
    (void)HAL_UART_AbortReceive(&huart4);
    __HAL_UART_CLEAR_OREFLAG(&huart4);
    taskENTER_CRITICAL();
    latest.valid = 0;
    taskEXIT_CRITICAL();
    Laser_Init();
    last_poll = now;
    return;
  }
  u16 end = (RX_SIZE-__HAL_DMA_GET_COUNTER(huart4.hdmarx)) & (RX_SIZE-1u);
  if (now-last_poll > LW_MAX_AGE_MS) {
    used = 0;
    cursor = end;
  }
  last_poll = now;
  while (cursor != end) {
    parse(dma_buf[cursor]);
    cursor = (cursor+1u) & (RX_SIZE-1u);
  }
}
