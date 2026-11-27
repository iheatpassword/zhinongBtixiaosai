#include "laser.h"
#include "board.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* Two F1 boards, two UARTs, no daisy chain.
     UART4 (PC11) <- head board, sends AA 57 frames
     USART6 (PC7) <- tail board, sends AA 58 frames
   A chained link (tail -> head -> F4) was the original design, but both boards
   transmit continuously with no arbitration, so sharing one wire loses frames
   and the head board has to relay the tail's bytes with no flow control.  Each
   board now owns a UART and each has its own byte-stream state, so a partial
   frame on one link can never corrupt the other. */
#define RX_SIZE 256u

typedef struct {
  u8  buf[RX_SIZE];
  u16 cursor;
  u32 last_poll;
  u8  frame[LW_LEN];
  u8  used;
} LaserStream;

static LaserStream s_head, s_tail;
static LaserData latest[2];  /* independent frame clocks, head then tail */

/* Publish only whole CRC-checked frames. The control task can preempt polling. */
static void parse(LaserStream *st, u8 byte)
{
  st->frame[st->used++] = byte;
  while (st->used && (st->frame[0] != LW_SYNC0 ||
         (st->used >= 2 && st->frame[1] != LW_SYNC1 && st->frame[1] != LW_REAR_SYNC1))) {
    --st->used;
    memmove(st->frame, st->frame+1, st->used);
  }
  if (st->used != LW_LEN) return;
  if (lw_crc(st->frame,LW_CRC) != st->frame[LW_CRC]) {
    --st->used;
    memmove(st->frame, st->frame+1, st->used);
    return;
  }
  LaserData data = {0};
  u16 *mm[] = {&data.dL,&data.dR,&data.dF,&data.dRidge};
  for (unsigned i=0; i<LW_CHANNELS; ++i) {
    *mm[i] = st->frame[LW_MM+2*i] | ((u16)st->frame[LW_MM+2*i+1] << 8);
    data.sample_seq[i] = st->frame[LW_SEQ+i];
    data.sample_age[i] = st->frame[LW_AGE+i];
  }
  data.flags = st->frame[LW_FLAGS];
  data.valid = 1;
  data.last_ms = BSP_Ms();
  taskENTER_CRITICAL();
  latest[st->frame[1] == LW_REAR_SYNC1] = data;
  taskEXIT_CRITICAL();
  st->used = 0;
}

static void expire_channels(LaserData *data, u32 now)
{
  u32 lag = now - data->last_ms;
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

void Laser_Snapshot(LaserData *head, LaserData *tail)
{
  taskENTER_CRITICAL();
  *head = latest[0];
  *tail = latest[1];
  u32 now = BSP_Ms();
  taskEXIT_CRITICAL();
  expire_channels(head, now);
  expire_channels(tail, now);
}

void Laser_Init(void)
{
  memset(&s_head, 0, sizeof s_head);
  memset(&s_tail, 0, sizeof s_tail);
  (void)HAL_UART_Receive_DMA(&huart4, s_head.buf, RX_SIZE);
  (void)HAL_UART_Receive_DMA(&huart6, s_tail.buf, RX_SIZE);
}

static void poll(LaserStream *st, UART_HandleTypeDef *h, unsigned idx)
{
  u32 now = BSP_Ms();
  if (h->RxState != HAL_UART_STATE_BUSY_RX || h->ErrorCode != HAL_UART_ERROR_NONE) {
    (void)HAL_UART_AbortReceive(h);
    __HAL_UART_CLEAR_OREFLAG(h);
    taskENTER_CRITICAL();
    latest[idx].valid = 0;
    taskEXIT_CRITICAL();
    st->used = 0;
    st->cursor = 0;
    st->last_poll = now;
    (void)HAL_UART_Receive_DMA(h, st->buf, RX_SIZE);
    return;
  }
  u16 end = (u16)((RX_SIZE-__HAL_DMA_GET_COUNTER(h->hdmarx)) & (RX_SIZE-1u));
  if (now - st->last_poll > LW_MAX_AGE_MS) {
    st->used = 0;
    st->cursor = end;
  }
  st->last_poll = now;
  while (st->cursor != end) {
    parse(st, st->buf[st->cursor]);
    st->cursor = (u16)((st->cursor+1u) & (RX_SIZE-1u));
  }
}

void Laser_Poll(void)
{
  poll(&s_head, &huart4, 0u);
  poll(&s_tail, &huart6, 1u);
}
