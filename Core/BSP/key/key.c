/**
  * @file    key.c
  * @brief   PE0 user key click/long-press detector.
  */
#include "key.h"
#include "board.h"
#include "main.h"       /* USER_KEY_Pin / USER_KEY_GPIO_Port */

#define KEY_SAMPLE_MS   20u
#define KEY_LONG_MS     1200u
#define KEY_CLICK_MS    300u
#define KEY_MAX_CLICKS  3u

static uint8_t s_event;
static uint8_t s_curr;
static uint8_t s_prev;
static uint8_t s_clicks;
static uint8_t s_long_done;
static uint32_t s_last_sample;
static uint32_t s_down_ms;
static uint32_t s_up_ms;
static uint8_t  s_seen_release;   /* 1 = the key has been seen open once */

void Key_Init(void)
{
  s_event       = KEY_EV_NONE;
  s_curr        = 0u;
  s_prev        = 0u;
  s_clicks      = 0u;
  s_long_done   = 0u;
  s_last_sample = 0u;
  s_down_ms     = 0u;
  s_up_ms       = 0u;
  s_seen_release = 0u;
}

uint8_t Key_Raw(void)
{
  return (HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == GPIO_PIN_RESET)
         ? 1u : 0u;
}

uint8_t Key_Event(void)
{
  uint8_t ev = s_event;
  s_event = KEY_EV_NONE;
  return ev;
}

void Key_Tick(void)
{
  uint32_t now = BSP_Ms();
  uint8_t  raw;

  if ((uint32_t)(now - s_last_sample) < KEY_SAMPLE_MS) return;
  s_last_sample = now;

  raw    = Key_Raw();
  s_prev = s_curr;
  s_curr = raw;

  /* A key that is already closed when the firmware starts (stuck, shorted, or
     a long press across a reset) must not fire an event.  Wait until the pin
     has been seen open at least once. */
  if (s_seen_release == 0u)
  {
    if (s_curr == 0u) s_seen_release = 1u;
    s_prev = s_curr;
    return;
  }

  /* Press edge */
  if ((s_curr != 0u) && (s_prev == 0u))
  {
    s_down_ms   = now;
    s_long_done = 0u;
    if (s_clicks < KEY_MAX_CLICKS) s_clicks++;
  }

  /* Long press fires while still held */
  if ((s_curr != 0u) && (s_long_done == 0u) &&
      ((uint32_t)(now - s_down_ms) >= KEY_LONG_MS))
  {
    s_long_done = 1u;
    s_clicks    = 0u;
    s_event     = KEY_EV_LONG;
  }

  /* Release edge */
  if ((s_curr == 0u) && (s_prev != 0u))
  {
    s_up_ms = now;
  }

  /* Click burst resolved after the window expires */
  if ((s_curr == 0u) && (s_long_done == 0u) && (s_clicks > 0u) &&
      ((uint32_t)(now - s_up_ms) >= KEY_CLICK_MS))
  {
    s_event  = s_clicks;
    s_clicks = 0u;
  }
}
