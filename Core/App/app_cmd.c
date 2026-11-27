/**
  * @file    app_cmd.c
  * @brief   Serial command ports and key mapping.
  *
  *  Commands (one line, case insensitive, CR or LF terminated):
  *      F / B        forward / backward at the base speed
  *      S / X / Q    stop, clear every mode
  *      V<n>         set base speed, counts per 20 ms
  *      A            start the 5-ridge competition FSM
  *      E            4WS edge-follow test
  *      W / W-       crab right / left
  *      T / T-       all servos +90 / -90 deg
  *      C            all servos centred
  *      H / ?        help
  */
#include "tune_params.h"
#include "app_cmd.h"
#include "app_config.h"
#include "app_if.h"
#include "app_report.h"
#include "app_tasks.h"
#include <string.h>
#include <stdio.h>

#include "usart.h"      /* huart1 / huart3 / huart5 */
#include "key/key.h"

#define CMD_BUF_SIZE   128u

typedef struct
{
  UART_HandleTypeDef *h;
  u8   buf[CMD_BUF_SIZE];
  u16  last;
  char line[CMD_LINE_MAX];
  u8   discard;
  u8   len;
  u8   id;
} CmdPort;

static CmdPort s_p5;

/* Last accepted command, for the on-screen / log diagnostic. */
static char s_last_cmd;
static u8   s_last_port;

char AppCmd_LastChar(void)
{
  return s_last_cmd;
}

u8 AppCmd_LastPort(void)
{
  return s_last_port;
}

static void cmd_help(void)
{
  App_Puts("\r\n=== C30D 4WS ===\r\n");
  App_Puts("F/B   forward / backward\r\n");
  App_Puts("S     stop (clears all modes)\r\n");
  App_Puts("V<n>  base speed counts/20ms, e.g. V900\r\n");
  App_Puts("A     start FSM2 competition sequence\r\n");
  App_Puts("E     4WS edge-follow test\r\n");
  App_Puts("W/W-  crab right / left\r\n");
  App_Puts("T/T-  servos +90 / -90,  C centre\r\n");
  App_Puts("P LIST | P GET key | P SET key value | P RESET\r\n");
  App_Puts("H/?   help\r\n");
  App_Puts("================\r\n");
}

static int parse_int(const char *s)
{
  int v = 0;

  while ((*s >= '0') && (*s <= '9')) { v = v * 10 + (*s - '0'); s++; }
  return v;
}

static void cmd_exec(char *line)
{
  u8 i;

  for (i = 0u; (line[i] != '\0') && (i < (CMD_LINE_MAX - 1u)); i++)
  {
    if ((line[i] >= 'a') && (line[i] <= 'z')) line[i] = (char)(line[i] - 'a' + 'A');
  }

  if (i == 0u) return;
  if (strcmp(line, "J6") == 0 || strcmp(line, "J-6") == 0) {
    App_PostCmd(line[1] == '-' ? APP_CMD_SERVO_M6 : APP_CMD_SERVO_P6);
    return;
  }
  if (strcmp(line, "J?") == 0) {
    AppInputs in;
    char out[144];
    App_ReadInputs(&in);
    snprintf(out, sizeof out, "STEP MS=%lu T=%d ACT=%d/%d/%d/%d AT=%lu/%lu/%lu/%lu\r\n",
             (unsigned long)in.ms, in.steer_target10[0],
             in.steer_feedback10[0], in.steer_feedback10[1], in.steer_feedback10[2], in.steer_feedback10[3],
             (unsigned long)in.steer_sample_ms[0], (unsigned long)in.steer_sample_ms[1],
             (unsigned long)in.steer_sample_ms[2], (unsigned long)in.steer_sample_ms[3]);
    App_Puts(out);
    return;
  }
  if (line[0] == 'P' && (line[1] == ' ' || !line[1])) { Tune_Command(line); return; }

  /* Shape check: one command letter, then only digits / '-' / spaces.
     A binary frame from a sensor wired to a command port is full of printable
     bytes too, so "reject non-printable" alone is not enough - this pattern
     check is what actually stops a laser frame from being read as "F". */
  {
    u8 j;

    for (j = 1u; j < i; j++)
    {
      char c = line[j];

      if (!(((c >= '0') && (c <= '9')) || (c == '-') || (c == ' ')))
      {
        return;
      }
    }
  }

  s_last_cmd = line[0];

  switch (line[0])
  {
    case 'F': App_PostCmd(APP_CMD_FWD);   App_Puts("OK FWD\r\n");        break;
    case 'B': App_PostCmd(APP_CMD_BACK);  App_Puts("OK BACK\r\n");       break;
    case 'S':
    case 'X':
    case 'Q': App_PostCmd(APP_CMD_STOP);  App_Puts("OK STOP\r\n");       break;
    case 'A': App_PostCmd(APP_CMD_FSM2);  App_Puts("OK FSM2\r\n");       break;
    case 'E': App_PostCmd(APP_CMD_EDGE);  App_Puts("OK EDGE\r\n");       break;
    case 'W':
      if (line[1] == '-') { App_PostCmd(APP_CMD_CRAB_L); App_Puts("OK CRAB L\r\n"); }
      else                { App_PostCmd(APP_CMD_CRAB_R); App_Puts("OK CRAB R\r\n"); }
      break;
    case 'T':
      if (line[1] == '-') { App_PostCmd(APP_CMD_SERVO_M90); App_Puts("OK -90\r\n"); }
      else                { App_PostCmd(APP_CMD_SERVO_P90); App_Puts("OK +90\r\n"); }
      break;
    case 'C': App_PostCmd(APP_CMD_SERVO_CENTER); App_Puts("OK SERVO 0\r\n"); break;
    case 'R':
    {
      int mm = (line[1] == '\0') ? (int)REV_EDGE_DEFAULT_MM : parse_int(&line[1]);
      if (mm < 100) mm = 100;
      App_SetRevTargetCm((u16)(mm / 10));
      App_PostCmd(APP_CMD_REV_EDGE);
      App_Puts("OK REV ");
      App_PutInt(mm);
      App_Puts("\r\n");
      break;
    }
    case 'V':
    {
      int v = parse_int(&line[1]);
      App_SetBaseSpeed((s16)v);
      App_Puts("OK V=");
      App_PutInt(v);
      App_Puts("\r\n");
      break;
    }
    case 'H':
    case '?': cmd_help(); break;
    default:
      App_Puts("ERR: H for help\r\n");
      break;
  }
}

static void port_poll(CmdPort *p)
{
  u16 pos = (u16)((CMD_BUF_SIZE - __HAL_DMA_GET_COUNTER(p->h->hdmarx))
                  & (CMD_BUF_SIZE - 1u));
  u8  b;

  while (p->last != pos)
  {
    b = p->buf[p->last];
    p->last = (u16)((p->last + 1u) & (CMD_BUF_SIZE - 1u));

    if ((b == '\r') || (b == '\n'))
    {
      if (p->discard) { p->discard = 0u; p->len = 0u; continue; }
      if (p->len > 0u)
      {
        p->line[p->len] = '\0';
        p->len = 0u;
        s_last_port = p->id;
        cmd_exec(p->line);
      }
    }
    else if ((b < 0x20u) || (b > 0x7Eu))
    {
      /* Binary traffic on a command port (mis-wired sensor, noise, wrong
         baud): throw the partial line away so it can never be turned into a
         command.  Without this, any frame whose byte happens to be 'F' or 'B'
         followed by CR/LF would start the car. */
      p->discard = 1u;
      p->len = 0u;
    }
    else if (p->len < (u8)(CMD_LINE_MAX - 1u))
    {
      p->line[p->len++] = (char)b;
    }
    else p->discard = 1u;
  }
}

static void port_start(CmdPort *p, UART_HandleTypeDef *h, u8 id)
{
  p->h    = h;
  p->last = 0u;
  p->len  = 0u;
  p->id   = id;
  (void)HAL_UART_Receive_DMA(h, p->buf, CMD_BUF_SIZE);
}

void AppCmd_Init(void)
{
  s_last_cmd  = ' ';
  s_last_port = 0u;

  /* Console on UART5 only: USART3 and UART4 now carry the two F1 laser
     uplinks, so they must not also be command ports. */
  port_start(&s_p5, &huart5, 5u);
}

void AppCmd_Poll(void)
{
  port_poll(&s_p5);
  Tune_PollList();
}

void AppCmd_HandleKey(u8 ev)
{
  switch (ev)
  {
    case KEY_EV_SINGLE: App_PostCmd(APP_CMD_FSM2);     break;
    case KEY_EV_DOUBLE: App_PostCmd(APP_CMD_EDGE);     break;
    case KEY_EV_TRIPLE: App_PostCmd(APP_CMD_CRAB_R);   break;
    case KEY_EV_LONG:   App_PostCmd(APP_CMD_WHEELTEST);break;
    default: break;
  }
}
