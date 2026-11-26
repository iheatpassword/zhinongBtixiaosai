/* Same ASCII command ports as A. Numeric wire values use at most three decimals. */
#include "tune_params.h"
#include "app_config.h"
#include "app_report.h"
#include "centering.h"
#include "safety.h"
#include "app_if.h"
#include "tune_store.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

typedef struct {
  const char *name;
  size_t offset;
  int fractional, low, high;
} Param;

static const Param params[] = {
#define TUNE_I(name, field, initial, lo, hi) {#name, offsetof(TuneValues, field), 0, lo, hi},
#define TUNE_F(name, field, initial, lo, hi) {#name, offsetof(TuneValues, field), 1, lo, hi},
#include "tune_params.def"
#undef TUNE_I
#undef TUNE_F
};
static const TuneValues factory = {
#define TUNE_I(name, field, initial, lo, hi) .field = initial,
#define TUNE_F(name, field, initial, lo, hi) .field = initial,
#include "tune_params.def"
#undef TUNE_I
#undef TUNE_F
};
static TuneValues live;
const volatile TuneValues *const app_tune = &live;
static volatile uint32_t revision;
static int list_index = -1;
static uint32_t list_revision;
#define PARAM_COUNT ((int)(sizeof(params)/sizeof(params[0])))

static int consistent(const TuneValues *v);   /* defined below */

void Tune_Init(void)
{
  /* A saved set beats the factory table, but only if it still satisfies the
     cross-field limits of this firmware revision; otherwise fall back. */
  if (Tune_Store_Load(&live) && consistent(&live))
  {
    revision = 1u;
    return;
  }
  live = factory;
  revision = 0u;
}

uint32_t Tune_Revision(void) { return revision; }

static int read_milli(const Param *p, const TuneValues *cfg)
{
  const char *field = (const char *)cfg + p->offset;
  return p->fractional ? (int)lroundf(*(const float *)field * 1000.0f)
                       : *(const int *)field * 1000;
}

static void write_milli(const Param *p, TuneValues *cfg, int value)
{
  char *field = (char *)cfg + p->offset;
  if (p->fractional) *(float *)field = value / 1000.0f;
  else *(int *)field = value / 1000;
}

static int parse_milli(const char *s, int *out)
{
  int negative = *s == '-';
  if (negative || *s == '+') s++;
  if (*s < '0' || *s > '9') return 0;
  int whole = 0;
  while (*s >= '0' && *s <= '9') {
    whole = whole * 10 + (*s++ - '0');
    if (whole > 32767) return 0;
  }
  int value = whole * 1000;
  if (*s == '.') {
    s++;
    if (*s < '0' || *s > '9') return 0;
    for (int place = 100; place && *s >= '0' && *s <= '9'; place /= 10)
      value += (*s++ - '0') * place;
  }
  *out = negative ? -value : value;
  return *s == '\0';
}

static void decimal(char out[20], int milli)
{
  int a = abs(milli);
  snprintf(out, 20, "%s%d.%03d", milli < 0 ? "-" : "", a/1000, a%1000);
}

static int consistent(const TuneValues *v)
{
  return v->entry_full_error_mm < v->entry_creep_error_mm &&
         v->entry_side_target_mm + v->entry_wall_band_mm < v->entry_wall_acquire_mm &&
         v->entry_wall_acquire_mm <= v->entry_wall_track_mm &&
         SAFE_STOP_MM + v->entry_coast_margin_mm < v->entry_side_target_mm &&
         v->entry_creep_counts <= v->route_entry_cruise &&
         v->entry_creep_counts <= v->entry_cruise_counts &&
         v->entry_min_cm < v->entry_max_cm;
}

static void describe(int index)
{
  const Param *p = &params[index];
  char current[20], initial[20], line[156];
  decimal(current, read_milli(p, &live));
  decimal(initial, read_milli(p, &factory));
  snprintf(line, sizeof line,
           "PARAM VALUE PV=%lu KEY=%s VAL=%s DEF=%s MIN=%d MAX=%d TYPE=%s\r\n",
           (unsigned long)revision, p->name, current, initial, p->low, p->high,
           p->fractional ? "DEC" : "INT");
  App_Puts(line);
}

void Tune_PollList(void)
{
  if (list_index < 0) return;
  if (revision != list_revision) {
    App_Puts("PARAM ERR LIST_CHANGED\r\n");
    list_index = -1;
  } else if (list_index < PARAM_COUNT) {
    describe(list_index++);
  } else {
    char line[64];
    snprintf(line, sizeof line, "PARAM END PV=%lu COUNT=%d\r\n",
             (unsigned long)revision, PARAM_COUNT);
    App_Puts(line);
    list_index = -1;
  }
}

void Tune_Command(char *line)
{
  char *state = NULL;
  char *verb = strtok_r(line + 1, " ", &state);
  char *key = strtok_r(NULL, " ", &state);
  char *value = strtok_r(NULL, " ", &state);
  char reply[156];
  if (!verb || strtok_r(NULL, " ", &state)) goto syntax;
  if (!strcmp(verb, "LIST") && !key) {
    list_index = 0;
    list_revision = revision;
    snprintf(reply, sizeof reply, "PARAM BEGIN PV=%lu COUNT=%d\r\n",
             (unsigned long)revision, PARAM_COUNT);
  } else if (!strcmp(verb, "RESET") && !key) {
    taskENTER_CRITICAL();
    live = factory;
    revision++;
    taskEXIT_CRITICAL();
    snprintf(reply, sizeof reply, "PARAM RESET PV=%lu\r\n", (unsigned long)revision);
  } else if (!strcmp(verb, "SAVE") && !key) {
    /* An erase stalls the whole scheduler for over a second, so it must never
       happen while the drive is active. */
    if (AppControl_Mode() != APP_MODE_IDLE) {
      App_Puts("PARAM ERR BUSY\r\n");
      return;
    }
    int rc = Tune_Store_Save(&live, revision);
    if (rc == 0) snprintf(reply, sizeof reply, "PARAM SAVE OK PV=%lu\r\n", (unsigned long)revision);
    else         snprintf(reply, sizeof reply, "PARAM SAVE ERR RC=%d\r\n", rc);
  } else if (!strcmp(verb, "LOAD") && !key) {
    static TuneValues stored;      /* the command task stack is small */
    if (!Tune_Store_Load(&stored)) { App_Puts("PARAM ERR EMPTY\r\n"); return; }
    if (!consistent(&stored))      { App_Puts("PARAM ERR RELATION\r\n"); return; }
    taskENTER_CRITICAL();
    live = stored;
    revision++;
    taskEXIT_CRITICAL();
    snprintf(reply, sizeof reply, "PARAM LOAD OK PV=%lu\r\n", (unsigned long)revision);
  } else {
    int set = !strcmp(verb, "SET");
    if (!key || (set ? !value : (value || strcmp(verb, "GET")))) goto syntax;
    int i = 0;
    while (i < PARAM_COUNT && strcmp(key, params[i].name)) i++;
    if (i == PARAM_COUNT) { App_Puts("PARAM ERR KEY\r\n"); return; }
    if (!set) { describe(i); return; }
    const Param *p = &params[i];
    int v;
    if (!parse_milli(value, &v) || (!p->fractional && v%1000)) {
      App_Puts("PARAM ERR NUMBER\r\n"); return;
    }
    if (v < p->low*1000 || v > p->high*1000) { App_Puts("PARAM ERR RANGE\r\n"); return; }
    TuneValues next = live;
    write_milli(p, &next, v);
    if (!consistent(&next)) { App_Puts("PARAM ERR RELATION\r\n"); return; }
    int old = read_milli(p, &live);
    if (old != v) {
      taskENTER_CRITICAL();
      live = next;
      revision++;
      taskEXIT_CRITICAL();
    }
    char before[20], after[20];
    decimal(before, old);
    decimal(after, v);
    snprintf(reply, sizeof reply, "PARAM SET PV=%lu KEY=%s OLD=%s NEW=%s\r\n",
             (unsigned long)revision, key, before, after);
  }
  App_Puts(reply);
  return;
syntax:
  App_Puts("PARAM ERR SYNTAX\r\n");
}
