#ifndef ENTRY_WALL_H
#define ENTRY_WALL_H

#include "app_tasks.h"

/* Zero-initialize on every new run. All counters count distinct left samples. */
typedef struct {
  u8 acquired, near_samples, centered_samples, last_seq;
  u8 cruising;      /* 1 = running at ENTRY_CRUISE_COUNTS (latched) */
  u8 off_samples;   /* consecutive off-band samples, cruise->creep hysteresis */
} EntryWall;

typedef struct {
  s16 speed;      /* zero requests a hold */
  float angle;    /* positive is right */
} EntryWallCommand;

EntryWallCommand EntryWall_Update(EntryWall *wall, const AppInputs *in, u16 travel_cm);

#endif
