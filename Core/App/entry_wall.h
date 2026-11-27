#ifndef ENTRY_WALL_H
#define ENTRY_WALL_H

#include "app_tasks.h"

/* Zero-initialize on every new run. All counters count distinct left samples. */
typedef struct {
  u8 acquired, near_samples, centered_samples, last_seq;
  u8 cruising;      /* centred votes permit cruise, subject to clearance */
  u8 off_samples;   /* consecutive off-band samples, cruise->creep hysteresis */
  u16 previous_mm;
  u32 sample_ms;
  float pair_trim_deg; /* filtered wall-angle correction for forward entry */
  float range_rate; /* positive when moving away from the left wall, mm/s */
  float speed_error_mm; /* slow average of the same error, used for speed */
  float integ_deg;      /* integral action that walks the wheel past linkage slack */
  u32 speed_ms;
  s16 last_speed;
  float last_angle;   /* previous common angle, for the slew limit */
} EntryWall;

typedef struct {
  s16 speed;      /* zero requests a hold */
  float angle;    /* positive at small forward angles moves left */
  float yaw_trim_deg; /* front +trim, rear -trim: rotates the body parallel */
  u8 coast;       /* remove wheel drive while retaining corrective steering */
} EntryWallCommand;

EntryWallCommand EntryWall_Update(EntryWall *wall, const AppInputs *in, u16 travel_cm,
                                  s16 search_speed, s16 cruise_speed);

/* Pair geometry is used only when both beams see the same nearby outer wall. */
int EntryWall_Pair(const AppInputs *in, int left, u16 limit,
                   float *center_mm, float *forward_trim);

#endif
