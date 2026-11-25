#include "entry_wall.h"
#include "app_config.h"
#include "laser_wire.h"

EntryWallCommand EntryWall_Update(EntryWall *wall, const AppInputs *in, u16 travel_cm)
{
  EntryWallCommand cmd = {0};
  /* Two thresholds: nothing but a genuine rail may start the follow (the rail
     always reads under 100 mm when first met), but once locked a wider return
     must not drop the creep and brake the car. */
  u16 limit = wall->acquired ? ENTRY_WALL_TRACK_MM : ENTRY_WALL_ACQUIRE_MM;
  int visible = in->vL && in->dL < limit;
  if (!visible) {
    wall->near_samples = 0;
    wall->centered_samples = 0;
    wall->off_samples = 0;
    /* The start box has no left fence, so "no left return" is normal there and
       must not veto the creep. Keep searching up to ENTRY_SEARCH_LIMIT_CM; past
       that the caller stops the car and raises F2_MISSING_RANGE. */
    if (!wall->acquired && travel_cm < ENTRY_SEARCH_LIMIT_CM)
      cmd.speed = ROUTE_CREEP_COUNTS;
    return cmd;
  }

  int error_mm = (int)in->dL - (int)SIDE_CENTER_MM;
  int centered = error_mm >= -ENTRY_WALL_BAND_MM && error_mm <= ENTRY_WALL_BAND_MM;
  u8 seq = in->sample_seq[LW_LEFT];
  if (!wall->near_samples || seq != wall->last_seq) {
    wall->last_seq = seq;
    if (wall->near_samples < 3) wall->near_samples++;
    if (centered) {
      wall->off_samples = 0;              /* back in band: reset the exit count */
      if (wall->centered_samples < 5) wall->centered_samples++;
    } else {
      wall->centered_samples = 0;
      if (wall->off_samples < ENTRY_WALL_HYST_VOTES) wall->off_samples++;
    }
  }
  if (wall->near_samples < 3) {
    if (!wall->acquired && travel_cm < ENTRY_SEARCH_LIMIT_CM) cmd.speed = ROUTE_CREEP_COUNTS;
    return cmd;
  }
  wall->acquired = 1;

  /* Speed hysteresis: five centred samples start the cruise, but a run of
     off-band samples is required to end it.  Without this, one sample at
     65 mm dropped 1200 -> 450 and the next sample put it straight back. */
  if (wall->cruising == 0u) {
    if (wall->centered_samples >= 5) wall->cruising = 1u;
  } else if (wall->off_samples >= ENTRY_WALL_HYST_VOTES) {
    wall->cruising = 0u;
    wall->centered_samples = 0;
  }

  cmd.angle = -ENTRY_WALL_KP * error_mm;
  if (cmd.angle > ENTRY_WALL_MAX_DEG) cmd.angle = ENTRY_WALL_MAX_DEG;
  if (cmd.angle < -ENTRY_WALL_MAX_DEG) cmd.angle = -ENTRY_WALL_MAX_DEG;
  cmd.speed = wall->cruising ? ENTRY_CRUISE_COUNTS : ROUTE_CREEP_COUNTS;
  return cmd;
}
