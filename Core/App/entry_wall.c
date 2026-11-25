#include "entry_wall.h"
#include "app_config.h"
#include "laser_wire.h"

EntryWallCommand EntryWall_Update(EntryWall *wall, const AppInputs *in, u16 travel_cm)
{
  EntryWallCommand cmd = {0};
  int visible = in->vL && in->dL < ENTRY_WALL_LIMIT_MM;
  if (!visible) {
    wall->near_samples = 0;
    wall->centered_samples = 0;
    if (!wall->acquired && in->dL != BSP_DIST_INVALID && travel_cm < ENTRY_SEARCH_LIMIT_CM)
      cmd.speed = ROUTE_CREEP_COUNTS;
    return cmd;
  }

  int error_mm = (int)in->dL - (int)SIDE_CENTER_MM;
  int centered = error_mm >= -ENTRY_WALL_BAND_MM && error_mm <= ENTRY_WALL_BAND_MM;
  if (!centered) wall->centered_samples = 0;
  u8 seq = in->sample_seq[LW_LEFT];
  if (!wall->near_samples || seq != wall->last_seq) {
    wall->last_seq = seq;
    if (wall->near_samples < 3) wall->near_samples++;
    if (centered && wall->centered_samples < 5) wall->centered_samples++;
  }
  if (wall->near_samples < 3) {
    if (!wall->acquired && travel_cm < ENTRY_SEARCH_LIMIT_CM) cmd.speed = ROUTE_CREEP_COUNTS;
    return cmd;
  }
  wall->acquired = 1;
  cmd.angle = -ENTRY_WALL_KP * error_mm;
  if (cmd.angle > ENTRY_WALL_MAX_DEG) cmd.angle = ENTRY_WALL_MAX_DEG;
  if (cmd.angle < -ENTRY_WALL_MAX_DEG) cmd.angle = -ENTRY_WALL_MAX_DEG;
  cmd.speed = wall->centered_samples >= 5 ? ENTRY_CRUISE_COUNTS : ROUTE_CREEP_COUNTS;
  return cmd;
}
