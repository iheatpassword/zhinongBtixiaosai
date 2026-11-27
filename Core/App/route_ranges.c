#include "route_ranges.h"
#include "app_config.h"
#include "laser_wire.h"
#include <stdlib.h>

RouteRange RouteRange_Side(const AppInputs *in, unsigned side, int allow_tail)
{
  RouteRange head = {side == LW_LEFT ? in->dL : in->dR, in->range_age[side],
                     in->sample_seq[side], 0};
  if (!in->laser_fresh || head.age > LW_MAX_AGE_MS ||
      !(in->laser_flags & lw_channel_bit(side))) head.mm = LW_MISSING;
  if (!allow_tail || !in->use_tail || !in->tail_fresh ||
      (head.mm > 0 && head.mm < BSP_DIST_OVERRANGE)) return head;
  u16 mm = side == LW_LEFT ? in->tail_left : in->tail_right;
  if (mm > 0 && mm < BSP_DIST_OVERRANGE && in->tail_age[side] <= LW_MAX_AGE_MS &&
      (in->tail_flags & lw_channel_bit(side))) {
    return (RouteRange){mm, in->tail_age[side], in->tail_seq[side], 1};
  }
  return head;
}

RidgeStatus RouteRange_Ridge(const AppInputs *in, float *error_mm)
{
  /* The centre pose in laser_calib.csv gives 206 / 216.3 mm. The old
     head-mounted rear beam had a different origin and keeps its own target. */
  int six = in->use_tail;
  u16 back = six ? in->tail_back : in->dB;
  u8 back_age = six ? in->tail_age[LW_BACK] : in->range_age[LW_BACK];
  int front_ok = in->laser_fresh && in->vF && in->range_age[LW_FRONT] <= LW_MAX_AGE_MS &&
                 in->dF >= 70 && in->dF <= 380;
  int back_ok = six ? in->tail_fresh && (in->tail_flags & lw_channel_bit(LW_BACK)) :
                      in->laser_fresh && in->vB;
  back_ok = back_ok && back_age <= LW_MAX_AGE_MS && back >= 40 && back <= 300;
  int midpoint = (RIDGE_FRONT_MM + RIDGE_TAIL_MM) / 2;
  int front_error = (int)in->dF - (six ? midpoint : (int)ENTRY_STOP_MM);
  int back_error = (six ? midpoint : (int)REAR_TARGET_MM) - (int)back;
  *error_mm = 0;
  if (front_ok && back_ok) {
    int skew = (int)in->range_age[LW_FRONT] - back_age;
    if (abs(skew) <= WALL_PAIR_MAX_SKEW_MS) {
      if (abs(front_error - back_error) > RIDGE_SUM_TOL_MM) return RIDGE_DISAGREE;
      *error_mm = 0.5f * (front_error + back_error);
      return six ? RIDGE_PAIR : RIDGE_VALID;
    }
    front_ok = skew < 0; /* Use the newer beam when comparing would mix poses. */
  }
  if (!front_ok && !back_ok) return RIDGE_MISSING;
  *error_mm = (float)(front_ok ? front_error : back_error);
  return RIDGE_VALID;
}
