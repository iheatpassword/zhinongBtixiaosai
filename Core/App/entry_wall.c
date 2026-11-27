#include "tune_params.h"
#include "entry_wall.h"
#include "app_config.h"
#include "laser_wire.h"
#include "safety.h"
#include <math.h>
#include <stdlib.h>

int EntryWall_Pair(const AppInputs *in, int left, u16 limit,
                   float *center_mm, float *forward_trim)
{
  unsigned side = left ? LW_LEFT : LW_RIGHT;
  u16 front = left ? in->dL : in->dR;
  u16 rear = left ? in->tail_left : in->tail_right;
  if (!in->use_tail || !in->tail_fresh || !in->laser_fresh ||
      !(in->laser_flags & lw_channel_bit(side)) || !(in->tail_flags & lw_channel_bit(side)) ||
      in->range_age[side] > LW_MAX_AGE_MS || in->tail_age[side] > LW_MAX_AGE_MS ||
      front == 0 || rear == 0 || front >= limit || rear >= limit) return 0;
  int delta = (int)front - rear;
  int skew = (int)in->range_age[side] - in->tail_age[side];
  if (abs(delta) > WALL_PAIR_MAX_DIFF_MM || abs(skew) > WALL_PAIR_MAX_SKEW_MS) return 0;
  *center_mm = 0.5f * (front + rear);
  /* Negative small steering points the wheels left. */
  float trim = (left ? -delta : delta) * app_tune->entry_yaw_trim_gain / WALL_SENSOR_BASELINE_MM;
  *forward_trim = fminf(app_tune->entry_yaw_trim_max_deg,
                        fmaxf(-app_tune->entry_yaw_trim_max_deg, trim));
  return 1;
}

static EntryWallCommand entry_guidance(EntryWall *wall, const AppInputs *in, u16 travel_cm,
                                  s16 search_speed, s16 cruise_speed)
{
  EntryWallCommand cmd = {0};
  /* The calibration probe must work even when no wall is in view: the early
     returns below (search, no votes, lost rail) never reach the trim block, so
     a probe set further down would silently stay zero and look like "the split
     has no effect". */
  cmd.yaw_trim_deg = app_tune->entry_debug_split_deg;
  /* Two thresholds: nothing but a genuine rail may start the follow (the rail
     always reads under 100 mm when first met), but once locked a wider return
     must not drop the creep and brake the car. */
  u16 limit = wall->acquired ? app_tune->entry_wall_track_mm : app_tune->entry_wall_acquire_mm;
  int visible = in->vL && in->range_age[LW_LEFT] <= LW_MAX_AGE_MS && in->dL < limit;
  if (!visible) {
    wall->near_samples = 0;
    wall->centered_samples = 0;
    wall->off_samples = 0;
    wall->cruising = 0;
    wall->range_rate = 0;
    wall->pair_trim_deg = 0;
    wall->speed_error_mm = 0;
    wall->integ_deg = 0;
    wall->speed_ms = 0;
    /* Keep last_speed: the nose-left beam drops out for single frames all the
       time (dL 65 -> 2000 -> 65), and zeroing the speed baseline here made every
       dropout restart the acceleration ramp, which is the stutter the car shows
       while it is correcting. */
    /* Inside the search window "the wall is gone" only means "not found yet",
       so drop the lock and keep looking.  Outside it the lock stays sticky:
       losing the wall there is a real fault and the caller must stop.  Without
       this a single spurious return (a ridge corner) locked the follow and the
       next lost sample parked the car for good. */
    if (travel_cm < app_tune->entry_search_limit_cm) wall->acquired = 0;
    /* The start box has no left fence, so "no left return" is normal there and
       must not veto the creep. Keep searching up to app_tune->entry_search_limit_cm; past
       that the caller stops the car and raises F2_MISSING_RANGE. */
    if (!wall->acquired && travel_cm < app_tune->entry_search_limit_cm)
      cmd.speed = search_speed;
    /* The search leg gets the same braking curve as the tracking leg.  Searching
       blind at cruise is fine while the corridor is clear, but the ridge still
       has to stop the car, and this branch returns before the ramp below. */
    {
      /* Rolled back: braking toward 0 mm rather than toward the stop line lets the
       car carry on into the ridge, which is where it has to end up. */
    float cap = sqrtf(2.0f * app_tune->entry_decel_mm_s2 * (float)in->dF) / MM_S_PER_COUNT;
      if (cap < (float)cmd.speed) cmd.speed = (s16)cap;
      if (cmd.speed < 0) cmd.speed = 0;
    }
    return cmd;
  }

  float center_mm = in->dL;
  float pair_trim = 0;
  int paired = EntryWall_Pair(in, 1, limit, &center_mm, &pair_trim);
  int error_mm = (int)center_mm - (int)app_tune->entry_side_target_mm;
  int centered = error_mm >= -app_tune->entry_wall_band_mm && error_mm <= app_tune->entry_wall_band_mm;
  u8 seq = in->sample_seq[LW_LEFT];
  if (!wall->near_samples || seq != wall->last_seq) {
    u32 sample_ms = in->ms - in->range_age[LW_LEFT];
    u32 dt = sample_ms - wall->sample_ms;
    if (wall->near_samples && dt && dt <= LW_MAX_AGE_MS) {
      float change = (int)in->dL - (int)wall->previous_mm;
      wall->range_rate += (1000.0f * change - dt * wall->range_rate) /
                          (app_tune->entry_range_filter_ms + dt);
    } else if (!wall->near_samples || dt > LW_MAX_AGE_MS) {
      wall->range_rate = 0;
    }
    wall->sample_ms = sample_ms;
    wall->previous_mm = in->dL;
    wall->last_seq = seq;
    if (wall->near_samples < app_tune->entry_acquire_votes) wall->near_samples++;
    if (centered) {
      wall->off_samples = 0;              /* back in band: reset the exit count */
      if (wall->centered_samples < app_tune->entry_center_votes) wall->centered_samples++;
    } else {
      wall->centered_samples = 0;
      if (wall->off_samples < app_tune->entry_wall_hyst_votes) wall->off_samples++;
    }
  }
  if (wall->near_samples < app_tune->entry_acquire_votes) {
    /* Only creep while the car has not started moving yet.  Once it is at
       cruise a single lost sample must not drop 5000 -> 450 (8 cm/s); that drop
       and the ramp back up is what looked like "it slows down to steer". */
    if (!wall->acquired && travel_cm < app_tune->entry_search_limit_cm &&
        in->wheel_speed_mm_s < 100.0f)
      cmd.speed = app_tune->entry_creep_counts;
    else if (wall->last_speed > 0)
      cmd.speed = wall->last_speed;
    return cmd;
  }
  wall->acquired = 1;

  /* Speed hysteresis: five centred samples start the cruise, but a run of
     off-band samples is required to end it.  Without this, one sample at
     65 mm dropped 1200 -> 450 and the next sample put it straight back. */
  if (wall->cruising == 0u) {
    if (wall->centered_samples >= app_tune->entry_center_votes) wall->cruising = 1u;
  } else if (wall->off_samples >= app_tune->entry_wall_hyst_votes) {
    wall->cruising = 0u;
    wall->centered_samples = 0;
  }

  float lead_s = (in->range_age[LW_LEFT] + app_tune->entry_steer_lead_ms) / 1000.0f;
  float steer_error = error_mm + wall->range_rate * lead_s;
  cmd.angle = -app_tune->entry_wall_kp * steer_error;
  if (cmd.angle > app_tune->entry_wall_max_deg) cmd.angle = app_tune->entry_wall_max_deg;
  if (cmd.angle < -app_tune->entry_wall_max_deg) cmd.angle = -app_tune->entry_wall_max_deg;

  /* Dead-zone feed-forward.  The linkage swallows the first ~3.5 deg of every
     command, so a 10 mm error (1 deg) does nothing at all and the loop never
     closes.  Jumping straight past the slack makes the response linear again,
     which is what removes the limit cycle the P/I pair falls into. */
  if (app_tune->entry_steer_deadzone_deg > 0.0f &&
      fabsf((float)error_mm) > app_tune->entry_steer_deadzone_mm) {
    cmd.angle -= (error_mm > 0) ? app_tune->entry_steer_deadzone_deg
                                : -app_tune->entry_steer_deadzone_deg;
    if (cmd.angle >  app_tune->entry_wall_max_deg) cmd.angle =  app_tune->entry_wall_max_deg;
    if (cmd.angle < -app_tune->entry_wall_max_deg) cmd.angle = -app_tune->entry_wall_max_deg;
  }

  /* Proportional action alone cannot beat the linkage slack: a 10 mm error asks
     for 1 deg, the wheel eats it, and the offset never closes.  The integrator
     keeps asking until the wheel actually moves; it is clamped and cleared when
     the wall is lost so it cannot wind up. */
  {
    float dt_s = (float)APP_CTRL_MS / 1000.0f;
    wall->integ_deg += app_tune->entry_steer_ki * (float)error_mm * dt_s;
    if (wall->integ_deg >  app_tune->entry_steer_integ_max_deg)
      wall->integ_deg =  app_tune->entry_steer_integ_max_deg;
    if (wall->integ_deg < -app_tune->entry_steer_integ_max_deg)
      wall->integ_deg = -app_tune->entry_steer_integ_max_deg;
    cmd.angle -= wall->integ_deg;
    if (cmd.angle >  app_tune->entry_wall_max_deg) cmd.angle =  app_tune->entry_wall_max_deg;
    if (cmd.angle < -app_tune->entry_wall_max_deg) cmd.angle = -app_tune->entry_wall_max_deg;
  }
  /* Calibration probe: a known constant added to the common angle. */
  cmd.angle += app_tune->entry_debug_angle_deg;
  if (cmd.angle >  app_tune->entry_wall_max_deg) cmd.angle =  app_tune->entry_wall_max_deg;
  if (cmd.angle < -app_tune->entry_wall_max_deg) cmd.angle = -app_tune->entry_wall_max_deg;

  /* Allow for both steering lag and the time needed to shed wheel speed.
     The range rate is lateral; wheel speed only sets the look-ahead time. */
  float lookahead_s = lead_s + in->wheel_speed_mm_s / (2.0f * app_tune->entry_decel_mm_s2);
  float predicted_mm = center_mm + wall->range_rate * lookahead_s;
  /* The speed decision uses the lateral error alone.  Folding the predicted
     lateral position in as well made the look-ahead grow with speed
     (v/(2*decel)), so the faster the car went the larger the apparent error and
     the harder the envelope throttled: a self-limiting loop that pinned the
     entry near 20 cm/s however high the cruise was set (T median 1600 of 5000,
     V peak 900 mm/s).  Stopping is the job of the dF/1200 ramp instead. */
  float error = fabsf((float)error_mm);

  /* The speed decision runs on a slow average of the error.  The steering loop
     deliberately works the raw error and swings a couple of centimetres; if the
     speed followed that swing it turned into a surge-pause cycle whose varying
     speed also changed the steering loop gain. */
  {
    u32 now_ms = in->ms;
    float dt_s  = (wall->speed_ms == 0u) ? 0.02f : (float)(now_ms - wall->speed_ms) / 1000.0f;
    float tau_s = (float)app_tune->entry_speed_filter_ms / 1000.0f;
    float alpha;
    if (dt_s > 0.2f) dt_s = 0.2f;
    wall->speed_ms = now_ms;
    alpha = (tau_s <= 0.001f) ? 1.0f : dt_s / (tau_s + dt_s);
    wall->speed_error_mm += (error - wall->speed_error_mm) * alpha;
  }

  /* Constant forward speed, reference-route style.  The lateral position is
     corrected by steering alone - their vy is clamped to 45 mm/s and vx never
     changes.  Any lateral-error term here (envelope, creep floor, gate) makes
     the speed surge and sag with the wall loop, which is exactly the
     accelerate/decelerate the car was showing: the only things allowed to shape
     the speed are the braking curve below and the per-cycle accel limit.
     wall->speed_error_mm is still tracked for diagnostics. */
  cmd.speed = cruise_speed;
  /* Final approach: brake along the physically correct curve for a constant
     deceleration, v = sqrt(2*a*d), instead of the reference route's linear
     dF/1200.  Linear starts slowing far too early - at 1.2 m it was already
     below cruise and the last metre dragged - while sqrt keeps full speed
     until the braking distance and then sheds it at exactly `decel`. */
  {
    /* Rolled back: braking toward 0 mm rather than toward the stop line lets the
       car carry on into the ridge, which is where it has to end up. */
    float cap = sqrtf(2.0f * app_tune->entry_decel_mm_s2 * (float)in->dF) / MM_S_PER_COUNT;
    if (cap < (float)cmd.speed) cmd.speed = (s16)cap;
    if (cmd.speed < 0) cmd.speed = 0;
  }
  if (cmd.speed > wall->last_speed + app_tune->entry_accel_counts)
    cmd.speed = wall->last_speed + app_tune->entry_accel_counts;
  /* The gyro loop in longitudinal() holds heading even without a wall pair. */
  if (paired) {
    float dt_ms = in->sample_dt_s * 1000.0f;
    float alpha = dt_ms / (dt_ms + app_tune->entry_yaw_trim_filter_ms);
    wall->pair_trim_deg += alpha * (pair_trim - wall->pair_trim_deg);
  } else {
    wall->pair_trim_deg = 0;
  }
  /* Split = pair_trim (static body angle) + yaw-rate loop + calibration probe.
     The yaw loop is a PD on the gyro heading that produces a rate command and
     then inverts omega = 2*v*delta/L to a split, so the authority no longer
     depends on speed.  Measured polarity: a POSITIVE split drives the body to a
     NEGATIVE yaw rate (+8 deg -> -0.8 deg/s), so the rate term is added, not
     subtracted - getting it backwards turns the loop into positive feedback. */
  {
    float wz = app_tune->entry_yaw_wz_kp * in->yaw_deg
             + app_tune->entry_yaw_wz_kd * in->yaw_rate_dps;
    float trim;
    if (wz >  app_tune->entry_yaw_wz_max) wz =  app_tune->entry_yaw_wz_max;
    if (wz < -app_tune->entry_yaw_wz_max) wz = -app_tune->entry_yaw_wz_max;
    if (in->wheel_speed_mm_s > 100.0f) {
      /* deg/s * mm / (mm/s) is already degrees; no rad/deg conversion needed. */
      trim = wz * app_tune->entry_yaw_wb_mm / (2.0f * in->wheel_speed_mm_s);
    } else {
      trim = 0.0f;              /* no speed, no yaw authority: stay parallel */
    }
    trim += wall->pair_trim_deg + app_tune->entry_debug_split_deg;
    if (trim >  app_tune->entry_yaw_trim_max_deg) trim =  app_tune->entry_yaw_trim_max_deg;
    if (trim < -app_tune->entry_yaw_trim_max_deg) trim = -app_tune->entry_yaw_trim_max_deg;
    cmd.yaw_trim_deg = trim;
  }

  float creep_mm_s = app_tune->entry_creep_counts * (3.14159265f * WHEEL_DIAMETER_MM * 1000.0f /
                                                     (ENCODER_CPR * APP_CTRL_MS));
  cmd.coast = in->wheel_speed_mm_s > creep_mm_s + app_tune->entry_coast_extra_mm_s &&
              fminf(fminf(in->dL, center_mm), predicted_mm) <= SAFE_STOP_MM + app_tune->entry_coast_margin_mm;
  if (cmd.coast && cmd.angle < 0) cmd.angle = 0;

  /* Slew-limit the common angle.  The reference route commands a lateral
     VELOCITY and lets the inverse kinematics turn it into at most ~2.6 deg of
     crab, with the PID output slope-limited, so the body just eases sideways.
     Ours computes an angle straight from the error, so when the error changes
     sign the servo is ordered to swing the whole way in one 20 ms tick - that
     is the stiff, snatching feel.  Ramping the angle costs a few tens of
     milliseconds of response and removes the snap. */
  {
    float step = app_tune->entry_steer_slew_dps * ((float)APP_CTRL_MS / 1000.0f);
    if (cmd.angle > wall->last_angle + step) cmd.angle = wall->last_angle + step;
    if (cmd.angle < wall->last_angle - step) cmd.angle = wall->last_angle - step;
    wall->last_angle = cmd.angle;
  }

  wall->last_speed = cmd.coast ? 0 : cmd.speed;
  return cmd;
}

/* Applied during search as well as tracking. A ridge end can appear first
   beside either axle while the other three side beams still see open space. */
EntryWallCommand EntryWall_Update(EntryWall *wall, const AppInputs *in, u16 travel_cm,
                                  s16 search_speed, s16 cruise_speed)
{
  EntryWallCommand cmd = entry_guidance(wall, in, travel_cm, search_speed, cruise_speed);
  unsigned nearest = BSP_DIST_INVALID;
  for (unsigned side = LW_LEFT; side <= LW_RIGHT; ++side) {
    unsigned front = side == LW_LEFT ? in->dL : in->dR;
    unsigned rear = side == LW_LEFT ? in->tail_left : in->tail_right;
    if (in->laser_fresh && (in->laser_flags & lw_channel_bit(side)) &&
        in->range_age[side] <= LW_MAX_AGE_MS && front < nearest) nearest = front;
    if (in->use_tail && in->tail_fresh && (in->tail_flags & lw_channel_bit(side)) &&
        in->tail_age[side] <= LW_MAX_AGE_MS && rear < nearest) nearest = rear;
  }
  float margin = SAFE_STOP_MM + app_tune->entry_coast_margin_mm;
  float fraction = fminf(1.0f, fmaxf(0.0f, (nearest - margin) / 20.0f));
  s16 cap = (s16)(app_tune->entry_creep_counts +
                  fraction * (cruise_speed - app_tune->entry_creep_counts));
  if (cmd.speed > cap) cmd.speed = cap;
  if (nearest <= margin) {
    cmd.coast = 1;
    cmd.speed = 0;
  }
  if (wall->last_speed > cmd.speed) wall->last_speed = cmd.speed;
  return cmd;
}
