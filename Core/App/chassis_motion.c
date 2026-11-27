#include "chassis_motion.h"
#include <math.h>

void Chassis_Solve(ChassisMotion cmd, int roll_sign, float overtravel_deg,
                   WheelMotion wheels[4])
{
  const float half_length = CHASSIS_WHEELBASE_MM * 0.5f;
  const float half_track = CHASSIS_TRACK_MM * 0.5f;
  float x = roll_sign * cmd.forward_mm_s;
  float y = roll_sign * cmd.right_mm_s;
  float wz = roll_sign * cmd.clockwise_rad_s;
  /* Keep the route's chosen rolling direction. Near +/-90 degrees also
     constrain yaw so every wheel stays within the calibrated steering arc.
     The common yaw limit preserves a single rigid-body velocity. */
  float limit = fmaxf(fabsf(x) / half_track, fabsf(y) / half_length);
  float tangent = tanf(overtravel_deg * CHASSIS_RAD_PER_DEG);
  float steering_limit = (x + fabsf(y) * tangent) / (half_track + half_length * tangent);
  limit = fminf(limit, fmaxf(0.0f, steering_limit));
  wz = fminf(limit, fmaxf(-limit, wz));
  for (unsigned i = 0; i < 4; ++i) {
    float forward = x + ((i == 0 || i == 2) ? half_track : -half_track) * wz;
    float right = y + (i < 2 ? half_length : -half_length) * wz;
    wheels[i].speed_mm_s = roll_sign * sqrtf(forward * forward + right * right);
    wheels[i].steer_deg = (forward == 0 && right == 0) ? 0 :
                          atan2f(right, forward) / CHASSIS_RAD_PER_DEG;
  }
}
