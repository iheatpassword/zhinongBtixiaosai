#ifndef CHASSIS_MOTION_H
#define CHASSIS_MOTION_H

/* Wheel-centre spacing from the reference chassis, in mm. */
#define CHASSIS_WHEELBASE_MM 590.0f
#define CHASSIS_TRACK_MM     200.0f
#define CHASSIS_RAD_PER_DEG  0.01745329252f

typedef struct {
  float forward_mm_s, right_mm_s, clockwise_rad_s;
} ChassisMotion;

typedef struct {
  float speed_mm_s, steer_deg;
} WheelMotion;

/* FL, FR, RL, RR. Reverse travel rolls backwards with the wheels facing
   forward. Lateral travel uses positive roll at either +90 or -90 degrees. */
void Chassis_Solve(ChassisMotion cmd, int roll_sign, float overtravel_deg,
                   WheelMotion wheels[4]);

#endif
