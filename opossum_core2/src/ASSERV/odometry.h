#ifndef ASSERV_ODOMETRY_H
#define ASSERV_ODOMETRY_H

#include "../../../opossum_common/common_type.h"

/* The signs and reduction ratio are kept here because they depend on the
 * mechanical assembly, not on the controller implementation. */
#ifndef ODO_MOTOR_REDUCTION
#define ODO_MOTOR_REDUCTION 36.0f
#endif

#ifndef ODO_MOTOR_1_SIGN
#define ODO_MOTOR_1_SIGN 1.0f
#define ODO_MOTOR_2_SIGN 1.0f
#define ODO_MOTOR_3_SIGN 1.0f
#define ODO_MOTOR_4_SIGN 1.0f
#endif

extern Speed speed_robot_odom;
extern Speed speed_robot_asserv;
extern Position odometry_position;
extern float robot_wheel_distance;
extern float wheel_speed[4];

void odometry_init(void);
void odometry_set_wheel_distance(float distance_m);
void odometry_update_from_motor_rpm(const int motor_rpm[4]);
void odometry_integrate(float dt_s);
void odometry_finish_slow(uint8_t sample_count);
void odometry_set_position(const Position *position);

#endif /* ASSERV_ODOMETRY_H */
