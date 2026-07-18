#include "asserv.h"

#define INV_SQRT2 0.70710678118f
#define RPM_TO_RAD_PER_S (0.10471975512f)

Speed speed_robot_odom;
Speed speed_robot_asserv;
Position odometry_position;
float robot_wheel_distance = DEFAULT_ODO_SPACING;
float wheel_speed[4];
static float accumulated_wheel_speed[4];
static Speed accumulated_robot_speed;

static float motor_rpm_to_wheel_speed(int rpm, float sign)
{
    const float wheel_angular_speed = ((float)rpm * RPM_TO_RAD_PER_S) / ODO_MOTOR_REDUCTION;
    return sign * wheel_angular_speed * DEFAULT_WHEEL_RADIUS;
}

void odometry_init(void)
{
    speed_robot_odom = (Speed){0};
    speed_robot_asserv = (Speed){0};
    odometry_position = (Position){0};
    wheel_speed[0] = wheel_speed[1] = wheel_speed[2] = wheel_speed[3] = 0.0f;
    accumulated_wheel_speed[0] = accumulated_wheel_speed[1] = 0.0f;
    accumulated_wheel_speed[2] = accumulated_wheel_speed[3] = 0.0f;
    accumulated_robot_speed = (Speed){0};
    robot_wheel_distance = DEFAULT_ODO_SPACING;
}

void odometry_set_wheel_distance(float distance_m)
{
    if (distance_m > 0.0f)
    {
        robot_wheel_distance = distance_m;
    }
}

void odometry_update_from_motor_rpm(const int motor_rpm[4])
{
    const float w1 = wheel_speed[0] = motor_rpm_to_wheel_speed(motor_rpm[0], ODO_MOTOR_1_SIGN);
    const float w2 = wheel_speed[1] = motor_rpm_to_wheel_speed(motor_rpm[1], ODO_MOTOR_2_SIGN);
    const float w3 = wheel_speed[2] = motor_rpm_to_wheel_speed(motor_rpm[2], ODO_MOTOR_3_SIGN);
    const float w4 = wheel_speed[3] = motor_rpm_to_wheel_speed(motor_rpm[3], ODO_MOTOR_4_SIGN);

    speed_robot_odom.vx = INV_SQRT2 * (-w1 + w2 + w3 - w4) * 0.5f;
    speed_robot_odom.vy = INV_SQRT2 * (w1 + w2 - w3 - w4) * 0.5f;
    speed_robot_odom.vt = -(w1 + w2 + w3 + w4) / (4.0f * robot_wheel_distance);

    speed_robot_asserv = speed_robot_odom;
    for (int i = 0; i < 4; ++i) {
        accumulated_wheel_speed[i] += wheel_speed[i];
    }
    accumulated_robot_speed.vx += speed_robot_odom.vx;
    accumulated_robot_speed.vy += speed_robot_odom.vy;
    accumulated_robot_speed.vt += speed_robot_odom.vt;
}

void odometry_integrate(float dt_s)
{
    const float angle_mid = odometry_position.t + 0.5f * speed_robot_odom.vt * dt_s;
    const float c = cosf(angle_mid);
    const float s = sinf(angle_mid);
    odometry_position.x += (speed_robot_odom.vx * c - speed_robot_odom.vy * s) * dt_s;
    odometry_position.y += (speed_robot_odom.vx * s + speed_robot_odom.vy * c) * dt_s;
    odometry_position.t += speed_robot_odom.vt * dt_s;
    while (odometry_position.t > 3.14159265359f)
        odometry_position.t -= 6.28318530718f;
    while (odometry_position.t < -3.14159265359f)
        odometry_position.t += 6.28318530718f;
}

void odometry_finish_slow(uint8_t sample_count)
{
    if (sample_count == 0U)
        return;
    const float inv_count = 1.0f / (float)sample_count;
    for (int i = 0; i < 4; ++i)
    {
        wheel_speed[i] = accumulated_wheel_speed[i] * inv_count;
        accumulated_wheel_speed[i] = 0.0f;
    }
    speed_robot_asserv.vx = accumulated_robot_speed.vx * inv_count;
    speed_robot_asserv.vy = accumulated_robot_speed.vy * inv_count;
    speed_robot_asserv.vt = accumulated_robot_speed.vt * inv_count;
    accumulated_robot_speed = (Speed){0};
}

void odometry_set_position(const Position *position) { odometry_position = *position; }
