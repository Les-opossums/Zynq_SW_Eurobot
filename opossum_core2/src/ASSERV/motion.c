#include "asserv.h"

#define MOTION_POSITION_KP_XY 2.0f
#define MOTION_POSITION_KP_T 3.0f
#define PI_F 3.14159265359f

static motion_mode_t mode;
static Position position_target;
static Speed speed_target;

static float wrap_angle(float angle)
{
    while (angle > PI_F)
        angle -= 2.0f * PI_F;
    while (angle < -PI_F)
        angle += 2.0f * PI_F;
    return angle;
}

void motion_init(void)
{
    mode = MOTION_FREE;
    position_target = (Position){0};
    speed_target = (Speed){0};
    Pid_Speed_En = 0;
}

void motion_set_position(const Position *target)
{
    position_target = *target;
    mode = MOTION_POSITION;
    Pid_Speed_En = 1;
}
void motion_set_speed(const Speed *speed)
{
    speed_target = *speed;
    mode = MOTION_SPEED;
    Pid_Speed_En = 1;
}
void motion_set_absolute_speed(const Speed *speed)
{
    speed_target = *speed;
    mode = MOTION_ABSOLUTE_SPEED;
    Pid_Speed_En = 1;
}
void motion_free(void)
{
    mode = MOTION_FREE;
    speed_order = (Speed){0};
    Pid_Speed_En = 1;
}
void motion_block(void)
{
    mode = MOTION_BLOCKED;
    speed_order = (Speed){0};
    Pid_Speed_En = 1;
}

void motion_step(const Position *current_position)
{
    if (mode == MOTION_POSITION)
    {
        const float dx = position_target.x - current_position->x;
        const float dy = position_target.y - current_position->y;
        const float c = cosf(current_position->t);
        const float s = sinf(current_position->t);
        const float distance = sqrtf(dx * dx + dy * dy);
        const float max_linear = fminf(robot_v_max, sqrtf(2.0f * robot_a_max * distance * 0.85f));
        const float world_vx = distance > DEFAULT_STOP_DISTANCE ? max_linear * dx / distance : 0.0f;
        const float world_vy = distance > DEFAULT_STOP_DISTANCE ? max_linear * dy / distance : 0.0f;
        speed_order.vx = c * world_vx + s * world_vy;
        speed_order.vy = -s * world_vx + c * world_vy;
        const float angle_error = wrap_angle(position_target.t - current_position->t);
        speed_order.vt = copysignf(
            fminf(robot_vt_max, sqrtf(2.0f * robot_at_max * fabsf(angle_error) * 0.85f)), angle_error);
    }
    else if (mode == MOTION_ABSOLUTE_SPEED)
    {
        const float c = cosf(current_position->t);
        const float s = sinf(current_position->t);
        speed_order.vx = c * speed_target.vx + s * speed_target.vy;
        speed_order.vy = -s * speed_target.vx + c * speed_target.vy;
        speed_order.vt = speed_target.vt;
    }
    else if (mode == MOTION_SPEED)
    {
        speed_order = speed_target;
    }
    else
    {
        speed_order = (Speed){0};
    }
}
