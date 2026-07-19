
#include "asserv.h"
#include "asserv_math.h"
#include <math.h>

Acceleration Accel_Max;

Speed speed_order;
Speed speed_order_constrained;
Speed speed_order_constrained_1;

float Speed_Order_1, Speed_Order_2, Speed_Order_3, Speed_Order_4;

float robot_v_max;
float robot_vt_max;

float robot_a_max;
float robot_at_max;

void speed_constrainer_init(void)
{
    robot_v_max = DEFAULT_CONSTRAINT_V_MAX;
    robot_vt_max = DEFAULT_CONSTRAINT_VT_MAX;

    speed_order.vx = 0;
    speed_order.vy = 0;
    speed_order.vt = 0;

    speed_order_constrained.vx = 0;
    speed_order_constrained.vy = 0;
    speed_order_constrained.vt = 0;

    speed_order_constrained_1.vx = 0;
    speed_order_constrained_1.vy = 0;
    speed_order_constrained_1.vt = 0;

    Speed_Order_1 = 0;
    Speed_Order_2 = 0;
    Speed_Order_3 = 0;
    Speed_Order_4 = 0;
}

void acceleration_constrainer_init(void)
{
    robot_a_max = DEFAULT_CONSTRAINT_A_MAX;
    robot_at_max = DEFAULT_CONSTRAINT_AT_MAX;
}

void constrain_speed_order(void)
{
    speed_order_constrained_1.vx = speed_order.vx;
    speed_order_constrained_1.vy = speed_order.vy;
    speed_order_constrained_1.vt = speed_order.vt;

    float v_linear = sqrtf(speed_order_constrained_1.vx * speed_order_constrained_1.vx +
                           speed_order_constrained_1.vy * speed_order_constrained_1.vy);

    if (v_linear > robot_v_max)
    {
        float scale = robot_v_max / v_linear;
        speed_order_constrained_1.vx *= scale;
        speed_order_constrained_1.vy *= scale;
    }
    speed_order_constrained_1.vt = limit_float(speed_order_constrained_1.vt, -robot_vt_max, robot_vt_max);
}

void constrain_acceleration_order(float period)
{
    float delta_v_max = robot_a_max * period;
    float delta_vt_max = robot_at_max * period;

    // process old speed constrained (be aware of the rotation)
    float delta_angle = speed_order_constrained.vt * period;
    float cos_angle = cosf(delta_angle);
    float sin_angle = sinf(delta_angle);
    float previous_vx_order = speed_order_constrained.vx * cos_angle + speed_order_constrained.vy * sin_angle;
    float previous_vy_order = speed_order_constrained.vy * cos_angle - speed_order_constrained.vx * sin_angle;

    // process acceleration steps
    float delta_vx = speed_order_constrained_1.vx - previous_vx_order;
    float delta_vy = speed_order_constrained_1.vy - previous_vy_order;

    float dv_linear = sqrtf(delta_vx * delta_vx + delta_vy * delta_vy);
    if (dv_linear > delta_v_max)
    {
        float scale = delta_v_max / dv_linear;
        speed_order_constrained.vx = previous_vx_order + delta_vx * scale;
        speed_order_constrained.vy = previous_vy_order + delta_vy * scale;
    }
    else
    {
        speed_order_constrained.vx = speed_order_constrained_1.vx;
        speed_order_constrained.vy = speed_order_constrained_1.vy;
    }
    speed_order_constrained.vt =
        limit_float(speed_order_constrained_1.vt, speed_order_constrained.vt - delta_vt_max,
                    speed_order_constrained.vt + delta_vt_max);

    float inv_sqrt2 = 0.70710678f; // 1 / sqrt(2)

    Speed_Order_1 = inv_sqrt2 * (-speed_order_constrained.vx + speed_order_constrained.vy) -
                    robot_wheel_distance * speed_order_constrained.vt; // avant-droite
    Speed_Order_2 = inv_sqrt2 * (speed_order_constrained.vx + speed_order_constrained.vy) -
                    robot_wheel_distance * speed_order_constrained.vt; // arrière-droite
    Speed_Order_3 = inv_sqrt2 * (speed_order_constrained.vx - speed_order_constrained.vy) -
                    robot_wheel_distance * speed_order_constrained.vt; // arrière-gauche
    Speed_Order_4 = inv_sqrt2 * (-speed_order_constrained.vx - speed_order_constrained.vy) -
                    robot_wheel_distance * speed_order_constrained.vt; // avant-gauche
}

void set_Constraint_vitesse_xy_max(float v_max)
{
    if (v_max > 0.0f)
    {
        if (v_max <= DEFAULT_CONSTRAINT_V_MAX)
        {
            robot_v_max = v_max;
        }
        else
        {
            robot_v_max = DEFAULT_CONSTRAINT_V_MAX;
        }
    }
    else
    {
        // Si on reçoit 0, -1, ou une valeur aberrante, on met la valeur par défaut
        robot_v_max = DEFAULT_CONSTRAINT_V_MAX;
    }
}

void set_Constraint_vt_max(float vt_max)
{
    if (vt_max > 0.0f)
    {
        if (vt_max <= DEFAULT_CONSTRAINT_VT_MAX)
        {
            robot_vt_max = vt_max;
        }
        else
        {
            robot_vt_max = DEFAULT_CONSTRAINT_VT_MAX;
        }
    }
    else
    {
        robot_vt_max = DEFAULT_CONSTRAINT_VT_MAX;
    }
}
void set_Constraint_a_xy_max(float a_max)
{
    if (a_max > 0.0f)
    {
        if (a_max <= DEFAULT_CONSTRAINT_A_MAX)
        {
            robot_a_max = a_max;
        }
        else
        {
            robot_a_max = DEFAULT_CONSTRAINT_A_MAX;
        }
    }
    else
    {
        robot_a_max = DEFAULT_CONSTRAINT_A_MAX;
    }
}

void set_Constraint_at_max(float at_max)
{
    if (at_max > 0.0f)
    {
        if (at_max <= DEFAULT_CONSTRAINT_AT_MAX)
        {
            robot_at_max = at_max;
        }
        else
        {
            robot_at_max = DEFAULT_CONSTRAINT_AT_MAX;
        }
    }
    else
    {
        robot_at_max = DEFAULT_CONSTRAINT_AT_MAX;
    }
}
