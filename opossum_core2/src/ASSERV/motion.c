#include "asserv.h"
#include "../../../opossum_common/IPC_MANAGER/IPC_manager.h"  /* ajuste la profondeur si besoin */
#include "xil_printf.h"
#include <math.h>

/* ================================================================== *
 * Prototypes internes (necessaires : ces fonctions s'appellent entre
 * elles avant leur definition plus bas dans le fichier -- sans ces
 * prototypes, un appel a une fonction non encore vue est traite comme
 * un retour int implicite, ce qui corromprait les floats retournes).
 * ================================================================== */
static void  motion_off(void);
static void  motion_pos(Position pos);
static void  motion_speed(Speed speed);
static void  motion_absolute_speed(Speed speed);
static void  update_shared_motion_done(motion_status_t new_state);
static void  asserv_off_step(void);
static void  asserv_free_step(void);
static void  pos_asserv_step(const Position *current_position);
static void  speed_asserv_step(void);
static void  absolute_speed_asserv_step(const Position *current_position);
static void  speed_asserv_break_step(void);
static float radial_speed_calculation(float distance);
static float angular_speed_calculation(float angle);

/* ================================================================== *
 * Variables
 * ================================================================== */
#define CTRL_POS_ALPHA 1.0f   /* filtre passe-bas sur la position de controle
                                * -- a 1.0f il est actuellement inerte (pas-
                                * -a-travers pur) ; laisse pour tuning futur */

asserv_mode_t   asserv_mode;
motion_status_t motion_done;

static Position control_pos;   /* position filtree utilisee par le controleur */
static float     blocked_time; /* reserve pour ASSERV_BLOCK_TIME_LIMIT, pas encore utilise */
static Position  Wanted_Pos;
static Speed     Wanted_Speed;
static int       emergency_break_requested;

/* Structure locale utilisee pour la remontee IPC de motion_done. */
static struct {
    uint32_t motion_done;
} local_data;

/* ================================================================== *
 * IPC
 * ================================================================== */
static void update_shared_motion_done(motion_status_t new_state)
{
    motion_done = new_state;
    local_data.motion_done = (uint32_t)new_state;
    SEND_FIELD(&local_data, motion_done);
}

/* ================================================================== *
 * API publique
 * ================================================================== */
void motion_init(void)
{
    asserv_mode = ASSERV_MODE_OFF;
    motion_done = MOTION_SUCCESS;
    update_shared_motion_done(MOTION_SUCCESS);
    blocked_time = 0;

    Wanted_Pos.x = 0; Wanted_Pos.y = 0; Wanted_Pos.t = 0;
    Wanted_Speed.vx = 0; Wanted_Speed.vy = 0; Wanted_Speed.vt = 0;
    control_pos.x = 0.0f; control_pos.y = 0.0f; control_pos.t = 0.0f;

    emergency_break_requested = 0;
}

void motion_set_position(const Position *target)      { motion_pos(*target); }
void motion_set_speed(const Speed *speed)              { motion_speed(*speed); }
void motion_set_absolute_speed(const Speed *speed)     { motion_absolute_speed(*speed); }

void motion_block(void)
{
    asserv_mode = ASSERV_MODE_BREAK;
    speed_order_constrained.vx = 0.0f;
    speed_order_constrained.vy = 0.0f;
    speed_order_constrained.vt = 0.0f;
    motion_done = MOTION_BRAKED;
    update_shared_motion_done(MOTION_BRAKED);
}

static void motion_off(void)
{
    asserv_mode = ASSERV_MODE_OFF;
    if (motion_done == MOTION_MOVING) {
        motion_done = MOTION_SUCCESS;
        update_shared_motion_done(MOTION_SUCCESS);
    }
}

void motion_free(void)
{
    asserv_mode = ASSERV_MODE_FREE;
    if (motion_done == MOTION_MOVING) {
        update_shared_motion_done(MOTION_SUCCESS);
        motion_done = MOTION_SUCCESS;
    }
}

static void motion_pos(Position pos)
{
    if (asserv_mode == ASSERV_MODE_BREAK) {
        pid_vitesse_reset();
        speed_order_constrained.vx = 0.0f;
        speed_order_constrained.vy = 0.0f;
        speed_order_constrained.vt = 0.0f;
    } else {
        pid_vitesse_reset();
        speed_order_constrained.vx = speed_robot_asserv.vx;
        speed_order_constrained.vy = speed_robot_asserv.vy;
        speed_order_constrained.vt = speed_robot_asserv.vt;
    }

    Wanted_Pos = pos;
    emergency_break_requested = 0;

    asserv_mode = ASSERV_MODE_POS;
    motion_done = MOTION_MOVING;
    update_shared_motion_done(MOTION_MOVING);
}

static void motion_speed(Speed speed)
{
    if (asserv_mode == ASSERV_MODE_OFF || asserv_mode == ASSERV_MODE_FREE) {
        pid_vitesse_reset();
        speed_order_constrained.vx = speed_robot_asserv.vx;
        speed_order_constrained.vy = speed_robot_asserv.vy;
        speed_order_constrained.vt = speed_robot_asserv.vt;
    } else if (asserv_mode == ASSERV_MODE_BREAK) {
        pid_vitesse_reset();
        speed_order_constrained.vx = 0.0f;
        speed_order_constrained.vy = 0.0f;
        speed_order_constrained.vt = 0.0f;
    }

    emergency_break_requested = 0;
    Wanted_Speed = speed;

    asserv_mode = ASSERV_MODE_SPEED;
    motion_done = MOTION_MOVING;
    update_shared_motion_done(MOTION_MOVING);
}

static void motion_absolute_speed(Speed speed)
{
    Wanted_Speed = speed;
    asserv_mode = ASSERV_MODE_ABSOLUTE_SPEED;
    motion_done = MOTION_MOVING;
    update_shared_motion_done(MOTION_MOVING);
}

void motion_step(const Position *current_position)
{
    switch (asserv_mode) {
        case ASSERV_MODE_OFF:            asserv_off_step(); break;
        case ASSERV_MODE_FREE:           asserv_free_step(); break;
        case ASSERV_MODE_POS:            pos_asserv_step(current_position); break;
        case ASSERV_MODE_SPEED:          speed_asserv_step(); break;
        case ASSERV_MODE_BREAK:          speed_asserv_break_step(); break;
        case ASSERV_MODE_ABSOLUTE_SPEED: absolute_speed_asserv_step(current_position); break;
        default:                         asserv_off_step(); break;
    }
}

/* ================================================================== *
 * Pas d'asservissement par mode
 * ================================================================== */
static void asserv_off_step(void)
{
    speed_order.vx = 0;
    speed_order.vy = 0;
    speed_order.vt = 0;
    Pid_Speed_En = 0;
    emergency_break_requested = 0;
    motion_off();
}

static void asserv_free_step(void)
{
    speed_order.vx = 0;
    speed_order.vy = 0;
    speed_order.vt = 0;
    Pid_Speed_En = 1;

    if ((fabsf(speed_robot_asserv.vx) < DEFAULT_SPEED_LIN_STOP) &&
        (fabsf(speed_robot_asserv.vy) < DEFAULT_SPEED_LIN_STOP) &&
        (fabsf(speed_robot_asserv.vt) < DEFAULT_SPEED_ROT_STOP)) {
        motion_off();
    }
}

static void speed_asserv_break_step(void)
{
    if (fabsf(speed_robot_asserv.vx) > DEFAULT_SPEED_LIN_STOP ||
        fabsf(speed_robot_asserv.vy) > DEFAULT_SPEED_LIN_STOP ||
        fabsf(speed_robot_asserv.vt) > DEFAULT_SPEED_ROT_STOP) {
        emergency_break_requested = 1;
        speed_order.vx = 0;
        speed_order.vy = 0;
        speed_order.vt = 0;
        Pid_Speed_En = 1;
    } else {
        emergency_break_requested = 0;

        speed_order.vx = 0;
        speed_order.vy = 0;
        speed_order.vt = 0;
        pid_vitesse_reset();

        motion_done = MOTION_BRAKED;
        update_shared_motion_done(MOTION_BRAKED);

        motion_free();

        xil_printf("Break,done\n");
    }
}

static void pos_asserv_step(const Position *current_position)
{
    float x_o = Wanted_Pos.x;
    float y_o = Wanted_Pos.y;
    float t_o = Wanted_Pos.t;

    /* Filtre passe-bas (voir CTRL_POS_ALPHA) sur la position estimee fournie
     * par asserv_loop.c -- plus de dependance directe a kalman_current_state ici. */
    control_pos.x += CTRL_POS_ALPHA * (current_position->x - control_pos.x);
    control_pos.y += CTRL_POS_ALPHA * (current_position->y - control_pos.y);
    control_pos.t  = principal_angle(control_pos.t + CTRL_POS_ALPHA * principal_angle(current_position->t - control_pos.t));

    float x = control_pos.x;
    float y = control_pos.y;
    float t = control_pos.t;

    float rdx = x_o - x;
    float rdy = y_o - y;
    float d   = sqrtf(rdx * rdx + rdy * rdy);
    float dt  = principal_angle(t_o - t);

    float cos_t = cosf(t);
    float sin_t = sinf(t);
    float angle = atan2f(rdy, rdx);

    float v_profile       = radial_speed_calculation(d);
    float v_braking_limit = sqrtf(2.0f * robot_a_max * d);
    float v_cmd            = fminf(v_profile, fminf(v_braking_limit, robot_v_max));

    float vx_world = v_cmd * cosf(angle);
    float vy_world = v_cmd * sinf(angle);

    speed_order.vx =  vx_world * cos_t + vy_world * sin_t;
    speed_order.vy = -vx_world * sin_t + vy_world * cos_t;
    speed_order.vt = angular_speed_calculation(dt);

    Pid_Speed_En = 1;

    static uint8_t in_stop_zone = 0;
    static int     stop_zone_timer_ms = 0;

    if (d < DEFAULT_STOP_DISTANCE && fabsf(dt) < DEFAULT_STOP_ANGLE) {
        if (!in_stop_zone) {
            in_stop_zone = 1;
            stop_zone_timer_ms = 0;
        }
        stop_zone_timer_ms++;
    } else {
        in_stop_zone = 0;
        stop_zone_timer_ms = 0;
    }

    if (in_stop_zone) {
        float v_now = sqrtf(speed_robot_asserv.vx * speed_robot_asserv.vx +
                             speed_robot_asserv.vy * speed_robot_asserv.vy);
        if (v_now < DEFAULT_SPEED_LIN_STOP || stop_zone_timer_ms > 20) {
            in_stop_zone = 0;

            motion_done = MOTION_SUCCESS;
            update_shared_motion_done(MOTION_SUCCESS);

            motion_free();
            xil_printf("Pos,done\n");
        }
    }
}

static float radial_speed_calculation(float distance)
{
    float v_raw           = sqrtf(2.0f * robot_a_max * distance * 0.85f);
    float v_min           = 0.03f;
    float v_profile       = (v_raw > v_min) ? v_raw : v_min;
    float v_braking_limit = sqrtf(2.0f * robot_a_max * distance);
    return fminf(v_profile, fminf(v_braking_limit, robot_v_max));
}

static float angular_speed_calculation(float angle)
{
    float fabs_angle = fabsf(angle);
    int sign = (angle < 0) ? -1 : 1;
    float v = sqrtf(2.0f * robot_at_max * fabs_angle * 0.85f);
    return sign * fminf(v, robot_vt_max);
}

static void speed_asserv_step(void)
{
    speed_order.vx = Wanted_Speed.vx;
    speed_order.vy = Wanted_Speed.vy;
    speed_order.vt = Wanted_Speed.vt;
    Pid_Speed_En = 1;
}

static void absolute_speed_asserv_step(const Position *current_position)
{
    float cos_t = cosf(current_position->t);
    float sin_t = sinf(current_position->t);
    speed_order.vx =  Wanted_Speed.vx * cos_t + Wanted_Speed.vy * sin_t;
    speed_order.vy = -Wanted_Speed.vx * sin_t + Wanted_Speed.vy * cos_t;
    speed_order.vt = Wanted_Speed.vt;
    Pid_Speed_En = 1;
}

int Get_asserv_done(void)
{
    return (motion_done != MOTION_MOVING);
}