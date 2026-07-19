#include "asserv_commands.h"
#include "../APP_COM/interpreteur.h"
#include "../IO_config.h"
#include "../IPC_MANAGER/IPC_manager.h"
#include "../TIMER_MANAGER/timer_manager.h"
#include "../IO_MANAGER/DRIVER_ETH/driver_eth_io.h"
#include "ETH_protocol.h"
#include "robot_messages.h"
#include "common_type.h"
#include <stdio.h>
#include <math.h>

#define MOVE_SEQ_FIFO_SIZE 16

/*
 * Structure locale (CORE0) miroir des champs de ipc_shared_data_t utilises
 * ici -- memes noms de champs que la structure partagee, pour que
 * SEND_FIELD/CHECK_FIELD fonctionnent directement dessus.
 */
typedef struct {
    Position        cmd_position;
    Speed           cmd_speed;
    Speed           cmd_abs_speed;
    int             asserv_mode;
    Position        set_pos;
    float           vmax;
    float           vtmax;
    float           amax;
    float           odo_spacing;
    ESC_Command     cmd_esc;
    Enable_Kalman   enable_kalman;
    Set_lidar_noise kalman_noise_lidar;
    Set_lidar       set_lidar;
    Set_camera      set_camera_1;
    Set_camera      set_camera_2;
    Set_camera      set_camera_3;

    Position        kalman_out;
    Speed           speed_robot;
    Speed           cmd_speed_constrained;
    uint32_t        motion_done;
} core0_asserv_data_t;

static core0_asserv_data_t local_data;

typedef struct {
    Position positions[MOVE_SEQ_FIFO_SIZE];
    int head;
    int tail;
    int count;
} MoveSeqFifo;

static MoveSeqFifo move_seq_fifo = {0};
static int         move_seq_active = 0;

static int move_seq_push(Position pos) {
    if (move_seq_fifo.count >= MOVE_SEQ_FIFO_SIZE) return 0;
    move_seq_fifo.positions[move_seq_fifo.tail] = pos;
    move_seq_fifo.tail = (move_seq_fifo.tail + 1) % MOVE_SEQ_FIFO_SIZE;
    move_seq_fifo.count++;
    return 1;
}

static int move_seq_pop(Position *pos) {
    if (move_seq_fifo.count == 0) return 0;
    *pos = move_seq_fifo.positions[move_seq_fifo.head];
    move_seq_fifo.head = (move_seq_fifo.head + 1) % MOVE_SEQ_FIFO_SIZE;
    move_seq_fifo.count--;
    return 1;
}

static void move_seq_clear(void) {
    move_seq_fifo.head  = 0;
    move_seq_fifo.tail  = 0;
    move_seq_fifo.count = 0;
    move_seq_active     = 0;
}

/* ================================================================== *
 * Commandes de mouvement
 * ================================================================== */

uint8_t Move_Cmd(void) {
    if (AU_state) {
        printf("INVALID COMMAND : AU\n");
        return 0;
    }
    move_seq_clear();
    float temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    local_data.cmd_position.x = temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    local_data.cmd_position.y = temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    local_data.cmd_position.t = temp_val;
    SEND_FIELD(&local_data, cmd_position);
    return 0;
}

uint8_t Move_Seq_Cmd(void) {
    if (AU_state) { printf("INVALID COMMAND : AU\n"); return 0; }

    Position pos;
    float temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    pos.x = temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    pos.y = temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    pos.t = temp_val;

    if (!move_seq_push(pos)) {
        printf("MOVESEX,FIFO_FULL\n");
        return 1;
    }
    return 0;
}

void Move_Seq_Loop(void) {
    CHECK_FIELD(&local_data, motion_done);

    if (move_seq_active && local_data.motion_done) {
        move_seq_active = 0;
    }

    if (!move_seq_active && move_seq_fifo.count > 0) {
        Position pos;
        if (move_seq_pop(&pos)) {
            local_data.cmd_position = pos;
            SEND_FIELD(&local_data, cmd_position);
            move_seq_active = 1;
        }
    }
}

uint8_t Speed_Cmd(void) {
    if (AU_state) {
        printf("INVALID COMMAND : AU\n");
        return 0;
    }
    float temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    local_data.cmd_speed.vx = temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    local_data.cmd_speed.vy = temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    local_data.cmd_speed.vt = temp_val;
    SEND_FIELD(&local_data, cmd_speed);
    return 0;
}

uint8_t Absolute_Speed_Cmd(void) {
    if (AU_state) {
        printf("INVALID COMMAND : AU\n");
        return 0;
    }
    float temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    local_data.cmd_abs_speed.vx = temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    local_data.cmd_abs_speed.vy = temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    local_data.cmd_abs_speed.vt = temp_val;
    SEND_FIELD(&local_data, cmd_abs_speed);
    return 0;
}

uint8_t FREE_Cmd(void) {
    local_data.asserv_mode = 0;
    SEND_FIELD(&local_data, asserv_mode);
    return 0;
}

uint8_t BLOCK_Cmd(void) {
    if (AU_state) {
        printf("INVALID COMMAND : AU\n");
        return 0;
    }
    move_seq_clear();
    local_data.asserv_mode = 4;
    SEND_FIELD(&local_data, asserv_mode);
    return 0;
}

uint8_t Start_Wheel_FF_Calibration_Cmd(void) {
    if (AU_state) {
        printf("INVALID COMMAND : AU\n");
        return 0;
    }
    local_data.asserv_mode = 5;
    SEND_FIELD(&local_data, asserv_mode);
    return 0;
}

/* ================================================================== *
 * Lecture d'etat
 * ================================================================== */

uint8_t Asserv_Done_Cmd(void) {
    CHECK_FIELD(&local_data, motion_done);
    printf("%lu\n", (unsigned long)local_data.motion_done);
    return 0;
}

uint8_t Get_Pos_Cmd(void) {
    if (CHECK_FIELD(&local_data, kalman_out)) {
        printf("GETPOS %.4f %.4f %.4f\n",
               (double)local_data.kalman_out.x,
               (double)local_data.kalman_out.y,
               (double)local_data.kalman_out.t);
    } else {
        printf("GETPOS ERRROR: Position not valid\n");
    }
    return 0;
}

uint8_t Get_Odo_Cmd(void) {
    int status1 = CHECK_FIELD(&local_data, kalman_out);
    int status2 = CHECK_FIELD(&local_data, speed_robot);

    if (!status1 || !status2) {
        printf("GETODO ERROR: Position or speed not valid\n");
        return 0;
    }

    printf("ODO %.4f %.4f %.4f %.4f %.4f %.4f\n",
           (double)local_data.kalman_out.x, (double)local_data.kalman_out.y, (double)local_data.kalman_out.t,
           (double)local_data.speed_robot.vx, (double)local_data.speed_robot.vy, (double)local_data.speed_robot.vt);
    return 0;
}

/* ================================================================== *
 * Configuration / recalage
 * ================================================================== */

uint8_t SET_Cmd(void) {
    float temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    local_data.set_pos.x = temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    local_data.set_pos.y = temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    local_data.set_pos.t = temp_val;
    SEND_FIELD(&local_data, set_pos);
    return 0;
}

uint8_t SET0_Cmd(void) {
    local_data.set_pos.x = 0.0f;
    local_data.set_pos.y = 0.0f;
    local_data.set_pos.t = 0.0f;
    SEND_FIELD(&local_data, set_pos);
    return 0;
}

uint8_t Set_Lidar_Cmd(void) {
    float temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    local_data.set_lidar.lidar_position_x = temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    local_data.set_lidar.lidar_position_y = temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    local_data.set_lidar.lidar_position_t = temp_val;
    uint32_t tmp_delay;
    if (Get_Param_u32(&tmp_delay)) return PARAM_ERROR_CODE;
    local_data.set_lidar.delay = tmp_delay;
    SEND_FIELD(&local_data, set_lidar);
    return 0;
}

uint8_t Set_Lidar_Noise_Cmd(void) {
    float temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    local_data.kalman_noise_lidar.process_noise_lidar_x = temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    local_data.kalman_noise_lidar.process_noise_lidar_y = temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    local_data.kalman_noise_lidar.process_noise_lidar_t = temp_val;
    SEND_FIELD(&local_data, kalman_noise_lidar);
    return 0;
}

static uint8_t set_camera_cmd_impl(Set_camera *dst) {
    float temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    dst->camera_position_x = temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    dst->camera_position_y = temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    dst->camera_position_t = temp_val;
    uint32_t tmp_delay;
    if (Get_Param_u32(&tmp_delay)) return PARAM_ERROR_CODE;
    dst->delay = tmp_delay;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    dst->noise_x = temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    dst->noise_y = temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    dst->noise_t = temp_val;
    return 0;
}

uint8_t Set_Camera_1_Cmd(void) {
    uint8_t err = set_camera_cmd_impl(&local_data.set_camera_1);
    if (err) return err;
    SEND_FIELD(&local_data, set_camera_1);
    return 0;
}
uint8_t Set_Camera_2_Cmd(void) {
    uint8_t err = set_camera_cmd_impl(&local_data.set_camera_2);
    if (err) return err;
    SEND_FIELD(&local_data, set_camera_2);
    return 0;
}
uint8_t Set_Camera_3_Cmd(void) {
    uint8_t err = set_camera_cmd_impl(&local_data.set_camera_3);
    if (err) return err;
    SEND_FIELD(&local_data, set_camera_3);
    return 0;
}

uint8_t VMAX_Cmd(void) {
    float temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    local_data.vmax = temp_val;
    SEND_FIELD(&local_data, vmax);
    return 0;
}

uint8_t VTMAX_Cmd(void) {
    float temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    local_data.vtmax = temp_val;
    SEND_FIELD(&local_data, vtmax);
    return 0;
}

uint8_t AMAX_Cmd(void) {
    float temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    local_data.amax = temp_val;
    SEND_FIELD(&local_data, amax);
    return 0;
}

uint8_t PWM_Func(void) {
    float temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    local_data.cmd_esc.command1 = temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    local_data.cmd_esc.command2 = temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    local_data.cmd_esc.command3 = temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    local_data.cmd_esc.command4 = temp_val;
    SEND_FIELD(&local_data, cmd_esc);
    return 0;
}

uint8_t Enable_Kalman_Cmd(void) {
    uint32_t temp_val;
    if (Get_Param_u32(&temp_val)) return PARAM_ERROR_CODE;
    local_data.enable_kalman.enable_lidar_kalman = (int)temp_val;
    if (Get_Param_u32(&temp_val)) return PARAM_ERROR_CODE;
    local_data.enable_kalman.enable_camera_kalman = (int)temp_val;
    SEND_FIELD(&local_data, enable_kalman);
    return 0;
}

uint8_t Set_Odo_Spacing_Cmd(void) {
    float temp_val;
    if (Get_Param_Float(&temp_val)) return PARAM_ERROR_CODE;
    local_data.odo_spacing = temp_val;
    SEND_FIELD(&local_data, odo_spacing);
    return 0;
}

/* ================================================================== *
 * Telemetrie automatique (UART + Ethernet)
 * ================================================================== */

static int      auto_printpos_en = 1;
static uint32_t auto_printpos_delay_uart = 100;
static uint32_t Last_Timer_print_pos_uart = 0;
static uint32_t auto_printpos_delay_eth = 1;
static uint32_t Last_Timer_print_pos_eth = 0;

uint8_t Activate_Position_Sending_Func(void) {
    uint32_t state;
    if (Get_Param_u32(&state)) return PARAM_ERROR_CODE;
    auto_printpos_en = (state != 0);
    Last_Timer_print_pos_uart = Timer_ms1;
    Last_Timer_print_pos_eth  = Timer_ms1;

    uint32_t delay;
    if (!Get_Param_u32(&delay)) {
        auto_printpos_delay_uart = delay;
    }
    return 0;
}

static void compute_robot_state(eth_payload_robot_state_t *rs, float *speed_linear, float *speed_direction) {
    *speed_linear    = sqrtf(local_data.speed_robot.vx * local_data.speed_robot.vx +
                              local_data.speed_robot.vy * local_data.speed_robot.vy);
    *speed_direction = atan2f(local_data.speed_robot.vy, local_data.speed_robot.vx);

    rs->timestamp_ms   = (uint32_t)Timer_ms1;
    rs->x               = local_data.kalman_out.x;
    rs->y               = local_data.kalman_out.y;
    rs->theta           = local_data.kalman_out.t;
    rs->speed_linear    = *speed_linear;
    rs->speed_direction = *speed_direction;
    rs->speed_angular   = local_data.speed_robot.vt;
    rs->motion_done     = (uint8_t)local_data.motion_done;
}

void Print_Position_loop(void) {
    if (!auto_printpos_en) return;
    if (!CHECK_FIELD(&local_data, kalman_out) || !CHECK_FIELD(&local_data, speed_robot)) return;

    if ((Timer_ms1 - Last_Timer_print_pos_uart) >= auto_printpos_delay_uart) {
        Last_Timer_print_pos_uart = Timer_ms1;

        float speed_linear    = sqrtf(local_data.speed_robot.vx * local_data.speed_robot.vx +
                                       local_data.speed_robot.vy * local_data.speed_robot.vy);
        float speed_direction = atan2f(local_data.speed_robot.vy, local_data.speed_robot.vx);
        
        printf("ROBOTDATA %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f\n",
               (double)local_data.kalman_out.x, (double)local_data.kalman_out.y, (double)local_data.kalman_out.t,
               (double)speed_linear, (double)speed_direction, (double)local_data.speed_robot.vt);
    }

    if ((Timer_ms1 - Last_Timer_print_pos_eth) >= auto_printpos_delay_eth) {
        Last_Timer_print_pos_eth = Timer_ms1;

        eth_payload_robot_state_t rs;
        float speed_linear, speed_direction;
        compute_robot_state(&rs, &speed_linear, &speed_direction);
        eth_send_frame(ETH_MSG_ROBOT_STATE, &rs, sizeof(rs));
    }
}

/* ================================================================== *
 * SPEEDTEST - Reglage PID vitesse
 * ================================================================== */

static int      speed_timed_active     = 0;
static uint32_t speed_timed_end_time   = 0;
static uint32_t speed_timed_last_print = 0;
static uint32_t speed_timed_print_per  = 10;

uint8_t Speed_Timed_Cmd(void) {
    if (AU_state) {
        printf("INVALID COMMAND : AU\n");
        return 0;
    }

    float vx, vy, vt;
    uint32_t duration_ms;

    if (Get_Param_Float(&vx))        return PARAM_ERROR_CODE;
    if (Get_Param_Float(&vy))        return PARAM_ERROR_CODE;
    if (Get_Param_Float(&vt))        return PARAM_ERROR_CODE;
    if (Get_Param_u32(&duration_ms)) return PARAM_ERROR_CODE;

    uint32_t print_period_ms = 10;
    Get_Param_u32(&print_period_ms);

    local_data.cmd_speed.vx = vx;
    local_data.cmd_speed.vy = vy;
    local_data.cmd_speed.vt = vt;
    SEND_FIELD(&local_data, cmd_speed);

    speed_timed_end_time   = Timer_ms1 + duration_ms;
    speed_timed_print_per  = (print_period_ms > 0) ? print_period_ms : 10;
    speed_timed_last_print = Timer_ms1;
    speed_timed_active     = 1;

    printf("SPEEDTEST START vx=%.3f vy=%.3f vt=%.3f dur=%lums print=%lums\n",
           (double)vx, (double)vy, (double)vt,
           (unsigned long)duration_ms, (unsigned long)speed_timed_print_per);

    return 0;
}

void Speed_Timed_Loop(void) {
    if (!speed_timed_active) return;

    uint32_t now = Timer_ms1;

    CHECK_FIELD(&local_data, cmd_speed_constrained);
    CHECK_FIELD(&local_data, speed_robot);

    if ((int32_t)(now - speed_timed_end_time) >= 0) {
        speed_timed_active = 0;

        local_data.asserv_mode = 0;
        SEND_FIELD(&local_data, asserv_mode);

        printf("SPEEDTEST DONE\n");
        return;
    }

    if ((now - speed_timed_last_print) >= speed_timed_print_per) {
        speed_timed_last_print = now;
        // Print detaille desactive par defaut (voir version originale
        // commentee) -- decommenter au besoin pour le reglage PID.
    }
}