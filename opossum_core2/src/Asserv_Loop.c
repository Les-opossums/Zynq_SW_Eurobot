#include "main.h"
#include "lib_asserv/Lib_Asserv.h"

#define CTRL_POS_ALPHA 0.5f // coefficient du filtre passe-bas pour la position de contrôle (entre 0 et 1, plus c'est petit plus le filtrage est fort)

uint16_t auto_printpos_delay = 100;

uint8_t Debug_Timing = 0;

uint16_t Asserv_Full_Count = 0;

CAN_Message CAN_Motor1;
CAN_Message CAN_Motor2;
CAN_Message CAN_Motor3;
CAN_Message CAN_Motor4;

uint8_t Channel_Motor1 = 0;
uint8_t Channel_Motor2 = 1;
uint8_t Channel_Motor3 = 2;
uint8_t Channel_Motor4 = 3;

int16_t Rotor_RPM1 = 0;
int16_t Rotor_RPM2 = 0;
int16_t Rotor_RPM3 = 0;
int16_t Rotor_RPM4 = 0;

float wheel_speed1 = 0;
float wheel_speed2 = 0;
float wheel_speed3 = 0;
float wheel_speed4 = 0;

uint8_t stop = 0;

Position position_lidar;
Position control_pos;

int Last_Timer_Asserv = 0;
int Asserv_State = 0;
int Asserv_Odo_Count = 0;

ESC_Command Consigne;
ESC_Command Wanted_Forced_Consigne;
ESC_Command old_Consigne;

Enable_Kalman en_kalman;

int Lidar_inconsistency_count = 0;

int kalman_initialized = 0;

float dx, dy, dt = 0;

int lidar_delay = 0; // délai de la dernière mesure lidar

int tampon;
int tampon3;
int tampon4 = 0;

// Définition des profils de bruit
float R_lidar[3];

float R_camera[3] = {OBS_NOISE_CAMERA_XY * OBS_NOISE_CAMERA_XY,
                     OBS_NOISE_CAMERA_XY * OBS_NOISE_CAMERA_XY, 
                     OBS_NOISE_CAMERA_THETA * OBS_NOISE_CAMERA_THETA};


extern volatile uint32_t new_cmd_from_core0;

void Init_Asserv(void) {
    Consigne.command1 = 0;
    Consigne.command2 = 0;
    Consigne.command3 = 0;
    Consigne.command4 = 0;

    Wanted_Forced_Consigne.command1 = 0;
    Wanted_Forced_Consigne.command2 = 0;
    Wanted_Forced_Consigne.command3 = 0;
    Wanted_Forced_Consigne.command4 = 0;

    old_Consigne.command1 = 0;
    old_Consigne.command2 = 0;
    old_Consigne.command3 = 0;
    old_Consigne.command4 = 0;

    R_lidar[0]  = OBS_NOISE_LIDAR_X * OBS_NOISE_LIDAR_X;
    R_lidar[1]  = OBS_NOISE_LIDAR_Y * OBS_NOISE_LIDAR_Y;
    R_lidar[2]  = OBS_NOISE_LIDAR_THETA * OBS_NOISE_LIDAR_THETA;

    en_kalman.enable_lidar_kalman = 1;
    en_kalman.enable_camera_kalman = 0;

    asserv_init();

    Last_Timer_Asserv = Timer_ms1;
}

// Variables de synchronisation
static int  last_odo_ms   = 0;
static int  odo_count     = 0;
static uint8_t slow_loop_due = 0;

void Asserv_Loop(void)
{
	// =================================================================
    // SECTION 1 — FAST LOOP : ODO + Kalman predict — cadencé à 1ms
    // =================================================================
    if ((Timer_ms1 - last_odo_ms) >= ODO_EVERY_MS) {
        last_odo_ms += ODO_EVERY_MS;

        // Capture atomique des vitesses moteurs
        // (protection contre écriture concurrente du CAN ISR)
        int s1, s2, s3, s4;
        uint32_t cpsr = mfcpsr();
        mtcpsr(cpsr | 0x80);          // __disable_irq() sur Cortex-A9
        s1 = speed_motor_1;
        s2 = speed_motor_2;
        s3 = speed_motor_3;
        s4 = speed_motor_4;
        mtcpsr(cpsr);                 // __enable_irq()

        odo_speed_step(s1, s2, s3, s4);
        odo_position_step(ODO_EVERY_MS * 0.001f);   // BUG CORRIGÉ

        kalman_predict(&kalman_current_state, ODO_EVERY_MS * 0.001f);
        kalman_update_odo(&kalman_current_state, &speed_robot_odom);
        kalman_fifo_push(&kalman_fifo, &kalman_current_state, &speed_robot_odom);

        odo_count++;
        if (odo_count >= ASSERV_EVERY) {
            odo_count    = 0;
            slow_loop_due = 1;
        }
    }

    // =================================================================
    // SECTION 2 — COMMANDS : fusion Kalman — dès que disponible
    //
    // Pas de section temporisée : s'exécute dans le même tour de boucle
    // que la détection du flag. Latence typique < 50μs après la SGI.
    // Aucun conflit avec le CAN ISR (variables disjointes).
    // =================================================================
    if (new_cmd_from_core0) {
        new_cmd_from_core0 = 0;
        Process_Shared_Memory_Commands();
    }

    // =================================================================
    // SECTION 3 — SLOW LOOP : contrôle position — cadencé à ASSERV_EVERY ms
    // =================================================================
    if (slow_loop_due) {
        slow_loop_due = 0;

        odo_speed_cumulate_step(ASSERV_EVERY);

        // Filtre passe-bas position de contrôle
        control_pos.x += CTRL_POS_ALPHA * (kalman_current_state.x[0] - control_pos.x);
        control_pos.y += CTRL_POS_ALPHA * (kalman_current_state.x[1] - control_pos.y);
        control_pos.t  = principal_angle(control_pos.t + CTRL_POS_ALPHA * principal_angle(kalman_current_state.x[2] - control_pos.t));

        // Mise à jour mémoire partagée
        local_data.kalman_out.x             = kalman_current_state.x[0];
        local_data.kalman_out.y             = kalman_current_state.x[1];
        local_data.kalman_out.t             = kalman_current_state.x[2];
        local_data.speed_robot              = speed_robot_asserv;
        local_data.cmd_speed_constrained    = speed_order_constrained;
        SEND_FIELD(&local_data, kalman_out);
        SEND_FIELD(&local_data, speed_robot);
        SEND_FIELD(&local_data, cmd_speed_constrained);

        // Contrôle mouvement
        motion_step();
        constrain_speed_order();
        constrain_acceleration_order(ASSERV_EVERY * ODO_EVERY_MS * 0.001f);

        // PID → commandes moteurs
        Asserv_PWM_calculator(&Consigne);

        // Traitement de la calibration asynchrone
        // step_wheel_ff_calibration();

        // Forced command override
        if (Wanted_Forced_Consigne.command1 != 0 || Wanted_Forced_Consigne.command2 != 0 ||
            Wanted_Forced_Consigne.command3 != 0 || Wanted_Forced_Consigne.command4 != 0) {
            Consigne = Wanted_Forced_Consigne;
        }

        // Normalisation 10000
        float c_max = Max_Quatre(Abs_Ternaire(Consigne.command1), Abs_Ternaire(Consigne.command2),
                                 Abs_Ternaire(Consigne.command3), Abs_Ternaire(Consigne.command4));
        if (c_max > 10000.0f) {
            float r = 10000.0f / c_max;
            Consigne.command1 *= r; Consigne.command2 *= r;
            Consigne.command3 *= r; Consigne.command4 *= r;
        }

        // Arrêt d'urgence
        if (AU_state) {
            asserv_off_step();
        } else {
            motor1_current_order = Consigne.command1;
            motor2_current_order = Consigne.command2;
            motor3_current_order = Consigne.command3;
            motor4_current_order = Consigne.command4;
        }
        CAN_transmit_motor(motor1_current_order, motor2_current_order,
                           motor3_current_order, motor4_current_order);
    }
}

void Set_Lidar_Noise_Cmd(Set_lidar_noise kalman_noise_lidar) {
    R_lidar[0]  = kalman_noise_lidar.process_noise_lidar_x * kalman_noise_lidar.process_noise_lidar_x;
    R_lidar[1]  = kalman_noise_lidar.process_noise_lidar_y * kalman_noise_lidar.process_noise_lidar_y;
    R_lidar[2]  = kalman_noise_lidar.process_noise_lidar_t * kalman_noise_lidar.process_noise_lidar_t;
}

void Set_Kalman_Enable_Cmd(Enable_Kalman enable_kalman) {
    en_kalman.enable_lidar_kalman = enable_kalman.enable_lidar_kalman;
    en_kalman.enable_camera_kalman = enable_kalman.enable_camera_kalman;
}


int count_lidar_cycle = 0; // nombre de cycles de la boucle d'asserv avant d'initialiser le kalman avec le lidar, pour laisser le temps au filtre de se stabiliser

void Process_Shared_Memory_Commands(void) {
    if (CHECK_FIELD(&local_data, cmd_position)) { motion_pos(local_data.cmd_position); }
    if (CHECK_FIELD(&local_data, cmd_speed)) { motion_speed(local_data.cmd_speed); }
    if (CHECK_FIELD(&local_data, cmd_abs_speed)) { motion_absolute_speed(local_data.cmd_abs_speed); }
    
    if (CHECK_FIELD(&local_data, asserv_mode)) {
        if (local_data.asserv_mode == 0) { motion_free(); } 
        else if (local_data.asserv_mode == 4) { motion_block(); }
        else if (local_data.asserv_mode == 5) { start_wheel_ff_calibration(); }
    }

    if (CHECK_FIELD(&local_data, set_pos)) { set_position(local_data.set_pos); }
    if (CHECK_FIELD(&local_data, vmax)) { set_Constraint_vitesse_xy_max(local_data.vmax); }
    if (CHECK_FIELD(&local_data, vtmax)){ set_Constraint_vt_max(local_data.vtmax); }
    if (CHECK_FIELD(&local_data, amax)) { set_Constraint_a_xy_max(local_data.amax); }
    if (CHECK_FIELD(&local_data, cmd_esc)) { Wanted_Forced_Consigne = local_data.cmd_esc; }
    if (CHECK_FIELD(&local_data, enable_kalman)){ Set_Kalman_Enable_Cmd(local_data.enable_kalman); }
    if (CHECK_FIELD(&local_data, odo_spacing)){ odo_set_spacing(local_data.odo_spacing); }
    if (CHECK_FIELD(&local_data, kalman_noise_lidar)){ Set_Lidar_Noise_Cmd(local_data.kalman_noise_lidar); }

    if (!kalman_initialized) {
        // Attendre quelques cycles lidar pour laisser le filtre se stabiliser
        if (CHECK_FIELD(&local_data, set_lidar)) {
            if (count_lidar_cycle < 10) {
                count_lidar_cycle++;
            } else {
                Position init_pos = {
                    local_data.set_lidar.lidar_position_x,
                    local_data.set_lidar.lidar_position_y,
                    local_data.set_lidar.lidar_position_t
                };
                kalman_init_with_lidar(&kalman_fifo, &init_pos);
                kalman_initialized = 1;
            }
        }
        return;
    }

    // --- Collecte de toutes les observations Kalman ---
    int earliest_index = -1;
    int earliest_delay = -1; // le plus grand delay = le plus loin dans le passé

    if (CHECK_FIELD(&local_data, set_lidar) && en_kalman.enable_lidar_kalman) {
        int idx = kalman_fifo_insert_lidar(&kalman_fifo, &local_data.set_lidar, R_lidar);
        if (idx >= 0 && local_data.set_lidar.delay > earliest_delay) {
            earliest_delay = local_data.set_lidar.delay;
            earliest_index = idx;
        }
    }

    Set_camera* cameras[3] = {&local_data.set_camera_1,
                               &local_data.set_camera_2,
                               &local_data.set_camera_3};
    uint8_t cam_fields[3] = {
        CHECK_FIELD(&local_data, set_camera_1),
        CHECK_FIELD(&local_data, set_camera_2),
        CHECK_FIELD(&local_data, set_camera_3)
    };

    for (int i = 0; i < 3; i++) {
        if (cam_fields[i] && en_kalman.enable_camera_kalman) {
            int idx = kalman_fifo_insert_camera(&kalman_fifo, cameras[i], i); // 0-indexé
            if (idx >= 0 && cameras[i]->delay > earliest_delay) {
                earliest_delay = cameras[i]->delay;
                earliest_index = idx;
            }
        }
    }

    // --- Une seule repropagate depuis le point le plus ancien ---
    if (earliest_index >= 0) {
        kalman_fifo_repropagate(&kalman_fifo, earliest_index,
                                ODO_EVERY_MS * 0.001f, R_lidar);
        kalman_current_state = kalman_fifo.buffer[
            (kalman_fifo.head - 1 + KALMAN_FIFO_LEN) % KALMAN_FIFO_LEN];
    }
}
