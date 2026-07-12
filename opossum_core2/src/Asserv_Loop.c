#include "main.h"
#include "lib_asserv/Lib_Asserv.h"

// #define TIMING_MEASURE          // Commenter pour désactiver

#ifdef TIMING_MEASURE

typedef struct {
    int32_t  min_us;
    int32_t  max_us;
    int64_t  sum_us;
    uint32_t count;
    const char* name;
} TimingStats;

#define TIMING_STATS_INIT(label) { .min_us = INT32_MAX, .max_us = INT32_MIN, \
                                   .sum_us = 0, .count = 0, .name = label }

static TimingStats ts_fast_total       = TIMING_STATS_INIT("Fast loop total ");
static TimingStats ts_fast_imu         = TIMING_STATS_INIT("  BNO085_Poll   ");
static TimingStats ts_fast_kalman      = TIMING_STATS_INIT("  Kalman predict");
// Repropagate : deux stats distinctes
//   ts_reprop_tick  = durée d'UN tick (10-20 slots) — mesurée à chaque fast loop active
//   ts_reprop_total = durée totale d'UN job complet (du start au dernier tick)
static TimingStats ts_reprop_tick      = TIMING_STATS_INIT("  Reprop tick   ");
static TimingStats ts_reprop_total     = TIMING_STATS_INIT("  Reprop job    ");
static TimingStats ts_slow_total       = TIMING_STATS_INIT("Slow loop total ");
static TimingStats ts_slow_motion      = TIMING_STATS_INIT("  motion_step   ");
static TimingStats ts_slow_pid_can     = TIMING_STATS_INIT("  PID+CAN       ");

static void ts_update(TimingStats* s, int32_t elapsed_us) {
    if (elapsed_us < 0) return;   // débordement du timer 32 bits → ignorer
    if (elapsed_us < s->min_us) s->min_us = elapsed_us;
    if (elapsed_us > s->max_us) s->max_us = elapsed_us;
    s->sum_us += elapsed_us;
    s->count++;
}

static void ts_print_one(TimingStats* s) {
    if (s->count == 0) {
        xil_printf("%s : (no data)\r\n", s->name);
    } else {
        int32_t avg = (int32_t)(s->sum_us / s->count);
        xil_printf("%s : %4ld / %4ld / %4ld  [n=%lu]\r\n",
                   s->name, (long)s->min_us, (long)avg,
                   (long)s->max_us, (unsigned long)s->count);
    }
    s->min_us = INT32_MAX; s->max_us = INT32_MIN;
    s->sum_us = 0; s->count = 0;
}

static void ts_print_all(void) {
    xil_printf("\r\n=== TIMING (us) === min / avg / max / count ===\r\n");
    ts_print_one(&ts_fast_total);
    ts_print_one(&ts_fast_imu);
    ts_print_one(&ts_fast_kalman);
    ts_print_one(&ts_reprop_tick);   // coût d'un tick dans la fast loop
    ts_print_one(&ts_reprop_total);  // durée totale d'un job (wall clock)
    ts_print_one(&ts_slow_total);
    ts_print_one(&ts_slow_motion);
    ts_print_one(&ts_slow_pid_can);
    xil_printf("  Budget fast : 1000 us  |  Budget slow : %d us\r\n\r\n",
               ASSERV_EVERY * ODO_EVERY_MS * 1000);
}

#define T_START(id)        int32_t _t_##id = Timer_us1
#define T_STOP(id, stats)  ts_update(&(stats), Timer_us1 - _t_##id)

// Macro spéciale pour le job de repropagate :
// stocke le timestamp de début du job dans une variable statique,
// puis mesure la durée totale quand le job se termine.
// Usage : REPROP_JOB_START() au lancement, REPROP_JOB_END() à la fin.
static int32_t _t_reprop_job_start = 0;
#define REPROP_JOB_START()   (_t_reprop_job_start = Timer_us1)
#define REPROP_JOB_END()     ts_update(&ts_reprop_total, Timer_us1 - _t_reprop_job_start)

#else
#define T_START(id)
#define T_STOP(id, stats)
#define REPROP_JOB_START()
#define REPROP_JOB_END()
static void ts_print_all(void) {}
#endif


#define CTRL_POS_ALPHA 0.5f

volatile int imu_needs_reinit = 0;
float imu_yaw_offset = 0.0f;

extern BNO085_Dev imu;

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

Position position_lidar;

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
int lidar_delay = 0;

int tampon;
int tampon3;
int tampon4 = 0;

float R_lidar[3];

float R_camera[3] = {OBS_NOISE_CAMERA_XY    * OBS_NOISE_CAMERA_XY,
                     OBS_NOISE_CAMERA_XY    * OBS_NOISE_CAMERA_XY,
                     OBS_NOISE_CAMERA_THETA * OBS_NOISE_CAMERA_THETA};

extern volatile uint32_t new_cmd_from_core0;

static uint8_t need_kalman_hard_reset = 0;
static int     last_odo_ms   = 0;
static int     odo_count     = 0;
static uint8_t slow_loop_due = 0;


void Init_Asserv(void) {
    Consigne.command1 = 0; Consigne.command2 = 0;
    Consigne.command3 = 0; Consigne.command4 = 0;

    Wanted_Forced_Consigne.command1 = 0; Wanted_Forced_Consigne.command2 = 0;
    Wanted_Forced_Consigne.command3 = 0; Wanted_Forced_Consigne.command4 = 0;

    old_Consigne.command1 = 0; old_Consigne.command2 = 0;
    old_Consigne.command3 = 0; old_Consigne.command4 = 0;

    R_lidar[0] = OBS_NOISE_LIDAR_X     * OBS_NOISE_LIDAR_X;
    R_lidar[1] = OBS_NOISE_LIDAR_Y     * OBS_NOISE_LIDAR_Y;
    R_lidar[2] = OBS_NOISE_LIDAR_THETA * OBS_NOISE_LIDAR_THETA;

    en_kalman.enable_lidar_kalman  = 1;
    en_kalman.enable_camera_kalman = 0;

    asserv_init();
    Last_Timer_Asserv = Timer_ms1;
}


void Asserv_Loop(void)
{
    static int last_timing_print_ms = 0;
    if ((Timer_ms1 - last_timing_print_ms) >= 1000) {
        last_timing_print_ms = Timer_ms1;
        ts_print_all();
    }

    // =================================================================
    // SECTION 1 — FAST LOOP : ODO + Kalman predict — cadencé à 1ms
    // =================================================================
    if ((Timer_ms1 - last_odo_ms) >= ODO_EVERY_MS) {
        last_odo_ms += ODO_EVERY_MS;

        T_START(fast_total);

        int s1, s2, s3, s4;
        uint32_t cpsr = mfcpsr();
        mtcpsr(cpsr | 0x80);
        s1 = speed_motor_1; s2 = speed_motor_2;
        s3 = speed_motor_3; s4 = speed_motor_4;
        mtcpsr(cpsr);

        odo_speed_step(s1, s2, s3, s4);
        odo_position_step(ODO_EVERY_MS * 0.001f);

        T_START(fast_imu);
        if (imu_ok) { BNO085_Poll(&imu); }
        T_STOP(fast_imu, ts_fast_imu);

        uint8_t imu_available_for_kalman = 0;
        float bno_vtheta = 0.0f;

        T_START(fast_kalman);
        kalman_predict(&kalman_current_state, ODO_EVERY_MS * 0.001f);
        kalman_update_odo(&kalman_current_state, &speed_robot_odom);

        if (imu.data.new_data) {
            bno_vtheta        = imu.data.gyro.z;
            imu.data.new_data = 0;
            if (imu.data.calib_status >= 1) {
                imu_available_for_kalman = 1;
                kalman_update_imu(&kalman_current_state, bno_vtheta);
            }
        }

        kalman_fifo_push(&kalman_fifo, &kalman_current_state, &speed_robot_odom,
                         imu_available_for_kalman, bno_vtheta);
        T_STOP(fast_kalman, ts_fast_kalman);

        // --- Tick de repropagate asynchrone ---
        // On ne mesure le tick QUE si le job est actif (évite de polluer
        // ts_reprop_tick avec des mesures à 0µs sur les 990ms où rien ne tourne).
        if (repropagate_job.active) {
            T_START(reprop_tick);
            int done = kalman_fifo_repropagate_tick(&kalman_fifo);
            T_STOP(reprop_tick, ts_reprop_tick);

            // Quand le job se termine, on enregistre la durée totale (wall clock).
            // kalman_current_state a déjà été mis à jour dans repropagate_tick.
            if (done) { REPROP_JOB_END(); }
        }

        odo_count++;
        if (odo_count >= ASSERV_EVERY) { odo_count = 0; slow_loop_due = 1; }
        T_STOP(fast_total, ts_fast_total);
    }

    // =================================================================
    // SECTION 2 — COMMANDS : fusion Kalman — dès que disponible
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

        T_START(slow_total);

        odo_speed_cumulate_step(ASSERV_EVERY);

        local_data.kalman_out.x          = kalman_current_state.x[0];
        local_data.kalman_out.y          = kalman_current_state.x[1];
        local_data.kalman_out.t          = kalman_current_state.x[2];
        local_data.speed_robot           = speed_robot_asserv;
        local_data.cmd_speed_constrained = speed_order_constrained;
        SEND_FIELD(&local_data, kalman_out);
        SEND_FIELD(&local_data, speed_robot);
        SEND_FIELD(&local_data, cmd_speed_constrained);

        T_START(slow_motion);
        motion_step();
        constrain_speed_order();
        constrain_acceleration_order(ASSERV_EVERY * ODO_EVERY_MS * 0.001f);
        T_STOP(slow_motion, ts_slow_motion);

        T_START(slow_pid_can);
        Asserv_PWM_calculator(&Consigne);

        if (Wanted_Forced_Consigne.command1 != 0 || Wanted_Forced_Consigne.command2 != 0 ||
            Wanted_Forced_Consigne.command3 != 0 || Wanted_Forced_Consigne.command4 != 0) {
            Consigne = Wanted_Forced_Consigne;
        }

        float c_max = Max_Quatre(Abs_Ternaire(Consigne.command1), Abs_Ternaire(Consigne.command2),
                                 Abs_Ternaire(Consigne.command3), Abs_Ternaire(Consigne.command4));
        if (c_max > 10000.0f) {
            float r = 10000.0f / c_max;
            Consigne.command1 *= r; Consigne.command2 *= r;
            Consigne.command3 *= r; Consigne.command4 *= r;
        }

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

        T_STOP(slow_pid_can, ts_slow_pid_can);
        T_STOP(slow_total,   ts_slow_total);
    }
}


void Set_Lidar_Noise_Cmd(Set_lidar_noise kalman_noise_lidar) {
    R_lidar[0] = kalman_noise_lidar.process_noise_lidar_x * kalman_noise_lidar.process_noise_lidar_x;
    R_lidar[1] = kalman_noise_lidar.process_noise_lidar_y * kalman_noise_lidar.process_noise_lidar_y;
    R_lidar[2] = kalman_noise_lidar.process_noise_lidar_t * kalman_noise_lidar.process_noise_lidar_t;
}

void Set_Kalman_Enable_Cmd(Enable_Kalman enable_kalman) {
    en_kalman.enable_lidar_kalman  = enable_kalman.enable_lidar_kalman;
    en_kalman.enable_camera_kalman = enable_kalman.enable_camera_kalman;
}


int count_lidar_cycle = 0;

void Process_Shared_Memory_Commands(void) {
    if (CHECK_FIELD(&local_data, cmd_position))  { motion_pos(local_data.cmd_position); }
    if (CHECK_FIELD(&local_data, cmd_speed))     { motion_speed(local_data.cmd_speed); }
    if (CHECK_FIELD(&local_data, cmd_abs_speed)) { motion_absolute_speed(local_data.cmd_abs_speed); }

    if (CHECK_FIELD(&local_data, asserv_mode)) {
        if      (local_data.asserv_mode == 0) { motion_free(); }
        else if (local_data.asserv_mode == 4) { motion_block(); }
        else if (local_data.asserv_mode == 5) { start_wheel_ff_calibration(); }
    }

    if (CHECK_FIELD(&local_data, set_pos))            { set_position(local_data.set_pos); }
    if (CHECK_FIELD(&local_data, vmax))               { set_Constraint_vitesse_xy_max(local_data.vmax); }
    if (CHECK_FIELD(&local_data, vtmax))              { set_Constraint_vt_max(local_data.vtmax); }
    if (CHECK_FIELD(&local_data, amax))               { set_Constraint_a_xy_max(local_data.amax); }
    if (CHECK_FIELD(&local_data, cmd_esc))            { Wanted_Forced_Consigne = local_data.cmd_esc; }
    if (CHECK_FIELD(&local_data, enable_kalman))      { Set_Kalman_Enable_Cmd(local_data.enable_kalman); }
    if (CHECK_FIELD(&local_data, odo_spacing))        { odo_set_spacing(local_data.odo_spacing); }
    if (CHECK_FIELD(&local_data, kalman_noise_lidar)) { Set_Lidar_Noise_Cmd(local_data.kalman_noise_lidar); }

    if (!kalman_initialized) {
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

    int earliest_index = -1;
    int earliest_delay = -1;

    if (CHECK_FIELD(&local_data, set_lidar) && en_kalman.enable_lidar_kalman) {
        int idx = kalman_fifo_insert_lidar(&kalman_fifo, &local_data.set_lidar, R_lidar);
        if (idx >= 0 && local_data.set_lidar.delay > earliest_delay) {
            earliest_delay = local_data.set_lidar.delay;
            earliest_index = idx;
        }
    }

    Set_camera* cameras[3] = { &local_data.set_camera_1,
                                &local_data.set_camera_2,
                                &local_data.set_camera_3 };
    uint8_t cam_fields[3] = {
        CHECK_FIELD(&local_data, set_camera_1),
        CHECK_FIELD(&local_data, set_camera_2),
        CHECK_FIELD(&local_data, set_camera_3)
    };

    for (int i = 0; i < 3; i++) {
        if (cam_fields[i] && en_kalman.enable_camera_kalman) {
            int idx = kalman_fifo_insert_camera(&kalman_fifo, cameras[i], i);
            if (idx >= 0 && cameras[i]->delay > earliest_delay) {
                earliest_delay = cameras[i]->delay;
                earliest_index = idx;
            }
        }
    }

    if (earliest_index >= 0) {
        // Lance le job asynchrone. REPROP_JOB_START() note le timestamp de départ
        // pour que REPROP_JOB_END() (appelé dans la fast loop au dernier tick)
        // puisse calculer la durée totale wall-clock du job.
        kalman_fifo_repropagate_start(&kalman_fifo, earliest_index,
                                       ODO_EVERY_MS * 0.001f, R_lidar);
        REPROP_JOB_START();
    }
}