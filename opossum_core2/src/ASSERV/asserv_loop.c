/**
 * @file asserv_loop.c
 * @brief Adaptateur Zynq (CPU1) de l'asservissement opossum_control.
 *
 * Toute la logique (odometrie, Kalman, trajectoire, contraintes, PID) est dans
 * libs/opossum_control (C pur, aussi compile sur PC pour la simulation). Ce
 * fichier ne fait que la traduction avec le materiel de CPU1 :
 *   - commandes et observations recues de CORE0 par IPC -> control_apply_commands()
 *   - retours moteurs (CAN) et IMU (IPC)                 -> control_fast_step()
 *   - commandes moteur                                   -> bus CAN
 *   - pose, vitesses, motion_done, etat d'init           -> IPC vers CORE0
 *   - cadencement par Timer_ms1
 */

#include "asserv.h"
#include "../../../opossum_common/TIMER_MANAGER/timing_stats.h"

/* Activation/desactivation du monitoring : cf TIMING_MEASURE dans
 * opossum_common/TIMER_MANAGER/timing_stats.h (interrupteur unique,
 * commun aux 2 cœurs). */

/* Les etats d'init de localisation publies a CORE0 doivent garder les memes
 * valeurs des deux cotes. */
_Static_assert(CONTROL_LOC_INIT_IDLE    == LOC_INIT_IDLE,    "LOC_INIT_IDLE");
_Static_assert(CONTROL_LOC_INIT_RUNNING == LOC_INIT_RUNNING, "LOC_INIT_RUNNING");
_Static_assert(CONTROL_LOC_INIT_READY   == LOC_INIT_READY,   "LOC_INIT_READY");
_Static_assert(CONTROL_LOC_INIT_DONE    == LOC_INIT_DONE,    "LOC_INIT_DONE");

static uint32_t last_fast_ms;

#if defined(TIMING_MEASURE)
static TimingStats ts_slow_can, ts_slow_ipc, ts_slow_total;
static uint32_t    last_timing_print_ms;

static void timing_print_all(void)
{
    xil_printf("--- Timing asserv (CPU%d) (budget fast=%u us, slow=%u us) ---\r\n",
               THIS_CORE_ID, (unsigned)(CONTROL_FAST_PERIOD_MS * 1000U), (unsigned)(CONTROL_SLOW_PERIOD_MS * 1000U));
    ctrl_hal_zynq_print_timing();
    ts_print_one("slow_can",    &ts_slow_can);
    ts_print_one("slow_ipc",    &ts_slow_ipc);
    ts_print_one("slow_total",  &ts_slow_total);
}
#endif /* TIMING_MEASURE */

/* Publie l'etat de l'init de localisation vers CORE0 (bandeau LED). */
static void publish_loc_init(void)
{
    uint32_t state, progress;
    control_get_loc_init(&state, &progress);
    IPC_DATA->loc_init_state    = state;
    IPC_DATA->loc_init_progress = progress;
}

/* ================================================================== *
 * Gestion des Commandes Inter-Cœurs (IPC)
 * ================================================================== */

/**
 * @brief Recupere toutes les commandes recues de l'autre cœur via IPC et les
 * transmet a opossum_control.
 */
static void receive_commands(void)
{
    // Structure locale regroupant les données à recevoir.
    // Les noms des champs DOIVENT correspondre exactement à ceux de IPC_DATA.
    struct {
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
    } rx;
    control_commands_t cmd = {0};

#define TAKE(field) \
    if ((cmd.has_##field = (uint8_t)CHECK_FIELD(&rx, field)) != 0u) { cmd.field = rx.field; }

    TAKE(cmd_position)
    TAKE(cmd_speed)
    TAKE(cmd_abs_speed)
    TAKE(asserv_mode)
    TAKE(set_pos)
    TAKE(vmax)
    TAKE(vtmax)
    TAKE(amax)
    TAKE(odo_spacing)
    TAKE(cmd_esc)
    TAKE(enable_kalman)
    TAKE(kalman_noise_lidar)
    TAKE(set_lidar)
#undef TAKE

    // Les observations camera ne sont lues (donc acquittees) qu'une fois le
    // Kalman initialise (par set_pos ou une premiere trame lidar) : avant, elles
    // restent en attente dans l'IPC, comme dans la version d'origine.
    if (control_kalman_initialized() || cmd.has_set_pos) {
        if ((cmd.has_set_camera[0] = (uint8_t)CHECK_FIELD(&rx, set_camera_1)) != 0u) { cmd.set_camera[0] = rx.set_camera_1; }
        if ((cmd.has_set_camera[1] = (uint8_t)CHECK_FIELD(&rx, set_camera_2)) != 0u) { cmd.set_camera[1] = rx.set_camera_2; }
        if ((cmd.has_set_camera[2] = (uint8_t)CHECK_FIELD(&rx, set_camera_3)) != 0u) { cmd.set_camera[2] = rx.set_camera_3; }
    }

    control_apply_commands(&cmd, IPC_DATA->imu_yaw, IPC_DATA->imu_yaw_seq);
    publish_loc_init();
}

/* ================================================================== *
 * Boucles de Contrôle
 * ================================================================== */

/**
 * @brief Boucle rapide : lecture capteurs puis pas rapide d'opossum_control.
 * @return 1 si un pas lent est du.
 */
static uint8_t fast_loop(void)
{
    control_inputs_t in;

    // Retours encodeurs des moteurs (CAN)
    in.motor_rpm[0] = motor_feedback[0].speed_motor;
    in.motor_rpm[1] = motor_feedback[1].speed_motor;
    in.motor_rpm[2] = motor_feedback[2].speed_motor;
    in.motor_rpm[3] = motor_feedback[3].speed_motor;

    // Gyro IMU publie par CORE0 : relecture du compteur pour detecter une
    // lecture dechiree (CORE0 a ecrit pendant qu'on lisait)
    in.imu_seq          = IPC_DATA->imu_seq;
    in.imu_gyro_z       = IPC_DATA->imu_gyro_z;
    in.imu_calib_status = IPC_DATA->imu_calib_status;
    in.imu_seq_stable   = (uint8_t)(in.imu_seq == IPC_DATA->imu_seq);

    // Cap absolu IMU et etats GPIO (AU, laisse)
    in.au_state    = IPC_DATA->AU_state;
    in.leash_state = IPC_DATA->leash_state;
    in.imu_yaw_seq = IPC_DATA->imu_yaw_seq;
    in.imu_yaw     = IPC_DATA->imu_yaw;

    const uint8_t slow_due = control_fast_step(&in);
    publish_loc_init();
    return slow_due;
}

/**
 * @brief Boucle lente : pas lent d'opossum_control, envoi CAN, telemetrie IPC.
 */
static void slow_loop(void)
{
#if defined(TIMING_MEASURE)
    T_START(slow_total);
#endif
    control_outputs_t out;
    control_slow_step(IPC_DATA->AU_state, &out);

#if defined(TIMING_MEASURE)
    T_START(slow_can);
#endif
    // Transmission au bus CAN
    CAN_transmit_motor(&Can0_Ctx, out.motors, 4);
#if defined(TIMING_MEASURE)
    T_STOP(slow_can, ts_slow_can);
#endif

#if defined(TIMING_MEASURE)
    T_START(slow_ipc);
#endif
    // Remontée de la télémétrie vers l'autre cœur via IPC
    (void)IPC_SendToOtherCore(&out.pose, sizeof(out.pose),
                              &IPC_DATA->kalman_out, &IPC_DATA->flag_kalman_out_valid, &IPC_DATA->flag_kalman_out_ack);

    (void)IPC_SendToOtherCore(&out.speed_robot, sizeof(out.speed_robot),
                              &IPC_DATA->speed_robot, &IPC_DATA->flag_speed_robot_valid, &IPC_DATA->flag_speed_robot_ack);

    (void)IPC_SendToOtherCore(&out.speed_order_constrained, sizeof(out.speed_order_constrained),
                              &IPC_DATA->cmd_speed_constrained, &IPC_DATA->flag_cmd_speed_constrained_valid, &IPC_DATA->flag_cmd_speed_constrained_ack);

    // Log de calibration (emis sur Ethernet par CORE0 si CALIBLOG 1)
    Calib_sample calib_sample;
    control_get_calib_sample(&calib_sample);
    (void)IPC_SendToOtherCore(&calib_sample, sizeof(calib_sample),
                              &IPC_DATA->calib_sample, &IPC_DATA->flag_calib_sample_valid, &IPC_DATA->flag_calib_sample_ack);
#if defined(TIMING_MEASURE)
    T_STOP(slow_ipc, ts_slow_ipc);
    T_STOP(slow_total, ts_slow_total);
#endif
}

/* ================================================================== *
 * API Publique
 * ================================================================== */

/**
 * @brief Initialise tous les modules liés à l'asservissement et au filtrage.
 */
void asserv_loop_init(void)
{
    Init_CAN_MOTOR_variables();
    control_init(IPC_DATA->imu_yaw_seq);
    publish_loc_init();

    last_fast_ms = (uint32_t)Timer_ms1;
}

/**
 * @brief Routine de mise à jour globale. Gère le séquencement des boucles.
 */
void asserv_loop_update(void)
{
    receive_commands();

    const uint32_t now_ms = (uint32_t)Timer_ms1;

    // Attente de l'échéance de la boucle rapide
    if ((uint32_t)(now_ms - last_fast_ms) < CONTROL_FAST_PERIOD_MS) {
        return;
    }

    // Mise à jour du timer (évite le rattrapage brutal de cycles après une surcharge)
    last_fast_ms = now_ms;

    // Exécution de la boucle rapide, puis de la lente tous les CONTROL_TICKS_PER_SLOW
    if (fast_loop()) {
        slow_loop();

#if defined(TIMING_MEASURE)
        // Marge temps-reel CPU1 : etapes de l'asserv + peripheriques
        // IO_Manager (dont CAN_MOTORS, la "com" moteurs de ce cœur).
        if (ts_trigger_ms(1000U, &last_timing_print_ms)) {
            timing_print_all();
            IO_Manager_PrintTiming();
        }
#endif
    }
}
