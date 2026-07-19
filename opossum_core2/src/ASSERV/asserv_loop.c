/**
 * @file asserv.c
 * @brief Boucle principale d'asservissement, d'odométrie et de filtrage (Kalman).
 * 
 * Ce fichier gère la réception des commandes IPC, la boucle rapide (odométrie 
 * et fusion de capteurs à haute fréquence) et la boucle lente (calculs de 
 * trajectoire, PID et envois des commandes moteurs via CAN).
 */

#include "asserv.h"

/* Decommenter pour activer le monitoring des temps d'execution de l'asserv
 * (marge par rapport au budget temps-reel de chaque etape). Impressions
 * xil_printf periodiques sur la boucle lente. */
#define TIMING_MEASURE

#if defined(TIMING_MEASURE)
#include "xil_printf.h"
#endif

/* ================================================================== *
 * Macros et Constantes de Temps
 * ================================================================== */
#define ODO_PERIOD_MS         ODO_EVERY_MS
#define ASSERV_PERIOD_MS      (ASSERV_EVERY * ODO_EVERY_MS)
#define ASSERV_TICKS_PER_SLOW (ASSERV_PERIOD_MS / ODO_PERIOD_MS)

/* ================================================================== *
 * Types et Structures Locales
 * ================================================================== */
typedef struct {
    Enable_Kalman kalman_enable;
    float         lidar_noise[3];
    uint8_t       kalman_initialized;
    uint8_t       manual_esc_enabled;
    ESC_Command   manual_esc;
    uint32_t      last_imu_sequence;
} asserv_context_t;

/* ================================================================== *
 * Variables Globales Locales (Static)
 * ================================================================== */
static asserv_context_t ctx;
static uint32_t         last_fast_ms;
static uint8_t          fast_ticks_since_slow;

/* ================================================================== *
 * Monitoring des Temps d'Execution (TIMING_MEASURE)
 * ================================================================== *
 * Port de l'instrumentation de l'ancien firmware (TimingStats/T_START/
 * T_STOP), adapte a la structure actuelle fast_loop()/slow_loop().
 * Budget fast : ODO_PERIOD_MS * 1000 us | Budget slow : ASSERV_PERIOD_MS * 1000 us
 */
#if defined(TIMING_MEASURE)

typedef struct {
    uint32_t min_us;
    uint32_t max_us;
    uint32_t last_us;
    uint64_t sum_us;
    uint32_t count;
} TimingStats;

#define T_START(id)       uint32_t _t_##id = Timer_us1
#define T_STOP(id, stats) ts_update(&(stats), Timer_us1 - _t_##id)

static void ts_update(TimingStats *ts, uint32_t dt_us)
{
    if (ts->count == 0U || dt_us < ts->min_us) { ts->min_us = dt_us; }
    if (dt_us > ts->max_us)                    { ts->max_us = dt_us; }
    ts->last_us = dt_us;
    ts->sum_us += dt_us;
    ts->count++;
}

static void ts_print_one(const char *name, TimingStats *ts)
{
    if (ts->count == 0U) { return; }
    xil_printf("  %-11s min=%4u max=%4u avg=%4u last=%4u us (n=%u)\r\n",
               name, (unsigned)ts->min_us, (unsigned)ts->max_us,
               (unsigned)(ts->sum_us / ts->count), (unsigned)ts->last_us,
               (unsigned)ts->count);
    ts->min_us = 0xFFFFFFFFU;
    ts->max_us = 0U;
    ts->sum_us = 0U;
    ts->count  = 0U;
}

static TimingStats ts_fast_odo, ts_fast_kalman, ts_fast_imu, ts_fast_fifo, ts_fast_total;
static TimingStats ts_slow_motion, ts_slow_constrain, ts_slow_pid_can, ts_slow_ipc, ts_slow_total;
static uint32_t    slow_print_counter;

#define TIMING_PRINT_EVERY_N_SLOW  100U /* ~1s pour ASSERV_PERIOD_MS = 10ms */

static void timing_print_all(void)
{
    xil_printf("--- Timing asserv (budget fast=%u us, slow=%u us) ---\r\n",
               (unsigned)(ODO_PERIOD_MS * 1000U), (unsigned)(ASSERV_PERIOD_MS * 1000U));
    ts_print_one("fast_odo",    &ts_fast_odo);
    ts_print_one("fast_kalman", &ts_fast_kalman);
    ts_print_one("fast_imu",    &ts_fast_imu);
    ts_print_one("fast_fifo",   &ts_fast_fifo);
    ts_print_one("fast_total",  &ts_fast_total);
    ts_print_one("slow_motion", &ts_slow_motion);
    ts_print_one("slow_constr", &ts_slow_constrain);
    ts_print_one("slow_pidcan", &ts_slow_pid_can);
    ts_print_one("slow_ipc",    &ts_slow_ipc);
    ts_print_one("slow_total",  &ts_slow_total);
}

#endif /* TIMING_MEASURE */

/* ================================================================== *
 * Fonctions Utilitaires
 * ================================================================== */

/**
 * @brief Sature la commande moteur pour éviter de dépasser les limites matérielles.
 * @param command La puissance calculée (flottant).
 * @return La puissance bornée (entier 16 bits).
 */
static int16_t saturate_motor_command(float command)
{
    if (command > MOTOR_POWER_MAX) return MOTOR_POWER_MAX;
    if (command < MOTOR_POWER_MIN) return MOTOR_POWER_MIN;
    return (int16_t)command;
}

/* ================================================================== *
 * Gestion des Commandes Inter-Cœurs (IPC)
 * ================================================================== */

/**
 * @brief Vérifie et applique toutes les commandes reçues de l'autre cœur via IPC.
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

    /* --- Commandes de Mouvement et Asservissement --- */
    
    if (CHECK_FIELD(&rx, cmd_position))  { motion_set_position(&rx.cmd_position); }
    if (CHECK_FIELD(&rx, cmd_speed))     { motion_set_speed(&rx.cmd_speed); }
    if (CHECK_FIELD(&rx, cmd_abs_speed)) { motion_set_absolute_speed(&rx.cmd_abs_speed); }
    
    if (CHECK_FIELD(&rx, asserv_mode)) {
        ctx.manual_esc_enabled = 0;
        if (rx.asserv_mode == 0) { 
            motion_free();    
        } 
        else if (rx.asserv_mode == 4) { 
            motion_block();   
        }
    }

    /* --- Commandes de Configuration et Contraintes --- */
    
    if (CHECK_FIELD(&rx, set_pos)) {
        odometry_set_position(&rx.set_pos);
        kalman_init_with_lidar(&kalman_fifo, &rx.set_pos);
        ctx.kalman_initialized = 1;
    }

    if (CHECK_FIELD(&rx, vmax))        { set_Constraint_vitesse_xy_max(rx.vmax); }
    if (CHECK_FIELD(&rx, vtmax))       { set_Constraint_vt_max(rx.vtmax); }
    if (CHECK_FIELD(&rx, amax))        { set_Constraint_a_xy_max(rx.amax); }
    if (CHECK_FIELD(&rx, odo_spacing)) { odometry_set_wheel_distance(rx.odo_spacing); }

    /* --- Mode Manuel (Override) --- */
    
    if (CHECK_FIELD(&rx, cmd_esc)) {
        ctx.manual_esc = rx.cmd_esc;
        ctx.manual_esc_enabled = 1; // Active la dérogation matérielle
    }

    /* --- Paramétrage du Filtre de Kalman --- */
    
    if (CHECK_FIELD(&rx, enable_kalman)) { 
        ctx.kalman_enable = rx.enable_kalman; 
    }

    if (CHECK_FIELD(&rx, kalman_noise_lidar)) {
        // Mise au carré pour obtenir la variance
        ctx.lidar_noise[0] = rx.kalman_noise_lidar.process_noise_lidar_x * rx.kalman_noise_lidar.process_noise_lidar_x;
        ctx.lidar_noise[1] = rx.kalman_noise_lidar.process_noise_lidar_y * rx.kalman_noise_lidar.process_noise_lidar_y;
        ctx.lidar_noise[2] = rx.kalman_noise_lidar.process_noise_lidar_t * rx.kalman_noise_lidar.process_noise_lidar_t;
    }

    /* --- Gestion des Observations Lidar et Caméras --- */
    
    const uint8_t lidar_received = CHECK_FIELD(&rx, set_lidar);

    // Si le filtre n'est pas encore initialisé, on force l'initialisation avec la première trame Lidar
    if (!ctx.kalman_initialized) {
        if (lidar_received) {
            Position initial = {
                rx.set_lidar.lidar_position_x, 
                rx.set_lidar.lidar_position_y, 
                rx.set_lidar.lidar_position_t
            };
            odometry_set_position(&initial);
            kalman_init_with_lidar(&kalman_fifo, &initial);
            ctx.kalman_initialized = 1;
        }
        return; // On ne traite pas le reste tant que l'init n'est pas faite
    }

    int earliest_index = -1;
    int largest_delay = -1;

    // Insertion Lidar
    if (lidar_received && ctx.kalman_enable.enable_lidar_kalman) {
        int index = kalman_fifo_insert_lidar(&kalman_fifo, &rx.set_lidar, ctx.lidar_noise);
        if (index >= 0) {
            earliest_index = index;
            largest_delay = (int)rx.set_lidar.delay;
        }
    }

    // Insertion Caméra 1
    if (CHECK_FIELD(&rx, set_camera_1) && ctx.kalman_enable.enable_camera_kalman) {
        int index = kalman_fifo_insert_camera(&kalman_fifo, &rx.set_camera_1, 0);
        if (index >= 0 && (int)rx.set_camera_1.delay > largest_delay) {
            earliest_index = index;
            largest_delay = (int)rx.set_camera_1.delay;
        }
    }

    // Insertion Caméra 2
    if (CHECK_FIELD(&rx, set_camera_2) && ctx.kalman_enable.enable_camera_kalman) {
        int index = kalman_fifo_insert_camera(&kalman_fifo, &rx.set_camera_2, 1);
        if (index >= 0 && (int)rx.set_camera_2.delay > largest_delay) {
            earliest_index = index;
            largest_delay = (int)rx.set_camera_2.delay;
        }
    }

    // Insertion Caméra 3
    if (CHECK_FIELD(&rx, set_camera_3) && ctx.kalman_enable.enable_camera_kalman) {
        int index = kalman_fifo_insert_camera(&kalman_fifo, &rx.set_camera_3, 2);
        if (index >= 0 && (int)rx.set_camera_3.delay > largest_delay) {
            earliest_index = index;
            largest_delay = (int)rx.set_camera_3.delay;
        }
    }

    // Si on a inséré de nouvelles données dans le passé, on relance la propagation
    if (earliest_index >= 0) {
        kalman_fifo_repropagate_start(&kalman_fifo, earliest_index, ODO_PERIOD_MS * 0.001f, ctx.lidar_noise);
    }
}

/* ================================================================== *
 * Boucles de Contrôle
 * ================================================================== */

/**
 * @brief Boucle rapide (Haute fréquence). 
 * Gère l'odométrie, la prédiction et la mise à jour du filtre de Kalman.
 */
static void fast_loop(void)
{
#if defined(TIMING_MEASURE)
    T_START(fast_total);
#endif

    // 1. Lecture des retours encodeurs des moteurs
    int motor_rpm[4] = {
        motor_feedback[0].speed_motor,
        motor_feedback[1].speed_motor,
        motor_feedback[2].speed_motor,
        motor_feedback[3].speed_motor
    };

#if defined(TIMING_MEASURE)
    T_START(fast_odo);
#endif
    // 2. Mise à jour et intégration de l'odométrie pure
    odometry_update_from_motor_rpm(motor_rpm);
    odometry_integrate(ODO_PERIOD_MS * 0.001f);
#if defined(TIMING_MEASURE)
    T_STOP(fast_odo, ts_fast_odo);
#endif

#if defined(TIMING_MEASURE)
    T_START(fast_kalman);
#endif
    // 3. Prédiction du filtre de Kalman
    kalman_predict(&kalman_current_state, ODO_PERIOD_MS * 0.001f);
    kalman_update_odo(&kalman_current_state, &speed_robot_odom);
#if defined(TIMING_MEASURE)
    T_STOP(fast_kalman, ts_fast_kalman);
#endif

    // 4. Vérification et intégration des données IMU
    const uint32_t sequence_before = IPC_DATA->imu_seq;
    const float    gyro_z          = IPC_DATA->imu_gyro_z;
    const uint32_t calibration     = IPC_DATA->imu_calib_status;

    const uint8_t imu_available = (sequence_before == IPC_DATA->imu_seq &&
                                   sequence_before != ctx.last_imu_sequence &&
                                   calibration >= 1U);

#if defined(TIMING_MEASURE)
    T_START(fast_imu);
#endif
    if (imu_available) {
        ctx.last_imu_sequence = sequence_before;
        kalman_update_imu(&kalman_current_state, gyro_z);
    }
#if defined(TIMING_MEASURE)
    T_STOP(fast_imu, ts_fast_imu);
#endif

#if defined(TIMING_MEASURE)
    T_START(fast_fifo);
#endif
    // 5. Sauvegarde dans la FIFO (pour gestion du retard des capteurs externes)
    kalman_fifo_push(&kalman_fifo, &kalman_current_state, &speed_robot_odom, imu_available, gyro_z);

    // 6. Gestion du lissage (Repropagation) en tâche de fond
    if (repropagate_job.active) {
        (void)kalman_fifo_repropagate_tick(&kalman_fifo);
    }
#if defined(TIMING_MEASURE)
    T_STOP(fast_fifo, ts_fast_fifo);
    T_STOP(fast_total, ts_fast_total);
#endif
}

/**
 * @brief Boucle lente (Basse fréquence).
 * Gère le générateur de trajectoire, l'asservissement PID et l'envoi CAN.
 */
static void slow_loop(void)
{
#if defined(TIMING_MEASURE)
    T_START(slow_total);
#endif

#if defined(TIMING_MEASURE)
    T_START(slow_motion);
#endif
    // 1. Récupération de la position estimée actuelle
    Position current = {
        kalman_current_state.x[0],
        kalman_current_state.x[1],
        kalman_current_state.x[2]
    };

    // 2. Moyennage de l'odométrie lente et calcul des consignes
    odometry_finish_slow(ASSERV_TICKS_PER_SLOW);
    motion_step(&current);
#if defined(TIMING_MEASURE)
    T_STOP(slow_motion, ts_slow_motion);
#endif

#if defined(TIMING_MEASURE)
    T_START(slow_constrain);
#endif
    // 3. Application des contraintes physiques (rampement et saturation)
    constrain_speed_order();
    constrain_acceleration_order(ASSERV_PERIOD_MS * 0.001f);
#if defined(TIMING_MEASURE)
    T_STOP(slow_constrain, ts_slow_constrain);
#endif

#if defined(TIMING_MEASURE)
    T_START(slow_pid_can);
#endif
    // 4. Calcul des puissances moteurs (Asservissement PID)
    ESC_Command command;
    Asserv_PWM_calculator(&command);

    // 5. Écrasement manuel si la dérogation est active
    if (ctx.manual_esc_enabled) {
        command = ctx.manual_esc;
    }

    // 6. Saturation finale des commandes
    int16_t motors[4] = {
        saturate_motor_command(command.command1),
        saturate_motor_command(command.command2),
        saturate_motor_command(command.command3),
        saturate_motor_command(command.command4)
    };

    // 7. SÉCURITÉ : Coupure matérielle (Arrêt d'Urgence)
    if (IPC_DATA->AU_state) {
        motors[0] = motors[1] = motors[2] = motors[3] = 0;
        pid_vitesse_reset(); // Évite l'accumulation de l'erreur intégrale
    }

    // 8. Transmission au bus CAN
    CAN_transmit_motor(&Can0_Ctx, motors, 4);
#if defined(TIMING_MEASURE)
    T_STOP(slow_pid_can, ts_slow_pid_can);
#endif

#if defined(TIMING_MEASURE)
    T_START(slow_ipc);
#endif
    // 9. Remontée de la télémétrie vers l'autre cœur via IPC
    (void)IPC_SendToOtherCore(&current, sizeof(current),
                              &IPC_DATA->kalman_out, &IPC_DATA->flag_kalman_out_valid, &IPC_DATA->flag_kalman_out_ack);

    (void)IPC_SendToOtherCore(&speed_robot_asserv, sizeof(speed_robot_asserv),
                              &IPC_DATA->speed_robot, &IPC_DATA->flag_speed_robot_valid, &IPC_DATA->flag_speed_robot_ack);

    (void)IPC_SendToOtherCore(&speed_order_constrained, sizeof(speed_order_constrained),
                              &IPC_DATA->cmd_speed_constrained, &IPC_DATA->flag_cmd_speed_constrained_valid, &IPC_DATA->flag_cmd_speed_constrained_ack);
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
    ctx = (asserv_context_t){0};
    
    // Configuration par défaut du filtre de Kalman
    ctx.kalman_enable.enable_lidar_kalman = 1;
    ctx.lidar_noise[0] = OBS_NOISE_LIDAR_X * OBS_NOISE_LIDAR_X;
    ctx.lidar_noise[1] = OBS_NOISE_LIDAR_Y * OBS_NOISE_LIDAR_Y;
    ctx.lidar_noise[2] = OBS_NOISE_LIDAR_THETA * OBS_NOISE_LIDAR_THETA;

    // Initialisation des sous-systèmes
    Init_CAN_MOTOR_variables();
    odometry_init();
    motion_init();
    speed_constrainer_init();
    acceleration_constrainer_init();
    pid_vitesse_init();
    pid_vitesse_reset();
    
    kalman_fifo_init(&kalman_fifo);
    kalman_init(&kalman_current_state);
    
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
    if ((uint32_t)(now_ms - last_fast_ms) < ODO_PERIOD_MS) {
        return;
    }
    
    // Mise à jour du timer (évite le rattrapage brutal de cycles après une surcharge)
    last_fast_ms = now_ms; 

    // Exécution de la boucle rapide
    fast_loop();

    // Gestion du ratio (Diviseur de fréquence pour la boucle lente)
    if (++fast_ticks_since_slow >= ASSERV_TICKS_PER_SLOW) {
        fast_ticks_since_slow = 0;
        slow_loop();

#if defined(TIMING_MEASURE)
        if (++slow_print_counter >= TIMING_PRINT_EVERY_N_SLOW) {
            slow_print_counter = 0;
            timing_print_all();
        }
#endif
    }
}
