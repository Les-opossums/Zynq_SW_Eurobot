/**
 * @file asserv.c
 * @brief Boucle principale d'asservissement, d'odométrie et de filtrage (Kalman).
 * 
 * Ce fichier gère la réception des commandes IPC, la boucle rapide (odométrie 
 * et fusion de capteurs à haute fréquence) et la boucle lente (calculs de 
 * trajectoire, PID et envois des commandes moteurs via CAN).
 */

#include "asserv.h"
#include "../../../opossum_common/TIMER_MANAGER/timing_stats.h"

/* Activation/desactivation du monitoring : cf TIMING_MEASURE dans
 * opossum_common/TIMER_MANAGER/timing_stats.h (interrupteur unique,
 * commun aux 2 cœurs). */

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
 * Fusion de l'orientation absolue de l'IMU (recalage sur le repere monde)
 * ================================================================== *
 * L'IMU (BNO085) fournit un cap absolu (imu_yaw, cf driver_bno085_io.c) dont
 * l'origine est arbitraire (Game Rotation Vector) ou liee au Nord magnetique
 * (Rotation Vector). Pour l'exploiter dans le repere de la table, on mesure un
 * offset  monde = imu_yaw + offset  par moyenne circulaire sur plusieurs
 * mesures de cap absolu du lidar (set_lidar.lidar_position_t, calcule par le
 * Raspberry Pi).
 *
 * Sequence :
 *   - Au relachement de l'arret d'urgence (front descendant de AU_state) et
 *     tant que des trames lidar arrivent, on accumule les ecarts
 *     (lidar_theta - imu_yaw) -> etat RUNNING, le bandeau LED affiche une gauge.
 *   - Une fois HEADING_INIT_SAMPLES echantillons cumules, l'offset est fige
 *     -> etat READY (bandeau vert).
 *   - Quand la laisse est tiree (leash_state==1) -> etat DONE (bandeau eteint),
 *     le match est lance.
 * En READY et en DONE, chaque nouvelle mesure de cap IMU corrige le Kalman via
 * kalman_update_imu_theta(). L'init est rejouee a chaque nouveau relachement
 * d'AU. */
#define HEADING_INIT_SAMPLES 30u  /* nombre de trames lidar moyennees pour figer l'offset */

static struct {
    uint32_t state;         /* LOC_INIT_* (cf IPC_structure.h) */
    uint32_t prev_au;       /* AU_state precedent, pour detecter le front descendant */
    uint32_t count;         /* echantillons cumules pendant RUNNING */
    float    sum_sin;       /* somme des sin(lidar_theta - imu_yaw) (moyenne circulaire) */
    float    sum_cos;       /* somme des cos(lidar_theta - imu_yaw) */
    float    offset;        /* offset fige (rad) : monde = imu_yaw + offset */
    uint32_t last_yaw_seq;  /* dernier imu_yaw_seq consomme (evite de rejouer une mesure) */
} heading;

/* Publie l'etat de l'init vers CORE0 (bandeau LED). */
static void heading_publish(void)
{
    uint32_t prog;
    if (heading.state == LOC_INIT_READY || heading.state == LOC_INIT_DONE) {
        prog = 100u;
    } else if (heading.state == LOC_INIT_RUNNING) {
        prog = (heading.count * 100u) / HEADING_INIT_SAMPLES;
        if (prog > 100u) prog = 100u;
    } else {
        prog = 0u;
    }
    IPC_DATA->loc_init_state    = heading.state;
    IPC_DATA->loc_init_progress = prog;
}

static void heading_fusion_init(void)
{
    heading.state        = LOC_INIT_IDLE;
    heading.prev_au      = 1u; /* suppose l'AU appuye au boot : le 1er relachement lancera l'init */
    heading.count        = 0u;
    heading.sum_sin      = 0.0f;
    heading.sum_cos      = 0.0f;
    heading.offset       = 0.0f;
    heading.last_yaw_seq = IPC_DATA->imu_yaw_seq;
    heading_publish();
}

/* Appelee a chaque trame lidar recue (theta absolu monde, rad). Cumule l'offset
 * pendant la phase RUNNING. */
static void heading_fusion_on_lidar(float lidar_theta)
{
    if (heading.state != LOC_INIT_RUNNING) return;

    /* Pas d'orientation IMU encore recue : on attend (la gauge ne progresse pas)
     * pour ne pas moyenner l'offset contre un cap non initialise. */
    if (IPC_DATA->imu_yaw_seq == 0u) return;

    float d = lidar_theta - IPC_DATA->imu_yaw; /* imu_yaw : ~100 Hz, toujours frais vs le lidar */
    heading.sum_sin += sinf(d);
    heading.sum_cos += cosf(d);
    heading.count++;

    if (heading.count >= HEADING_INIT_SAMPLES) {
        heading.offset = atan2f(heading.sum_sin, heading.sum_cos);
        heading.state  = LOC_INIT_READY;
    }
    heading_publish();
}

/* Applique la correction Kalman du cap absolu si une nouvelle mesure IMU est
 * disponible depuis le dernier appel. */
static void heading_apply_correction(KalmanState *state)
{
    uint32_t seq = IPC_DATA->imu_yaw_seq;
    if (seq == heading.last_yaw_seq) return;
    heading.last_yaw_seq = seq;

    float meas = principal_angle(IPC_DATA->imu_yaw + heading.offset);
    kalman_update_imu_theta(state, meas);
}

/* Machine a etats, appelee a chaque cycle de la boucle rapide (apres les updates
 * odo / imu-gyro, avant le push FIFO). */
static void heading_fusion_update(KalmanState *state)
{
    uint32_t au = IPC_DATA->AU_state;

    /* Front descendant de l'AU -> (re)demarrage de l'init. */
    if (heading.prev_au && !au) {
        heading.count   = 0u;
        heading.sum_sin = 0.0f;
        heading.sum_cos = 0.0f;
        heading.state   = LOC_INIT_RUNNING;
    }
    heading.prev_au = au;

    /* AU appuye : robot desarme, pas de fusion ni de progression. */
    if (au) { heading_publish(); return; }

    if (heading.state == LOC_INIT_READY) {
        heading_apply_correction(state);
        if (IPC_DATA->leash_state == 1u) {
            heading.state = LOC_INIT_DONE; /* laisse tiree -> match lance */
        }
    } else if (heading.state == LOC_INIT_DONE) {
        heading_apply_correction(state); /* on continue a recaler le cap pendant le match */
    }

    heading_publish();
}

/* ================================================================== *
 * Monitoring des Temps d'Execution (TIMING_MEASURE)
 * ================================================================== *
 * Port de l'instrumentation de l'ancien firmware (TimingStats/T_START/
 * T_STOP, cf opossum_common/TIMER_MANAGER/timing_stats.h), adapte a la
 * structure actuelle fast_loop()/slow_loop().
 * Budget fast : ODO_PERIOD_MS * 1000 us | Budget slow : ASSERV_PERIOD_MS * 1000 us
 */
#if defined(TIMING_MEASURE)

static TimingStats ts_fast_odo, ts_fast_kalman, ts_fast_imu, ts_fast_fifo, ts_fast_total;
static TimingStats ts_slow_motion, ts_slow_constrain, ts_slow_pid_can, ts_slow_ipc, ts_slow_total;
static uint32_t    last_timing_print_ms;

static void timing_print_all(void)
{
    xil_printf("--- Timing asserv (CPU%d) (budget fast=%u us, slow=%u us) ---\r\n",
               THIS_CORE_ID, (unsigned)(ODO_PERIOD_MS * 1000U), (unsigned)(ASSERV_PERIOD_MS * 1000U));
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

    // Fusion heading : alimente le moyennage de l'offset IMU<->monde pendant la
    // phase d'init (sans effet hors phase RUNNING). Fait avant le "return" ci-
    // dessous pour que la gauge progresse meme si le Kalman n'est pas encore init.
    if (lidar_received) {
        heading_fusion_on_lidar(rx.set_lidar.lidar_position_t);
    }

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

    // 4bis. Fusion de l'orientation absolue de l'IMU (cap recale sur le repere
    // monde via l'offset lidar) + machine a etats de l'init (gauge/LED). La
    // correction du cap est appliquee sur l'etat courant, avant le push FIFO,
    // pour que l'etat sauvegarde soit deja corrige.
    heading_fusion_update(&kalman_current_state);

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

    heading_fusion_init();

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
        // Marge temps-reel CPU1 : etapes de l'asserv + peripheriques
        // IO_Manager (dont CAN_MOTORS, la "com" moteurs de ce cœur).
        if (ts_trigger_ms(1000U, &last_timing_print_ms)) {
            timing_print_all();
            IO_Manager_PrintTiming();
        }
#endif
    }
}
