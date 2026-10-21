/**
 * @file control_loop.c
 * @brief Sequencement de l'asservissement, de l'odometrie et du filtrage
 *        (Kalman), independant de la plateforme.
 *
 * Extrait de l'ancien opossum_core2/src/ASSERV/asserv_loop.c : la logique est
 * identique, seuls les acces materiels (IPC, CAN, timers) sont remplaces par
 * les structures d'entree/sortie de control_loop.h. L'adaptateur Zynq
 * (asserv_loop.c) fait la traduction.
 */

#include "opossum_control.h"
#include <math.h>

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
} control_context_t;

/* ================================================================== *
 * Variables Globales Locales (Static)
 * ================================================================== */
static control_context_t ctx;
static uint8_t           fast_ticks_since_slow;

/* ================================================================== *
 * Fusion de l'orientation absolue de l'IMU (recalage sur le repere monde)
 * ================================================================== *
 * L'IMU (BNO085) fournit un cap absolu (imu_yaw) dont l'origine est
 * arbitraire (Game Rotation Vector) ou liee au Nord magnetique (Rotation
 * Vector). Pour l'exploiter dans le repere de la table, on mesure un offset
 * monde = imu_yaw + offset  par moyenne circulaire sur plusieurs mesures de
 * cap absolu du lidar (set_lidar.lidar_position_t, calcule par le PC).
 *
 * Sequence :
 *   - Au relachement de l'arret d'urgence (front descendant de au_state) et
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
    uint32_t state;         /* CONTROL_LOC_INIT_* */
    uint32_t progress;      /* 0..100 % */
    uint32_t prev_au;       /* au_state precedent, pour detecter le front descendant */
    uint32_t count;         /* echantillons cumules pendant RUNNING */
    float    sum_sin;       /* somme des sin(lidar_theta - imu_yaw) (moyenne circulaire) */
    float    sum_cos;       /* somme des cos(lidar_theta - imu_yaw) */
    float    offset;        /* offset fige (rad) : monde = imu_yaw + offset */
    uint32_t last_yaw_seq;  /* dernier imu_yaw_seq consomme (evite de rejouer une mesure) */
} heading;

/* Met a jour l'avancement publie (bandeau LED de CORE0, via l'adaptateur). */
static void heading_publish(void)
{
    uint32_t prog;
    if (heading.state == CONTROL_LOC_INIT_READY || heading.state == CONTROL_LOC_INIT_DONE) {
        prog = 100u;
    } else if (heading.state == CONTROL_LOC_INIT_RUNNING) {
        prog = (heading.count * 100u) / HEADING_INIT_SAMPLES;
        if (prog > 100u) prog = 100u;
    } else {
        prog = 0u;
    }
    heading.progress = prog;
}

static void heading_fusion_init(uint32_t imu_yaw_seq)
{
    heading.state        = CONTROL_LOC_INIT_IDLE;
    heading.prev_au      = 1u; /* suppose l'AU appuye au boot : le 1er relachement lancera l'init */
    heading.count        = 0u;
    heading.sum_sin      = 0.0f;
    heading.sum_cos      = 0.0f;
    heading.offset       = 0.0f;
    heading.last_yaw_seq = imu_yaw_seq;
    heading_publish();
}

/* Appelee a chaque trame lidar recue (theta absolu monde, rad). Cumule l'offset
 * pendant la phase RUNNING. */
static void heading_fusion_on_lidar(float lidar_theta, float imu_yaw, uint32_t imu_yaw_seq)
{
    if (heading.state != CONTROL_LOC_INIT_RUNNING) return;

    /* Pas d'orientation IMU encore recue : on attend (la gauge ne progresse pas)
     * pour ne pas moyenner l'offset contre un cap non initialise. */
    if (imu_yaw_seq == 0u) return;

    float d = lidar_theta - imu_yaw; /* imu_yaw : ~100 Hz, toujours frais vs le lidar */
    heading.sum_sin += sinf(d);
    heading.sum_cos += cosf(d);
    heading.count++;

    if (heading.count >= HEADING_INIT_SAMPLES) {
        heading.offset = atan2f(heading.sum_sin, heading.sum_cos);
        heading.state  = CONTROL_LOC_INIT_READY;
    }
    heading_publish();
}

/* Applique la correction Kalman du cap absolu si une nouvelle mesure IMU est
 * disponible depuis le dernier appel. */
static void heading_apply_correction(KalmanState *state, float imu_yaw, uint32_t imu_yaw_seq)
{
    if (imu_yaw_seq == heading.last_yaw_seq) return;
    heading.last_yaw_seq = imu_yaw_seq;

    float meas = principal_angle(imu_yaw + heading.offset);
    kalman_update_imu_theta(state, meas);
}

/* Machine a etats, appelee a chaque cycle de la boucle rapide (apres les updates
 * odo / imu-gyro, avant le push FIFO). */
static void heading_fusion_update(KalmanState *state, const control_inputs_t *in)
{
    uint32_t au = in->au_state;

    /* Front descendant de l'AU -> (re)demarrage de l'init. */
    if (heading.prev_au && !au) {
        heading.count   = 0u;
        heading.sum_sin = 0.0f;
        heading.sum_cos = 0.0f;
        heading.state   = CONTROL_LOC_INIT_RUNNING;
    }
    heading.prev_au = au;

    /* AU appuye : robot desarme, pas de fusion ni de progression. */
    if (au) { heading_publish(); return; }

    if (heading.state == CONTROL_LOC_INIT_READY) {
        heading_apply_correction(state, in->imu_yaw, in->imu_yaw_seq);
        if (in->leash_state == 1u) {
            heading.state = CONTROL_LOC_INIT_DONE; /* laisse tiree -> match lance */
        }
    } else if (heading.state == CONTROL_LOC_INIT_DONE) {
        heading_apply_correction(state, in->imu_yaw, in->imu_yaw_seq); /* on continue a recaler le cap pendant le match */
    }

    heading_publish();
}

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
 * Consignes et observations
 * ================================================================== */

void control_apply_commands(const control_commands_t *rx, float imu_yaw, uint32_t imu_yaw_seq)
{
    /* --- Commandes de Mouvement et Asservissement --- */

    if (rx->has_cmd_position)  { motion_set_position(&rx->cmd_position); }
    if (rx->has_cmd_speed)     { motion_set_speed(&rx->cmd_speed); }
    if (rx->has_cmd_abs_speed) { motion_set_absolute_speed(&rx->cmd_abs_speed); }

    if (rx->has_asserv_mode) {
        ctx.manual_esc_enabled = 0;
        if (rx->asserv_mode == 0) {
            motion_free();
        }
        else if (rx->asserv_mode == 4) {
            motion_block();
        }
    }

    /* --- Commandes de Configuration et Contraintes --- */

    if (rx->has_set_pos) {
        Position set_pos = rx->set_pos;
        odometry_set_position(&set_pos);
        kalman_init_with_lidar(&kalman_fifo, &set_pos);
        ctx.kalman_initialized = 1;
    }

    if (rx->has_vmax)        { set_Constraint_vitesse_xy_max(rx->vmax); }
    if (rx->has_vtmax)       { set_Constraint_vt_max(rx->vtmax); }
    if (rx->has_amax)        { set_Constraint_a_xy_max(rx->amax); }
    if (rx->has_odo_spacing) { odometry_set_wheel_distance(rx->odo_spacing); }

    /* --- Mode Manuel (Override) --- */

    if (rx->has_cmd_esc) {
        ctx.manual_esc = rx->cmd_esc;
        ctx.manual_esc_enabled = 1; // Active la dérogation matérielle
    }

    /* --- Paramétrage du Filtre de Kalman --- */

    if (rx->has_enable_kalman) {
        ctx.kalman_enable = rx->enable_kalman;
    }

    if (rx->has_kalman_noise_lidar) {
        // Mise au carré pour obtenir la variance
        ctx.lidar_noise[0] = rx->kalman_noise_lidar.process_noise_lidar_x * rx->kalman_noise_lidar.process_noise_lidar_x;
        ctx.lidar_noise[1] = rx->kalman_noise_lidar.process_noise_lidar_y * rx->kalman_noise_lidar.process_noise_lidar_y;
        ctx.lidar_noise[2] = rx->kalman_noise_lidar.process_noise_lidar_t * rx->kalman_noise_lidar.process_noise_lidar_t;
    }

    /* --- Gestion des Observations Lidar et Caméras --- */

    const uint8_t lidar_received = rx->has_set_lidar;

    // Fusion heading : alimente le moyennage de l'offset IMU<->monde pendant la
    // phase d'init (sans effet hors phase RUNNING). Fait avant le "return" ci-
    // dessous pour que la gauge progresse meme si le Kalman n'est pas encore init.
    if (lidar_received) {
        heading_fusion_on_lidar(rx->set_lidar.lidar_position_t, imu_yaw, imu_yaw_seq);
    }

    // Si le filtre n'est pas encore initialisé, on force l'initialisation avec la première trame Lidar
    if (!ctx.kalman_initialized) {
        if (lidar_received) {
            Position initial = {
                rx->set_lidar.lidar_position_x,
                rx->set_lidar.lidar_position_y,
                rx->set_lidar.lidar_position_t
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
        Set_lidar set_lidar = rx->set_lidar;
        int index = kalman_fifo_insert_lidar(&kalman_fifo, &set_lidar, ctx.lidar_noise);
        if (index >= 0) {
            earliest_index = index;
            largest_delay = (int)rx->set_lidar.delay;
        }
    }

    // Insertion Caméras 1 a 3
    for (uint8_t cam = 0; cam < 3; cam++) {
        if (rx->has_set_camera[cam] && ctx.kalman_enable.enable_camera_kalman) {
            Set_camera set_camera = rx->set_camera[cam];
            int index = kalman_fifo_insert_camera(&kalman_fifo, &set_camera, cam);
            if (index >= 0 && (int)rx->set_camera[cam].delay > largest_delay) {
                earliest_index = index;
                largest_delay = (int)rx->set_camera[cam].delay;
            }
        }
    }

    // Si on a inséré de nouvelles données dans le passé, on relance la propagation
    if (earliest_index >= 0) {
        kalman_fifo_repropagate_start(&kalman_fifo, earliest_index, CONTROL_FAST_PERIOD_MS * 0.001f, ctx.lidar_noise);
    }
}

uint8_t control_kalman_initialized(void)
{
    return ctx.kalman_initialized;
}

/* ================================================================== *
 * Boucles de Contrôle
 * ================================================================== */

/**
 * @brief Boucle rapide (Haute fréquence).
 * Gère l'odométrie, la prédiction et la mise à jour du filtre de Kalman.
 */
uint8_t control_fast_step(const control_inputs_t *in)
{
    ctrl_hal_timing_start(CTRL_T_FAST_TOTAL);

    // 1. Lecture des retours encodeurs des moteurs
    int motor_rpm[4] = {
        in->motor_rpm[0],
        in->motor_rpm[1],
        in->motor_rpm[2],
        in->motor_rpm[3]
    };

    // 2. Mise à jour et intégration de l'odométrie pure
    ctrl_hal_timing_start(CTRL_T_FAST_ODO);
    odometry_update_from_motor_rpm(motor_rpm);
    odometry_integrate(CONTROL_FAST_PERIOD_MS * 0.001f);
    ctrl_hal_timing_stop(CTRL_T_FAST_ODO);

    // 3. Prédiction du filtre de Kalman
    ctrl_hal_timing_start(CTRL_T_FAST_KALMAN);
    kalman_predict(&kalman_current_state, CONTROL_FAST_PERIOD_MS * 0.001f);
    kalman_update_odo(&kalman_current_state, &speed_robot_odom);
    ctrl_hal_timing_stop(CTRL_T_FAST_KALMAN);

    // 4. Vérification et intégration des données IMU
    const uint32_t sequence_before = in->imu_seq;
    const float    gyro_z          = in->imu_gyro_z;
    const uint32_t calibration     = in->imu_calib_status;

    const uint8_t imu_available = (in->imu_seq_stable &&
                                   sequence_before != ctx.last_imu_sequence &&
                                   calibration >= 1U);

    ctrl_hal_timing_start(CTRL_T_FAST_IMU);
    if (imu_available) {
        ctx.last_imu_sequence = sequence_before;
        kalman_update_imu(&kalman_current_state, gyro_z);
    }
    ctrl_hal_timing_stop(CTRL_T_FAST_IMU);

    // 4bis. Fusion de l'orientation absolue de l'IMU (cap recale sur le repere
    // monde via l'offset lidar) + machine a etats de l'init (gauge/LED). La
    // correction du cap est appliquee sur l'etat courant, avant le push FIFO,
    // pour que l'etat sauvegarde soit deja corrige.
    heading_fusion_update(&kalman_current_state, in);

    // 5. Sauvegarde dans la FIFO (pour gestion du retard des capteurs externes)
    ctrl_hal_timing_start(CTRL_T_FAST_FIFO);
    kalman_fifo_push(&kalman_fifo, &kalman_current_state, &speed_robot_odom, imu_available, gyro_z);

    // 6. Gestion du lissage (Repropagation) en tâche de fond
    if (repropagate_job.active) {
        (void)kalman_fifo_repropagate_tick(&kalman_fifo);
    }
    ctrl_hal_timing_stop(CTRL_T_FAST_FIFO);
    ctrl_hal_timing_stop(CTRL_T_FAST_TOTAL);

    // Diviseur de fréquence pour la boucle lente
    if (++fast_ticks_since_slow >= CONTROL_TICKS_PER_SLOW) {
        fast_ticks_since_slow = 0;
        return 1;
    }
    return 0;
}

/**
 * @brief Boucle lente (Basse fréquence).
 * Gère le générateur de trajectoire et l'asservissement PID.
 */
void control_slow_step(uint32_t au_state, control_outputs_t *out)
{
    // 1. Récupération de la position estimée actuelle
    ctrl_hal_timing_start(CTRL_T_SLOW_MOTION);
    Position current = {
        kalman_current_state.x[0],
        kalman_current_state.x[1],
        kalman_current_state.x[2]
    };

    // 2. Moyennage de l'odométrie lente et calcul des consignes
    odometry_finish_slow(CONTROL_TICKS_PER_SLOW);
    motion_step(&current);
    ctrl_hal_timing_stop(CTRL_T_SLOW_MOTION);

    // 3. Application des contraintes physiques (rampement et saturation)
    ctrl_hal_timing_start(CTRL_T_SLOW_CONSTRAIN);
    constrain_speed_order();
    constrain_acceleration_order(CONTROL_SLOW_PERIOD_MS * 0.001f);
    ctrl_hal_timing_stop(CTRL_T_SLOW_CONSTRAIN);

    // 4. Calcul des puissances moteurs (Asservissement PID)
    ctrl_hal_timing_start(CTRL_T_SLOW_PID);
    ESC_Command command;
    Asserv_PWM_calculator(&command);

    // 5. Écrasement manuel si la dérogation est active
    if (ctx.manual_esc_enabled) {
        command = ctx.manual_esc;
    }

    // 6. Saturation finale des commandes
    out->motors[0] = saturate_motor_command(command.command1);
    out->motors[1] = saturate_motor_command(command.command2);
    out->motors[2] = saturate_motor_command(command.command3);
    out->motors[3] = saturate_motor_command(command.command4);

    // 7. SÉCURITÉ : Coupure (Arrêt d'Urgence)
    if (au_state) {
        out->motors[0] = out->motors[1] = out->motors[2] = out->motors[3] = 0;
        pid_vitesse_reset(); // Évite l'accumulation de l'erreur intégrale
    }
    ctrl_hal_timing_stop(CTRL_T_SLOW_PID);

    // 8. Télémétrie
    out->pose                    = current;
    out->speed_robot             = speed_robot_asserv;
    out->speed_order_constrained = speed_order_constrained;
}

void control_get_loc_init(uint32_t *state, uint32_t *progress)
{
    *state    = heading.state;
    *progress = heading.progress;
}

/* ================================================================== *
 * Initialisation
 * ================================================================== */

void control_init(uint32_t imu_yaw_seq)
{
    ctx = (control_context_t){0};
    fast_ticks_since_slow = 0;

    // Configuration par défaut du filtre de Kalman
    ctx.kalman_enable.enable_lidar_kalman = 1;
    for (int i = 0; i < 3; i++) {
        ctx.lidar_noise[i] = kalman_params.lidar[i] * kalman_params.lidar[i];
    }

    // Initialisation des sous-systèmes
    odometry_init();
    motion_init();
    speed_constrainer_init();
    acceleration_constrainer_init();
    pid_vitesse_init();
    pid_vitesse_reset();

    kalman_fifo_init(&kalman_fifo);
    kalman_init(&kalman_current_state);

    heading_fusion_init(imu_yaw_seq);
}
