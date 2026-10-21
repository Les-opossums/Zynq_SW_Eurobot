#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

/* ------------------------------------------------------------------------
 * Sequencement de l'asservissement, independant de la plateforme.
 *
 * L'appelant (adaptateur Zynq asserv_loop.c, ou simulateur PC) :
 *   1. control_init() une fois ;
 *   2. a chaque tour : control_apply_commands() avec les consignes et
 *      observations recues depuis le tour precedent ;
 *   3. tous les CONTROL_FAST_PERIOD_MS : control_fast_step() avec les
 *      mesures capteurs, puis, quand il renvoie 1, control_slow_step() qui
 *      calcule les commandes moteur a envoyer.
 * Tout l'etat est interne (variables globales des modules) : une seule
 * instance d'asservissement par processus.
 * ------------------------------------------------------------------------ */

#include <stdint.h>
#include "control_types.h"
#include "asserv_default.h"

#define CONTROL_FAST_PERIOD_MS  ODO_EVERY_MS                          /* 1 ms  */
#define CONTROL_SLOW_PERIOD_MS  (ASSERV_EVERY * ODO_EVERY_MS)         /* 10 ms */
#define CONTROL_TICKS_PER_SLOW  (CONTROL_SLOW_PERIOD_MS / CONTROL_FAST_PERIOD_MS)

/* Etats de l'init de localisation (fusion cap IMU <-> lidar). Memes valeurs
 * que LOC_INIT_* (IPC_structure.h), consommees par le bandeau LED de CORE0. */
#define CONTROL_LOC_INIT_IDLE     0u  /* rien en cours (boot, avant tout relachement d'AU) */
#define CONTROL_LOC_INIT_RUNNING  1u  /* moyennage de l'offset en cours */
#define CONTROL_LOC_INIT_READY    2u  /* offset fige, robot pret, en attente du depart */
#define CONTROL_LOC_INIT_DONE     3u  /* laisse tiree, match lance */

/* Consignes et observations recues. has_* = 1 si une nouvelle valeur est
 * arrivee depuis le dernier appel (le champ associe est alors lu). */
typedef struct {
    uint8_t has_cmd_position;      Position        cmd_position;
    uint8_t has_cmd_speed;         Speed           cmd_speed;
    uint8_t has_cmd_abs_speed;     Speed           cmd_abs_speed;
    uint8_t has_asserv_mode;       int             asserv_mode;   /* 0 : libre, 4 : bloque */
    uint8_t has_set_pos;           Position        set_pos;       /* recalage odometrie + Kalman */
    uint8_t has_vmax;              float           vmax;
    uint8_t has_vtmax;             float           vtmax;
    uint8_t has_amax;              float           amax;
    uint8_t has_odo_spacing;       float           odo_spacing;
    uint8_t has_cmd_esc;           ESC_Command     cmd_esc;       /* commande moteur manuelle */
    uint8_t has_enable_kalman;     Enable_Kalman   enable_kalman;
    uint8_t has_kalman_noise_lidar; Set_lidar_noise kalman_noise_lidar;
    uint8_t has_set_lidar;         Set_lidar       set_lidar;
    uint8_t has_set_camera[3];     Set_camera      set_camera[3];
} control_commands_t;

/* Mesures capteurs lues a chaque pas rapide. */
typedef struct {
    int      motor_rpm[4];      /* vitesse des moteurs (retour ESC), tr/min cote moteur */
    uint32_t imu_seq;           /* compteur de mise a jour du gyro */
    uint8_t  imu_seq_stable;    /* 1 si imu_seq n'a pas change pendant la lecture (pas de lecture dechiree) */
    float    imu_gyro_z;        /* rad/s */
    uint32_t imu_calib_status;  /* >= 1 : gyro utilisable */
    float    imu_yaw;           /* cap absolu IMU (repere IMU), rad */
    uint32_t imu_yaw_seq;       /* incremente a chaque nouvelle mesure de cap ; 0 = jamais recu */
    uint32_t au_state;          /* 1 : arret d'urgence appuye */
    uint32_t leash_state;       /* 1 : laisse tiree (depart du match) */
} control_inputs_t;

/* Resultat d'un pas lent. */
typedef struct {
    int16_t  motors[4];               /* commandes ESC saturees (0 si AU) */
    Position pose;                    /* pose estimee (Kalman) utilisee par ce pas */
    Speed    speed_robot;             /* vitesse odometrique moyennee (repere robot) */
    Speed    speed_order_constrained; /* consigne apres rampes (repere robot) */
} control_outputs_t;

/* imu_yaw_seq : valeur courante du compteur de cap, pour ne pas rejouer une
 * mesure deja presente au demarrage. */
void control_init(uint32_t imu_yaw_seq);

/* imu_yaw / imu_yaw_seq : dernier cap IMU connu, utilise pour moyenner
 * l'offset IMU <-> monde a chaque mesure lidar. */
void control_apply_commands(const control_commands_t *cmd, float imu_yaw, uint32_t imu_yaw_seq);

/* 1 si le filtre a deja ete initialise (par set_pos ou une premiere mesure
 * lidar). Tant que ce n'est pas le cas, les observations camera sont ignorees :
 * l'adaptateur Zynq les laisse alors en attente dans l'IPC. */
uint8_t control_kalman_initialized(void);

/* Pas rapide (CONTROL_FAST_PERIOD_MS). Renvoie 1 quand un pas lent est du. */
uint8_t control_fast_step(const control_inputs_t *in);

/* Pas lent (CONTROL_SLOW_PERIOD_MS). */
void control_slow_step(uint32_t au_state, control_outputs_t *out);

/* Echantillon du log de calibration (accumulateurs mis a jour a chaque pas
 * rapide, mesure lidar a chaque SETLIDAR). A emettre periodiquement (100 Hz). */
void control_get_calib_sample(Calib_sample *out);

/* Etat de l'init de localisation (CONTROL_LOC_INIT_*) et avancement 0..100 %. */
void control_get_loc_init(uint32_t *state, uint32_t *progress);

#endif /* CONTROL_LOOP_H */
