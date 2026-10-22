#ifndef __KALMAN_H_
#define __KALMAN_H_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include "control_types.h"

#define STATE_SIZE 6  // x, y, theta

static inline float principal_angle(float angle)
{
    while (angle > 3.14159265359f) angle -= 6.28318530718f;
    while (angle < -3.14159265359f) angle += 6.28318530718f;
    return angle;
}

// ============================================================================
// MATRICE Q : BRUIT DE PROCESSUS (CONFIANCE EN LA PRÉDICTION / KINÉMATIQUE)
// ============================================================================

// 1. Bruit de base (Robot à l'arrêt absolu)
// Valeurs infimes pour garantir que la matrice P reste inversible et définie positive.
#define PROCESS_NOISE_BASE_X      0.0001f // m par pas de 1 ms (soit 0.1 mm)
#define PROCESS_NOISE_BASE_Y      0.0001f // m par pas de 1 ms
#define PROCESS_NOISE_BASE_THETA  0.001f  // rad par pas de 1 ms

// 2. Facteur de glissement dynamique (Robot en mouvement)
// Les roues holonomes avec des M2006 patinent lors de fortes accélérations.
// 0.02f = 2% de l'odométrie est considérée comme de l'incertitude pure.
#define PROCESS_NOISE_VEL_X       0.02f   
#define PROCESS_NOISE_VEL_Y       0.02f   
#define PROCESS_NOISE_VEL_THETA   0.01f   // La rotation glisse souvent plus que la translation

// 3. Incertitude sur le modèle de vitesse (Modèle à vitesse constante)
// Les M2006 peuvent changer de vitesse brutalement. Il faut un Q_vitesse assez 
// élevé pour autoriser le filtre à suivre ces fortes accélérations sans trop de retard.
#define PROCESS_NOISE_VX          0.20f   // m/s
#define PROCESS_NOISE_VY          0.20f   // m/s
#define PROCESS_NOISE_VTHETA      0.50f   // rad/s

// ============================================================================
// MATRICE R : BRUIT DE MESURE (CONFIANCE EN LES CAPTEURS)
// ============================================================================

// 1. Odométrie interne (Encodeurs des M2006)
// Utilisé dans kalman_update_odo. Écart-type en m/s.
#define OBS_NOISE_ODO_VX          0.05f   
#define OBS_NOISE_ODO_VY          0.05f   
#define OBS_NOISE_ODO_VTHETA      0.05f   

// 2. Lidar (Triangulation via Raspberry Pi)
// Le Lidar est très précis, mais la triangulation peut osciller de quelques cm.
#define OBS_NOISE_LIDAR_X         0.02f   // 2 cm d'écart-type
#define OBS_NOISE_LIDAR_Y         0.02f   
#define OBS_NOISE_LIDAR_THETA     0.02f   

// 3. Caméra JeVois Pro
// Susceptible au flou de mouvement et aux imprécisions de détection des balises.
#define OBS_NOISE_CAMERA_XY       0.08f   // 8 cm d'écart-type
#define OBS_NOISE_CAMERA_THETA    0.15f

// 4. IMU (Inertial Measurement Unit)
// Le capteur IMU fournit des mesures d'angle et de vitesse angulaire.
#define OBS_NOISE_IMU_VTHETA      0.02f    // 0.1 deg/s d'écart-type

// 5. IMU - orientation absolue (cap Rotation Vector / Game Rotation Vector
// recale sur le repere monde via l'offset lidar). Ecart-type en rad : le BNO085
// donne un cap tres stable, ~1 deg d'incertitude typique -> ~0.02 rad.
#define OBS_NOISE_IMU_THETA       0.02f    // ~1.1 deg d'écart-type

#define S_INV_EPS 1e-6f       // regularisation ajoutee a la diagonale de S si besoin (variance)
#define S_INV_REL_EPS 1e-6f   // S jugee singuliere si |det| < S_INV_REL_EPS * S00*S11*S22

// ============================================================================
// PARAMETRES REGLABLES A L'EXECUTION
// ============================================================================
// Les #define ci-dessus sont les valeurs par defaut. Le filtre lit les valeurs
// courantes dans kalman_params (ecarts-types, memes unites que les #define),
// modifiables avec kalman_set_params() : reglage en simulation (SIL, optimiseur)
// ou plus tard par commande. Le bruit lidar reste fixe par LIDARNOISE
// (control_loop), lidar_* ne sert que de valeur initiale.
typedef struct {
    float q_base[3];      // PROCESS_NOISE_BASE_X / Y / THETA   (par pas de 1 ms)
    float q_vel[3];       // PROCESS_NOISE_VEL_X / Y / THETA    (x |vitesse|, par pas)
    float q_speed[3];     // PROCESS_NOISE_VX / VY / VTHETA
    float odo[3];         // OBS_NOISE_ODO_VX / VY / VTHETA
    float lidar[3];       // OBS_NOISE_LIDAR_X / Y / THETA      (valeur initiale)
    float camera_xy;      // OBS_NOISE_CAMERA_XY                (camera sans ecart-type annonce)
    float camera_theta;   // OBS_NOISE_CAMERA_THETA
    float imu_vtheta;     // OBS_NOISE_IMU_VTHETA
    float imu_theta;      // OBS_NOISE_IMU_THETA
} KalmanParams;

#define KALMAN_PARAMS_DEFAULT { \
    {PROCESS_NOISE_BASE_X, PROCESS_NOISE_BASE_Y, PROCESS_NOISE_BASE_THETA}, \
    {PROCESS_NOISE_VEL_X,  PROCESS_NOISE_VEL_Y,  PROCESS_NOISE_VEL_THETA}, \
    {PROCESS_NOISE_VX,     PROCESS_NOISE_VY,     PROCESS_NOISE_VTHETA}, \
    {OBS_NOISE_ODO_VX,     OBS_NOISE_ODO_VY,     OBS_NOISE_ODO_VTHETA}, \
    {OBS_NOISE_LIDAR_X,    OBS_NOISE_LIDAR_Y,    OBS_NOISE_LIDAR_THETA}, \
    OBS_NOISE_CAMERA_XY, OBS_NOISE_CAMERA_THETA, \
    OBS_NOISE_IMU_VTHETA, OBS_NOISE_IMU_THETA }

extern KalmanParams kalman_params;

/** Remplit p avec les valeurs par defaut (#define). */
void kalman_params_default(KalmanParams* p);

/** Applique p (copie dans kalman_params et recalcule les variances). */
void kalman_set_params(const KalmanParams* p);


typedef struct {
    float x[STATE_SIZE] __attribute__((aligned(16)));                 // état X[x, y, theta, vx, vy, vtheta]
    float P[STATE_SIZE][STATE_SIZE] __attribute__((aligned(16)));     // covariance
} KalmanState;

extern KalmanState kalman_current_state;

/**
 * Initialise l’état du filtre de Kalman.
 * 
 * @param state L’état à initialiser.
 */
void kalman_init(KalmanState* state);

/**
 * Applique la prédiction du modèle EKF à partir des vitesses dans le repère robot.
 * 
 * @param state L’état courant à prédire.
 * @param dt Pas de temps (s).
 */
void kalman_predict(KalmanState* state, float dt);

/**
 * Applique la correction EKF à partir d’une mesure z = [x, y, theta] dans le repère monde.
 * 
 * @param state L’état à corriger (potentiellement un état passé issu du FIFO).
 * @param z Mesure du LiDAR : position et angle absolus.
 * @param R_diag Diagonale de la matrice de bruit de mesure. 3 éléments : [R_x, R_y, R_theta].
 * @param bypass_outlier_rejection Si à 1, bypass la rejection d’outliers basée sur la distance de Mahalanobis.
 * 
 * @return Plusieurs returns possibles :
 *     - 0 : update réussi
 *     - 1 : erreur de mesure : mahalanobis trop grand (outlier)
 *     - 2 : erreur d’état (NaN)
 *     - 3 : matrice S singulière (non inversible)
 * 
 * @note plusieurs returns possibles :
 *      - 1 : erreur de mesure (NaN)
 *      - 2 : erreur d’état (NaN)
 *      - 3 : matrice S singulière (non inversible)
 *      - 4 : Clamp de sécurité post update
 *      - 5 : Clamp de sécurité post update
 */
uint8_t kalman_update(KalmanState* state, float z[3], float R_diag[3], uint8_t bypass_outlier_rejection);

/**
 * @brief Update spécifique pour l'odométrie interne (encodeurs des M2006), qui observe les vitesses (vx, vy, vtheta).
 * 
 * @param state État courant du Kalman
 * @param measured_speed Vitesses mesurées par les encodeurs (en m/s et rad/s)
 * @return uint8_t 
 */
uint8_t kalman_update_odo(KalmanState* state, Speed* measured_speed);


/**
 * @brief Update spécifique pour l'IMU, qui observe la vitesse angulaire (vtheta).
 * 
 * @param state État courant du Kalman
 * @param measured_vtheta Vitesse angulaire Z mesurée par l'IMU (en rad/s)
 * @return uint8_t 
 */
uint8_t kalman_update_imu(KalmanState* state, float measured_vtheta);


/**
 * @brief Update specifique pour l'orientation absolue de l'IMU, qui observe le
 * cap theta (x[2]).
 *
 * La mesure est le cap absolu de l'IMU (Rotation Vector ou Game Rotation Vector)
 * deja recale sur le repere monde par l'offset moyenne sur les mesures lidar
 * (cf fusion heading dans asserv_loop.c). L'innovation est ramenee dans
 * [-pi, pi] (principal_angle) pour gerer le repliement d'angle.
 *
 * @param state          Etat courant du Kalman
 * @param measured_theta Cap absolu mesure (repere monde, rad)
 * @return uint8_t  0 : ok, 3 : matrice S singuliere
 */
uint8_t kalman_update_imu_theta(KalmanState* state, float measured_theta);

#endif // __KALMAN_H_
