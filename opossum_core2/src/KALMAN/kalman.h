#ifndef __KALMAN_H_
#define __KALMAN_H_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include "../../../opossum_common/common_type.h"

#define STATE_SIZE 6  // x, y, theta
#define HISTORY_LEN 500  // pour 200 ms à 1 kHz

#define LIDAR_DELAY 90 // 100 ms

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

#define S_INV_EPS 1e-6f


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
uint8_t kalman_update(KalmanState* state, float z[STATE_SIZE], float R_diag[3], uint8_t bypass_outlier_rejection);

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

#endif // __KALMAN_H_
