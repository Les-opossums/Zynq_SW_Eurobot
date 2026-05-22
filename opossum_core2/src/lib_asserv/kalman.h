#ifndef __KALMAN_H_
#define __KALMAN_H_

#define STATE_SIZE 6  // x, y, theta
#define HISTORY_LEN 200  // pour 200 ms à 1 kHz

#define LIDAR_DELAY 90 // 100 ms

#define PROCESS_NOISE_ODOM_X 0.02f
#define PROCESS_NOISE_ODOM_Y 0.02f
#define PROCESS_NOISE_ODOM_THETA 0.03f

// 1. Bruit de base (Robot à l'arrêt absolu)
// Valeurs infimes, juste pour éviter que la matrice P ne devienne numériquement nulle
#define PROCESS_NOISE_ODOM_BASE_X      0.0001f // m par pas de temps
#define PROCESS_NOISE_ODOM_BASE_Y      0.0001f
#define PROCESS_NOISE_ODOM_BASE_THETA  0.005f // rad par pas de temps

// 2. Facteur de patinage (Robot en mouvement)
// Représente le pourcentage d'erreur généré par la vitesse.
// Ex: 0.05f signifie que 5% de la vitesse se transforme en incertitude de position.
#define PROCESS_NOISE_ODOM_VEL_X       0.01f   // était 0.05
#define PROCESS_NOISE_ODOM_VEL_Y       0.01f   // était 0.05
#define PROCESS_NOISE_ODOM_VEL_THETA   0.02f   // était 0.05

#define PROCESS_NOISE_ODOM_VX       0.02f
#define PROCESS_NOISE_ODOM_VY       0.02f
#define PROCESS_NOISE_ODOM_VTHETA   0.05f

#define PROCESS_NOISE_LIDAR_X       0.03f
#define PROCESS_NOISE_LIDAR_Y       0.03f
#define PROCESS_NOISE_LIDAR_THETA   0.03f

#define R_CAMERA_MIN_XY 0.1f 
#define R_CAMERA_MIN_T 0.3f  

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
 * @param speed Vitesse du robot dans le repère robot.
 * @param dt Pas de temps (s).
 */
void kalman_predict(KalmanState* state, Speed* speed, float dt);

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

#endif // __KALMAN_H_