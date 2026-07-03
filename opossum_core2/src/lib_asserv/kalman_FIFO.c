#include "../main.h"
#include "lib_asserv.h"

KalmanFIFO kalman_fifo;

void kalman_fifo_init(KalmanFIFO* fifo) {
    memset(fifo, 0, sizeof(KalmanFIFO));
    fifo->head  = 0;
    fifo->count = 0;

    KalmanState default_state;
    kalman_init(&default_state);

    for (int i = 0; i < KALMAN_FIFO_LEN; i++) {
        fifo->buffer[i] = default_state;

        fifo->speed_robot[i].vx = 0.0f;
        fifo->speed_robot[i].vy = 0.0f;
        fifo->speed_robot[i].vt = 0.0f;

        fifo->observations[i].has_lidar              = 0;
        fifo->observations[i].bypass_lidar_rejection = 0;
        fifo->observations[i].z_lidar[0] = 0.0f;
        fifo->observations[i].z_lidar[1] = 0.0f;
        fifo->observations[i].z_lidar[2] = 0.0f;

        for (int cam_id = 0; cam_id < 3; cam_id++) {
            fifo->observations[i].has_camera[cam_id] = 0;
            fifo->observations[i].z_camera[cam_id][0] = 0.0f;
            fifo->observations[i].z_camera[cam_id][1] = 0.0f;
            fifo->observations[i].z_camera[cam_id][2] = 0.0f;

            fifo->observations[i].r_camera[cam_id][0] = OBS_NOISE_CAMERA_XY * OBS_NOISE_CAMERA_XY;
            fifo->observations[i].r_camera[cam_id][1] = OBS_NOISE_CAMERA_XY * OBS_NOISE_CAMERA_XY;
            fifo->observations[i].r_camera[cam_id][2] = OBS_NOISE_CAMERA_THETA * OBS_NOISE_CAMERA_THETA;
        }
    }
}

void kalman_fifo_push(KalmanFIFO* fifo, KalmanState* state, Speed* speed_robot, uint8_t has_imu, float imu_vtheta) {
    // Stocke l'état à l'emplacement courant
    memcpy(&fifo->buffer[fifo->head], state, sizeof(KalmanState));

    // Stocke la vitesse du robot à l'emplacement courant
    memcpy(&fifo->speed_robot[fifo->head], speed_robot, sizeof(Speed));

    // Stocke les données IMU à l'emplacement courant
    fifo->has_imu[fifo->head] = has_imu;
    fifo->z_imu_vtheta[fifo->head] = imu_vtheta;

    fifo->observations[fifo->head].has_lidar = 0; // Par défaut, pas d'observation associée à ce nouvel état
    fifo->observations[fifo->head].bypass_lidar_rejection = 0; // Par défaut, pas de bypass

    for(int cam_id = 0; cam_id < 3; cam_id++) {
        fifo->observations[fifo->head].has_camera[cam_id] = 0; // Par défaut, pas d'observation associée à ce nouvel état
    }
    
    // Incrémente la tête de la FIFO en la ramenant dans les bornes
    fifo->head = (fifo->head + 1) % KALMAN_FIFO_LEN;

    // Incrémente le compteur d'éléments    
    if (fifo->count < KALMAN_FIFO_LEN) {
        fifo->count++;
    }
}

int kalman_fifo_get_delay(KalmanFIFO* fifo, int delay_ms, float dt_ms) {
    int samples_back = (int)(delay_ms / dt_ms); 
    if (fifo->count < samples_back) {
        // printf("WARNING: FIFO not enough samples\n");
        return -1; // Erreur : pas assez d'échantillons dans la FIFO
    }

    int index = fifo->head - samples_back - 1;
    if (index < 0) index += KALMAN_FIFO_LEN;

    return index;
}

void kalman_fifo_repropagate(KalmanFIFO* fifo, int delay_index, float dt_s, float R_lidar[3]) {
    int i = delay_index;
    int last = (fifo->head - 1 + KALMAN_FIFO_LEN) % KALMAN_FIFO_LEN;

    // On repropague en boucle circulaire jusqu'à la tête-1
    while (i != last) {
        int next_i = (i + 1) % KALMAN_FIFO_LEN;

        // utilisation direct de l’état précédent, sans copie complète
        KalmanState* current = &fifo->buffer[i];
        KalmanState* next = &fifo->buffer[next_i];

        Speed* speed = &fifo->speed_robot[next_i];

         // Calcul prédictif sur l’état "next" basé sur "current"
        memcpy(next, current, sizeof(KalmanState)); // Copie une fois

        // 1. Prédiction odométrique classique
        kalman_predict(next, dt_s);

        // 2. Si on avait lu l'IMU à ce moment-là dans le passé, on réapplique la mesure !
        if (fifo->has_imu[next_i]) {
            kalman_update_imu(next, fifo->z_imu_vtheta[next_i]);
        }

        // 3. Correction EKF avec la mesure d'odométrie 
        kalman_update_odo(next, speed);

        // 4. Si une mesure Lidar avait eu lieu à 'next_i', on la réapplique !
        if (fifo->observations[next_i].has_lidar) {
            kalman_update(next, fifo->observations[next_i].z_lidar, R_lidar, fifo->observations[next_i].bypass_lidar_rejection);
        }

        // 5. Si une mesure Caméra avait eu lieu à 'next_i', on la réapplique ! 
        for(int cam_id = 0; cam_id < 3; cam_id++) {
            if (fifo->observations[next_i].has_camera[cam_id]) {
                kalman_update(next, 
                              fifo->observations[next_i].z_camera[cam_id], 
                              fifo->observations[next_i].r_camera[cam_id], 
                              fifo->observations[next_i].bypass_camera_rejection[cam_id]); // Fini le '1' en dur
            }
        }

        i = next_i;
    }
    
    // Après propagation, mettre à jour l’état courant
    kalman_current_state = fifo->buffer[(fifo->head - 1 + KALMAN_FIFO_LEN) % KALMAN_FIFO_LEN];
}

void kalman_init_with_lidar(KalmanFIFO* fifo, Position* lidar_pos) {
    KalmanState init_state;
    memset(&init_state, 0, sizeof(KalmanState)); // Initialisation à 0 pour éviter les valeurs indéterminées

    // Initialiser la position
    init_state.x[0] = lidar_pos->x;
    init_state.x[1] = lidar_pos->y;
    init_state.x[2] = principal_angle(lidar_pos->t);

    // Initialiser les vitesses à 0 (ou valeurs par défaut)
    init_state.x[3] = 0.0f; // vx
    init_state.x[4] = 0.0f; // vy
    init_state.x[5] = 0.0f; // vtheta

    // Initialiser la matrice de covariance P (confiance initiale)
    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < STATE_SIZE; j++) {
            init_state.P[i][j] = 0.0f;
        }
        init_state.P[i][i] = 0.01f;  // petite incertitude initiale
    }

    kalman_current_state = init_state;

    // Initialiser la FIFO : on remplit tout avec cet état initial
    for (int i = 0; i < KALMAN_FIFO_LEN; i++) {
        fifo->buffer[i] = init_state;
        fifo->speed_robot[i].vx = 0.0f;
        fifo->speed_robot[i].vy = 0.0f;
        fifo->speed_robot[i].vt = 0.0f;

        fifo->observations[i].has_lidar              = 0;
        fifo->observations[i].bypass_lidar_rejection = 0;
    }
    fifo->head = 0;
    fifo->count = 0;
}

uint8_t lidar_consecutive_rejections = 0;

// Retourne l'index du slot, ou -1 si invalide
int kalman_fifo_insert_lidar(KalmanFIFO* fifo, Set_lidar* data, float R_lidar[3]) {
    if (data->delay < 0 || data->delay > 200) return -1;

    int idx = kalman_fifo_get_delay(fifo, data->delay, ODO_EVERY_MS);
    if (idx < 0) return -1;

    fifo->observations[idx].has_lidar = 1;
    fifo->observations[idx].bypass_lidar_rejection = (lidar_consecutive_rejections > 10);
    fifo->observations[idx].z_lidar[0] = data->lidar_position_x;
    fifo->observations[idx].z_lidar[1] = data->lidar_position_y;
    fifo->observations[idx].z_lidar[2] = data->lidar_position_t;

    // Update initial sur le slot historique (point de départ de la repropagate)
    float z[3] = {data->lidar_position_x, data->lidar_position_y, data->lidar_position_t};
    uint8_t result = kalman_update(&fifo->buffer[idx], z, R_lidar,
                                     fifo->observations[idx].bypass_lidar_rejection);
    if (result == 1) {          // rejeté outlier
        lidar_consecutive_rejections++;
    } else if (result == 0) {   // accepté
        lidar_consecutive_rejections = 0;
    }

    return idx;
}

uint8_t camera_consecutive_rejections[3] = {0, 0, 0};

int kalman_fifo_insert_camera(KalmanFIFO* fifo, Set_camera* data, uint8_t cam_id) {
    if (cam_id >= 3) return -1; // 0-indexé !
    if (data->delay < 0 || data->delay > 200) return -1;

    int idx = kalman_fifo_get_delay(fifo, data->delay, ODO_EVERY_MS);
    if (idx < 0) return -1;

    float R[3] = {data->noise_x * data->noise_x,
                  data->noise_y * data->noise_y,
                  data->noise_t * data->noise_t};

    fifo->observations[idx].has_camera[cam_id] = 1;
    fifo->observations[idx].z_camera[cam_id][0] = data->camera_position_x;
    fifo->observations[idx].z_camera[cam_id][1] = data->camera_position_y;
    fifo->observations[idx].z_camera[cam_id][2] = data->camera_position_t;

    fifo->observations[idx].r_camera[cam_id][0] = R[0];
    fifo->observations[idx].r_camera[cam_id][1] = R[1];
    fifo->observations[idx].r_camera[cam_id][2] = R[2];


    // Si on a rejeté la caméra trop de fois consécutivement
    // on force son acceptation pour recaler le robot
    if (camera_consecutive_rejections[cam_id] > 50) {
        fifo->observations[idx].bypass_camera_rejection[cam_id] = 1;
    } else {
        fifo->observations[idx].bypass_camera_rejection[cam_id] = 0;
    }

    // Vecteur de mesure pour le test
    float z_test[3] = {data->camera_position_x, data->camera_position_y, data->camera_position_t};
    
    // Test de l'update pour évaluer l'écart (l'état modifié ici sera écrasé lors de la vraie repropagation)
    uint8_t result = kalman_update(&fifo->buffer[idx], z_test, R,
                                     fifo->observations[idx].bypass_camera_rejection[cam_id]);

    // On suit la même logique que ton lidar : accepted == 1 signifie que la mesure a été rejetée par le filtre
    if (result == 1) {
        camera_consecutive_rejections[cam_id]++;
    } else if (result == 0) {
        camera_consecutive_rejections[cam_id] = 0;
    }
    return idx;
}