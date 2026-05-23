#include "../main.h"
#include "lib_asserv.h"


KalmanState kalman_current_state;

void kalman_init(KalmanState* state) {
    memset(state->x, 0, sizeof(state->x));
    memset(state->P, 0, sizeof(state->P));

    state->P[0][0] = 1.0f;                     
    state->P[1][1] = 1.0f;
    state->P[2][2] = 1.0f;                     // theta: ±10°
    state->P[3][3] = 1.0f;                     // vx: ±0.2 m/s
    state->P[4][4] = 1.0f;                     // vy: ±0.2 m/s
    state->P[5][5] = 1.0f;                     // vtheta: ±30°/s
}

void kalman_predict(KalmanState* state, float dt) {
    // 1. --- Récupération de l'état actuel ---
    float x = state->x[0];
    float y = state->x[1];
    float theta = state->x[2];
    
    // Les vitesses sont désormais celles estimées par le filtre, dans le REPÈRE ROBOT
    float vx = state->x[3]; 
    float vy = state->x[4]; 
    float vtheta = state->x[5];

    // --- Modèle cinématique (Vitesse Constante) ---
    // Utilisation de Runge-Kutta 2 (angle au milieu du pas) pour plus de précision
    float angle_mid = principal_angle(theta + vtheta * dt * 0.5f);
    float cos_t = cosf(angle_mid);
    float sin_t = sinf(angle_mid);

    // Transformation des vitesses robot -> monde pour la mise à jour de la position
    float dx = (vx * cos_t - vy * sin_t) * dt;
    float dy = (vx * sin_t + vy * cos_t) * dt;

    // Mise à jour de l'état prédit
    state->x[0] = x + dx;
    state->x[1] = y + dy;
    state->x[2] = principal_angle(theta + vtheta * dt);
    // state->x[3], x[4], x[5] restent inchangés (hypothèse de vitesse constante)

    // 2. --- Matrice Jacobienne F ---
    // Dérivées partielles de la position par rapport à l'angle (theta)
    float F02 = (-vx * sin_t - vy * cos_t) * dt;
    float F12 = ( vx * cos_t - vy * sin_t) * dt;

    // Dérivées partielles de la position par rapport aux vitesses (vx, vy)
    float F03 = cos_t * dt;
    float F04 = -sin_t * dt;
    float F13 = sin_t * dt;
    float F14 = cos_t * dt;

    // 3. --- Propagation de la covariance : P_new = F * P * F^T ---
    // Optimisation extrême pour architecture ARM : on calcule F*P et F*P*F^T de manière explicite
    // en ignorant toutes les multiplications par 0 ou 1 de la matrice F.
    
    float P[6][6];
    memcpy(P, state->P, sizeof(P)); // Copie locale pour lire les valeurs de l'étape k-1

    float FP[6][6];
    // Étape A : FP = F * P
    for (int j = 0; j < 6; j++) {
        FP[0][j] = P[0][j] + F02 * P[2][j] + F03 * P[3][j] + F04 * P[4][j];
        FP[1][j] = P[1][j] + F12 * P[2][j] + F13 * P[3][j] + F14 * P[4][j];
        FP[2][j] = P[2][j] + dt * P[5][j];
        FP[3][j] = P[3][j];
        FP[4][j] = P[4][j];
        FP[5][j] = P[5][j];
    }

    float P_new[6][6];
    // Étape B : P_new = FP * F^T
    for (int i = 0; i < 6; i++) {
        P_new[i][0] = FP[i][0] + FP[i][2] * F02 + FP[i][3] * F03 + FP[i][4] * F04;
        P_new[i][1] = FP[i][1] + FP[i][2] * F12 + FP[i][3] * F13 + FP[i][4] * F14;
        P_new[i][2] = FP[i][2] + FP[i][5] * dt;
        P_new[i][3] = FP[i][3];
        P_new[i][4] = FP[i][4];
        P_new[i][5] = FP[i][5];
    }

    // 4. --- Bruit de processus Q dynamique ---
    // Basé sur les vitesses estimées par le filtre
    float abs_vx = fabsf(vx);
    float abs_vy = fabsf(vy);
    float abs_vt = fabsf(vtheta);

    float q_var_x  = powf(PROCESS_NOISE_BASE_X     + PROCESS_NOISE_VEL_X     * abs_vx, 2.0f);
    float q_var_y  = powf(PROCESS_NOISE_BASE_Y     + PROCESS_NOISE_VEL_Y     * abs_vy, 2.0f);
    float q_var_t  = powf(PROCESS_NOISE_BASE_THETA + PROCESS_NOISE_VEL_THETA * abs_vt, 2.0f);
    
    float q_var_vx = PROCESS_NOISE_VX     * PROCESS_NOISE_VX;
    float q_var_vy = PROCESS_NOISE_VY     * PROCESS_NOISE_VY;
    float q_var_vt = PROCESS_NOISE_VTHETA * PROCESS_NOISE_VTHETA;

    // Ajout du bruit Q sur la diagonale (P_new = F*P*F^T + Q)
    P_new[0][0] += q_var_x;
    P_new[1][1] += q_var_y;
    P_new[2][2] += q_var_t;
    P_new[3][3] += q_var_vx;
    P_new[4][4] += q_var_vy;
    P_new[5][5] += q_var_vt;

    // 5. --- Symétrisation pour éviter les dérives numériques ---
    for (int i = 0; i < 6; ++i) {
        for (int j = i; j < 6; ++j) {
            float s = 0.5f * (P_new[i][j] + P_new[j][i]);
            state->P[i][j] = s;
            state->P[j][i] = s;
        }
    }
}

// Seuil de la loi du Chi-Carré pour 3 degrés de liberté (x, y, theta) à 99% de confiance
#define CHI2_THRESHOLD_99 11.34f

uint8_t kalman_update(KalmanState* state, float z[3], float R_diag[3], uint8_t bypass_outlier_rejection) {
    // Vérifs NaN
    for (int i = 0; i < 3; ++i) {
        if (isnan(z[i]) || isnan(state->x[i])) return 2; // erreur d’état ou de mesure
    }

    // H = [ I3  0 ] (mesure = x,y,theta)
    // innovation y = z - H x
    float y0 = z[0] - state->x[0];
    float y1 = z[1] - state->x[1];
    float y2 = principal_angle(z[2] - state->x[2]);

    // S = H P H^T + R = top-left 3x3 of P + R_diag
    float S[3][3];
    S[0][0] = state->P[0][0] + R_diag[0];
    S[0][1] = state->P[0][1];
    S[0][2] = state->P[0][2];

    S[1][0] = state->P[1][0];
    S[1][1] = state->P[1][1] + R_diag[1];
    S[1][2] = state->P[1][2];

    S[2][0] = state->P[2][0];
    S[2][1] = state->P[2][1];
    S[2][2] = state->P[2][2] + R_diag[2];

    // Inversion 3x3 de S avec régularisation numérique si besoin
    // calcul du déterminant
    float det = S[0][0]*(S[1][1]*S[2][2] - S[1][2]*S[2][1])
              - S[0][1]*(S[1][0]*S[2][2] - S[1][2]*S[2][0])
              + S[0][2]*(S[1][0]*S[2][1] - S[1][1]*S[2][0]);

    // si mal conditionné, ajouter eps sur la diagonale puis recalculer (simple fallback)
    if (fabsf(det) < S_INV_EPS) {
        S[0][0] += S_INV_EPS;
        S[1][1] += S_INV_EPS;
        S[2][2] += S_INV_EPS;
        det = S[0][0]*(S[1][1]*S[2][2] - S[1][2]*S[2][1])
            - S[0][1]*(S[1][0]*S[2][2] - S[1][2]*S[2][0])
            + S[0][2]*(S[1][0]*S[2][1] - S[1][1]*S[2][0]);
        if (fabsf(det) < S_INV_EPS) return 3; // toujours mal conditionné : abandonner update
    }

    float invDet = 1.0f / det;
    float S_inv[3][3];

    S_inv[0][0] =  (S[1][1]*S[2][2] - S[1][2]*S[2][1]) * invDet;
    S_inv[0][1] = -(S[0][1]*S[2][2] - S[0][2]*S[2][1]) * invDet;
    S_inv[0][2] =  (S[0][1]*S[1][2] - S[0][2]*S[1][1]) * invDet;

    S_inv[1][0] = -(S[1][0]*S[2][2] - S[1][2]*S[2][0]) * invDet;
    S_inv[1][1] =  (S[0][0]*S[2][2] - S[0][2]*S[2][0]) * invDet;
    S_inv[1][2] = -(S[0][0]*S[1][2] - S[0][2]*S[1][0]) * invDet;

    S_inv[2][0] =  (S[1][0]*S[2][1] - S[1][1]*S[2][0]) * invDet;
    S_inv[2][1] = -(S[0][0]*S[2][1] - S[0][1]*S[2][0]) * invDet;
    S_inv[2][2] =  (S[0][0]*S[1][1] - S[0][1]*S[1][0]) * invDet;


    // distance de Mahalanobis au carré pour rejection d'outliers
    float mahalanobis_sq = y0 * (y0*S_inv[0][0] + y1*S_inv[1][0] + y2*S_inv[2][0]) +
                           y1 * (y0*S_inv[0][1] + y1*S_inv[1][1] + y2*S_inv[2][1]) +
                           y2 * (y0*S_inv[0][2] + y1*S_inv[1][2] + y2*S_inv[2][2]);

    // Rejection d'outliers basée sur la distance de Mahalanobis : y^T S^-1 y > seuil chi2
    if (!bypass_outlier_rejection && mahalanobis_sq > CHI2_THRESHOLD_99) {
        return 1; // abandonner update
    }

    // Gain K = P * H^T * S_inv
    // H^T = [ I3; 0 ] => K rows 0..5, cols 0..2:
    float K[6][3];
    // pour i=0..5, K[i] = [ P[i][0], P[i][1], P[i][2] ] * S_inv
    for (int i = 0; i < 6; ++i) {
        // produit 1x3 = 1x3 * 3x3
        for (int j = 0; j < 3; ++j) {
            K[i][j] = state->P[i][0] * S_inv[0][j]
                    + state->P[i][1] * S_inv[1][j]
                    + state->P[i][2] * S_inv[2][j];
        }
    }

    // Mise à jour de l'état : x = x + K * y
    float dy[3] = { y0, y1, y2 };
    for (int i = 0; i < 6; ++i) {
        float delta = K[i][0]*dy[0] + K[i][1]*dy[1] + K[i][2]*dy[2];
        if (i == 2)
            state->x[i] = principal_angle(state->x[i] + delta);
        else
            state->x[i] += delta;
    }

    // Mise à jour de la covariance P via la forme Joseph :
    // P = (I - K H) P (I - K H)^T + K R K^T
    // Avec H = [I3 0], (I - K H) est une 6x6 = I6 ; les premiers 3 cols of KH are K[:,0..2] in rows 0..5
    // On calcule explicitement pour stabilité.

    // Precompute (I-KH) matrix works as:
    // (I-KH)[i][j] = -K[i][j] for j=0..2, else 1 if i==j (for j>=3), else 0
    // But better compute P_new using expanded Joseph form to avoid building large intermediates.

    float P_new[6][6];
    // Compute A = (I - K*H)
    // Then P_new = A * P * A^T + K * R * K^T
    // Because H picks top-left, A is simple. We'll compute directly with unrolled sums to reduce temporaries.

    // First compute A * P  (A is 6x6)
    // For row i, col j:
    // (A*P)[i][j] = P[i][j] - sum_{m=0..2} K[i][m] * P[m][j]
    float AP[6][6];
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            float val = state->P[i][j];
            val -= K[i][0] * state->P[0][j];
            val -= K[i][1] * state->P[1][j];
            val -= K[i][2] * state->P[2][j];
            AP[i][j] = val;
        }
    }

    // Then P_new = AP * A^T + K * R * K^T
    // Note A^T: (AP * A^T)[i][j] = AP[i][j] - sum_{m=0..2} AP[i][m]*K[j][m]
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            float val = AP[i][j];
            val -= AP[i][0] * K[j][0];
            val -= AP[i][1] * K[j][1];
            val -= AP[i][2] * K[j][2];
            P_new[i][j] = val;
        }
    }

    // Add K * R * K^T  (R is diagonal R_diag)
    // (K R K^T)[i][j] = sum_m K[i][m] * R_diag[m] * K[j][m]
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            float add = K[i][0] * R_diag[0] * K[j][0]
                      + K[i][1] * R_diag[1] * K[j][1]
                      + K[i][2] * R_diag[2] * K[j][2];
            P_new[i][j] += add;
        }
    }

    // Symétriser P_new et copier dans state->P
    for (int i = 0; i < 6; ++i) {
        for (int j = i; j < 6; ++j) {
            float s = 0.5f * (P_new[i][j] + P_new[j][i]);
            state->P[i][j] = s;
            state->P[j][i] = s;
        }
    }

    return 0; // update réussi
}

float R_odo[3] = {
    OBS_NOISE_ODO_VX * OBS_NOISE_ODO_VX, 
    OBS_NOISE_ODO_VY * OBS_NOISE_ODO_VY, 
    OBS_NOISE_ODO_VTHETA * OBS_NOISE_ODO_VTHETA
};

uint8_t kalman_update_odo(KalmanState* state, Speed* measured_speed) {
    // 1. --- Calcul de l'innovation (y = z - Hx) ---
    // Mesure z = [vx_odo, vy_odo, vtheta_odo]
    // L'état observé est la vitesse : x[3], x[4], x[5]
    float y0 = measured_speed->vx - state->x[3];
    float y1 = measured_speed->vy - state->x[4];
    float y2 = measured_speed->vt - state->x[5];

    // 2. --- Matrice de covariance de l'innovation S = H * P * H^T + R ---
    // Puisque H = [0_3x3  I_3x3], H*P*H^T sélectionne le bloc 3x3 en bas à droite de P
    float S[3][3];
    S[0][0] = state->P[3][3] + R_odo[0];
    S[0][1] = state->P[3][4];
    S[0][2] = state->P[3][5];

    S[1][0] = state->P[4][3];
    S[1][1] = state->P[4][4] + R_odo[1];
    S[1][2] = state->P[4][5];

    S[2][0] = state->P[5][3];
    S[2][1] = state->P[5][4];
    S[2][2] = state->P[5][5] + R_odo[2];

    // 3. --- Inversion de S (3x3) ---
    // Calcul du déterminant
    float det = S[0][0]*(S[1][1]*S[2][2] - S[1][2]*S[2][1])
              - S[0][1]*(S[1][0]*S[2][2] - S[1][2]*S[2][0])
              + S[0][2]*(S[1][0]*S[2][1] - S[1][1]*S[2][0]);

    // Fallback de régularisation si la matrice est singulière (copié de ta version existante)
    if (fabsf(det) < S_INV_EPS) {
        S[0][0] += S_INV_EPS;
        S[1][1] += S_INV_EPS;
        S[2][2] += S_INV_EPS;
        det = S[0][0]*(S[1][1]*S[2][2] - S[1][2]*S[2][1])
            - S[0][1]*(S[1][0]*S[2][2] - S[1][2]*S[2][0])
            + S[0][2]*(S[1][0]*S[2][1] - S[1][1]*S[2][0]);
        if (fabsf(det) < S_INV_EPS) return 3; // Abandon
    }

    float invDet = 1.0f / det;
    float S_inv[3][3];

    S_inv[0][0] =  (S[1][1]*S[2][2] - S[1][2]*S[2][1]) * invDet;
    S_inv[0][1] = -(S[0][1]*S[2][2] - S[0][2]*S[2][1]) * invDet;
    S_inv[0][2] =  (S[0][1]*S[1][2] - S[0][2]*S[1][1]) * invDet;

    S_inv[1][0] = -(S[1][0]*S[2][2] - S[1][2]*S[2][0]) * invDet;
    S_inv[1][1] =  (S[0][0]*S[2][2] - S[0][2]*S[2][0]) * invDet;
    S_inv[1][2] = -(S[0][0]*S[1][2] - S[0][2]*S[1][0]) * invDet;

    S_inv[2][0] =  (S[1][0]*S[2][1] - S[1][1]*S[2][0]) * invDet;
    S_inv[2][1] = -(S[0][0]*S[2][1] - S[0][1]*S[2][0]) * invDet;
    S_inv[2][2] =  (S[0][0]*S[1][1] - S[0][1]*S[1][0]) * invDet;

    // (Note : On ne fait pas de rejet d'outliers par distance de Mahalanobis ici,
    // car l'odométrie est supposée continue. Les rejets sont utiles pour le Lidar).

    // 4. --- Gain de Kalman K = P * H^T * S_inv ---
    // H^T sélectionne les colonnes 3, 4 et 5 de P
    float K[6][3];
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 3; ++j) {
            K[i][j] = state->P[i][3] * S_inv[0][j]
                    + state->P[i][4] * S_inv[1][j]
                    + state->P[i][5] * S_inv[2][j];
        }
    }

    // 5. --- Mise à jour de l'état : x = x + K * y ---
    float dy[3] = { y0, y1, y2 };
    for (int i = 0; i < 6; ++i) {
        float delta = K[i][0]*dy[0] + K[i][1]*dy[1] + K[i][2]*dy[2];
        if (i == 2) {
            state->x[i] = principal_angle(state->x[i] + delta);
        } else {
            state->x[i] += delta;
        }
    }

    // 6. --- Mise à jour de la covariance P via la forme de Joseph ---
    // P = (I - KH)P(I - KH)^T + KRK^T
    // Optimisation : on déroule les calculs sachant que H = [0 I]
    float P_new[6][6];
    float AP[6][6];

    // Calcul de AP = (I - K*H)*P
    // Pour H = [0 I], le bloc de gauche de (I-KH) est l'identité, le bloc de droite est -K
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            float val = state->P[i][j];
            // On soustrait la contribution de la partie droite de P (colonnes 3,4,5)
            val -= K[i][0] * state->P[3][j];
            val -= K[i][1] * state->P[4][j];
            val -= K[i][2] * state->P[5][j];
            AP[i][j] = val;
        }
    }

    // Calcul de P_new = AP * (I - K*H)^T
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            float val = AP[i][j];
            val -= AP[i][3] * K[j][0];
            val -= AP[i][4] * K[j][1];
            val -= AP[i][5] * K[j][2];
            P_new[i][j] = val;
        }
    }

    // Ajout du terme de bruit K * R * K^T
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            float add = K[i][0] * R_odo[0] * K[j][0]
                      + K[i][1] * R_odo[1] * K[j][1]
                      + K[i][2] * R_odo[2] * K[j][2];
            P_new[i][j] += add;
        }
    }

    // Symétrisation et copie finale
    for (int i = 0; i < 6; ++i) {
        for (int j = i; j < 6; ++j) {
            float s = 0.5f * (P_new[i][j] + P_new[j][i]);
            state->P[i][j] = s;
            state->P[j][i] = s;
        }
    }

    return 0; // Succès
}


float R_imu[2] = {
    OBS_NOISE_IMU_THETA * OBS_NOISE_IMU_THETA, 
    OBS_NOISE_IMU_VTHETA * OBS_NOISE_IMU_VTHETA
};

uint8_t kalman_update_imu(KalmanState* state, float measured_theta, float measured_vtheta) {
    // 1. --- Calcul de l'innovation (y = z - Hx) ---
    // Mesure z = [theta_imu, vtheta_imu]
    // L'état observé correspond aux index 2 (theta) et 5 (vtheta)
    float y0 = principal_angle(measured_theta - state->x[2]);
    float y1 = measured_vtheta - state->x[5];

    // 2. --- Matrice de covariance de l'innovation S = H * P * H^T + R ---
    // H sélectionne les lignes/colonnes 2 et 5
    float S[2][2];
    S[0][0] = state->P[2][2] + R_imu[0];
    S[0][1] = state->P[2][5];
    
    S[1][0] = state->P[5][2];
    S[1][1] = state->P[5][5] + R_imu[1];

    // 3. --- Inversion de S (2x2) ---
    float det = S[0][0]*S[1][1] - S[0][1]*S[1][0];
    
    // Fallback de régularisation si la matrice est singulière
    if (fabsf(det) < S_INV_EPS) {
        S[0][0] += S_INV_EPS;
        S[1][1] += S_INV_EPS;
        det = S[0][0]*S[1][1] - S[0][1]*S[1][0];
        if (fabsf(det) < S_INV_EPS) return 3; // Abandon
    }

    float invDet = 1.0f / det;
    float S_inv[2][2];
    S_inv[0][0] =  S[1][1] * invDet;
    S_inv[0][1] = -S[0][1] * invDet;
    S_inv[1][0] = -S[1][0] * invDet;
    S_inv[1][1] =  S[0][0] * invDet;

    // 4. --- Gain de Kalman K = P * H^T * S_inv ---
    // H^T sélectionne les colonnes 2 et 5 de P
    float K[6][2];
    for (int i = 0; i < 6; ++i) {
        K[i][0] = state->P[i][2] * S_inv[0][0] + state->P[i][5] * S_inv[1][0];
        K[i][1] = state->P[i][2] * S_inv[0][1] + state->P[i][5] * S_inv[1][1];
    }

    // 5. --- Mise à jour de l'état : x = x + K * y ---
    for (int i = 0; i < 6; ++i) {
        float delta = K[i][0]*y0 + K[i][1]*y1;
        if (i == 2) {
            state->x[i] = principal_angle(state->x[i] + delta);
        } else {
            state->x[i] += delta;
        }
    }

    // 6. --- Mise à jour de la covariance P via la forme de Joseph ---
    float P_new[6][6];
    float AP[6][6];

    // Calcul de AP = (I - K*H)*P
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            float val = state->P[i][j];
            val -= K[i][0] * state->P[2][j];
            val -= K[i][1] * state->P[5][j];
            AP[i][j] = val;
        }
    }

    // Calcul de P_new = AP * (I - K*H)^T
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            float val = AP[i][j];
            val -= AP[i][2] * K[j][0];
            val -= AP[i][5] * K[j][1];
            P_new[i][j] = val;
        }
    }

    // Ajout du terme de bruit K * R * K^T
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            float add = K[i][0] * R_imu[0] * K[j][0]
                      + K[i][1] * R_imu[1] * K[j][1];
            P_new[i][j] += add;
        }
    }

    // Symétrisation et copie finale
    for (int i = 0; i < 6; ++i) {
        for (int j = i; j < 6; ++j) {
            float s = 0.5f * (P_new[i][j] + P_new[j][i]);
            state->P[i][j] = s;
            state->P[j][i] = s;
        }
    }

    return 0; // Succès
}