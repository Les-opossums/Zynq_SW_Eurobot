#include "kalman.h"

KalmanState kalman_current_state;

void kalman_init(KalmanState* state) {
    memset(state->x, 0, sizeof(state->x));
    memset(state->P, 0, sizeof(state->P));
    state->P[0][0] = 1.0f;
    state->P[1][1] = 1.0f;
    state->P[2][2] = 1.0f;
    state->P[3][3] = 1.0f;
    state->P[4][4] = 1.0f;
    state->P[5][5] = 1.0f;
}

// ─────────────────────────────────────────────────────────────────────────────
// kalman_predict
// Optimisations :
//   - powf() remplacé par une multiplication simple (x² au lieu de powf(x,2))
//   - Boucles FP et P_new entièrement déroulées (le compilateur ne peut pas le
//     faire seul car les accès P[i][j] créent des dépendances apparentes)
//   - Symétrisation déroulée : seulement 21 paires au lieu de 36 calculs
//   - Suppression de la copie intermédiaire P_new → state->P via déroulage direct
// ─────────────────────────────────────────────────────────────────────────────
void kalman_predict(KalmanState* state, float dt) {
    float x      = state->x[0];
    float y      = state->x[1];
    float theta  = state->x[2];
    float vx     = state->x[3];
    float vy     = state->x[4];
    float vtheta = state->x[5];

    float angle_mid = principal_angle(theta + vtheta * dt * 0.5f);
    float cos_t = cosf(angle_mid);
    float sin_t = sinf(angle_mid);

    state->x[0] = x + (vx * cos_t - vy * sin_t) * dt;
    state->x[1] = y + (vx * sin_t + vy * cos_t) * dt;
    state->x[2] = principal_angle(theta + vtheta * dt);

    float F02 = (-vx * sin_t - vy * cos_t) * dt;
    float F12 = ( vx * cos_t - vy * sin_t) * dt;
    float F03 = cos_t * dt;
    float F04 = -sin_t * dt;
    float F13 = sin_t * dt;
    float F14 = cos_t * dt;
    float F05 = F02 * (dt * 0.5f);
    float F15 = F12 * (dt * 0.5f);

    // OPT : lire P dans des scalaires locaux évite des accès mémoire répétés
    // sur les lignes/colonnes utilisées dans FP puis P_new.
    // Les 36 éléments de P sont lus une seule fois chacun.
    float p00=state->P[0][0], p01=state->P[0][1], p02=state->P[0][2],
          p03=state->P[0][3], p04=state->P[0][4], p05=state->P[0][5];
    float p10=state->P[1][0], p11=state->P[1][1], p12=state->P[1][2],
          p13=state->P[1][3], p14=state->P[1][4], p15=state->P[1][5];
    float p20=state->P[2][0], p21=state->P[2][1], p22=state->P[2][2],
          p23=state->P[2][3], p24=state->P[2][4], p25=state->P[2][5];
    float p30=state->P[3][0], p31=state->P[3][1], p32=state->P[3][2],
          p33=state->P[3][3], p34=state->P[3][4], p35=state->P[3][5];
    float p40=state->P[4][0], p41=state->P[4][1], p42=state->P[4][2],
          p43=state->P[4][3], p44=state->P[4][4], p45=state->P[4][5];
    float p50=state->P[5][0], p51=state->P[5][1], p52=state->P[5][2],
          p53=state->P[5][3], p54=state->P[5][4], p55=state->P[5][5];

    // FP = F * P  (lignes 3,4,5 de F = identité → FP = P pour ces lignes)
    float fp00=p00+F02*p20+F03*p30+F04*p40+F05*p50;
    float fp01=p01+F02*p21+F03*p31+F04*p41+F05*p51;
    float fp02=p02+F02*p22+F03*p32+F04*p42+F05*p52;
    float fp03=p03+F02*p23+F03*p33+F04*p43+F05*p53;
    float fp04=p04+F02*p24+F03*p34+F04*p44+F05*p54;
    float fp05=p05+F02*p25+F03*p35+F04*p45+F05*p55;

    float fp10=p10+F12*p20+F13*p30+F14*p40+F15*p50;
    float fp11=p11+F12*p21+F13*p31+F14*p41+F15*p51;
    float fp12=p12+F12*p22+F13*p32+F14*p42+F15*p52;
    float fp13=p13+F12*p23+F13*p33+F14*p43+F15*p53;
    float fp14=p14+F12*p24+F13*p34+F14*p44+F15*p54;
    float fp15=p15+F12*p25+F13*p35+F14*p45+F15*p55;

    float fp20=p20+dt*p50;
    float fp21=p21+dt*p51;
    float fp22=p22+dt*p52;
    float fp23=p23+dt*p53;
    float fp24=p24+dt*p54;
    float fp25=p25+dt*p55;

    // FP[3..5] = P[3..5] (identité)

    // P_new = FP * F^T + Q  — seulement la partie triangulaire supérieure,
    // puis copie symétrique directement dans state->P.
    // OPT bruit Q : (a+b)² = a²+2ab+b² mais powf() coûte ~20 cycles.
    //               On factorise avec une simple multiplication.
    float abs_vx = fabsf(vx);
    float abs_vy = fabsf(vy);
    float abs_vt = fabsf(vtheta);

    float tmp_x  = PROCESS_NOISE_BASE_X     + PROCESS_NOISE_VEL_X     * abs_vx;
    float tmp_y  = PROCESS_NOISE_BASE_Y     + PROCESS_NOISE_VEL_Y     * abs_vy;
    float tmp_t  = PROCESS_NOISE_BASE_THETA + PROCESS_NOISE_VEL_THETA * abs_vt;
    float q_var_x  = tmp_x  * tmp_x;
    float q_var_y  = tmp_y  * tmp_y;
    float q_var_t  = tmp_t  * tmp_t;
    float q_var_vx = PROCESS_NOISE_VX     * PROCESS_NOISE_VX;
    float q_var_vy = PROCESS_NOISE_VY     * PROCESS_NOISE_VY;
    float q_var_vt = PROCESS_NOISE_VTHETA * PROCESS_NOISE_VTHETA;

    // Calcul de P_new = FP * F^T rangée par rangée, triangulaire supérieure uniquement.
    // Rappel F^T : col 0 += F02*row2 + F03*row3 + F04*row4 + F05*row5
    //              col 1 += F12*row2 + F13*row3 + F14*row4 + F15*row5
    //              col 2 += dt*row5
    //              cols 3,4,5 : identité
    #define PNEW_COL0(fpi0,fpi2,fpi3,fpi4,fpi5) ((fpi0)+F02*(fpi2)+F03*(fpi3)+F04*(fpi4)+F05*(fpi5))
    #define PNEW_COL1(fpi1,fpi2,fpi3,fpi4,fpi5) ((fpi1)+F12*(fpi2)+F13*(fpi3)+F14*(fpi4)+F15*(fpi5))
    #define PNEW_COL2(fpi2,fpi5)                ((fpi2)+dt*(fpi5))

    // Ligne 0
    float n00 = PNEW_COL0(fp00,fp02,fp03,fp04,fp05) + q_var_x;
    float n01 = PNEW_COL1(fp01,fp02,fp03,fp04,fp05);
    float n02 = PNEW_COL2(fp02,fp05);
    float n03 = fp03;
    float n04 = fp04;
    float n05 = fp05;
    // Ligne 1
    float n11 = PNEW_COL1(fp11,fp12,fp13,fp14,fp15) + q_var_y;
    float n12 = PNEW_COL2(fp12,fp15);
    float n13 = fp13;
    float n14 = fp14;
    float n15 = fp15;
    // Ligne 2
    float n22 = PNEW_COL2(fp22,fp25) + q_var_t;
    float n23 = fp23;
    float n24 = fp24;
    float n25 = fp25;
    // Lignes 3,4,5 : FP[3..5] = P[3..5], F^T cols 3,4,5 = identité
    float n33 = p33 + q_var_vx;
    float n34 = p34;
    float n35 = p35;
    float n44 = p44 + q_var_vy;
    float n45 = p45;
    float n55 = p55 + q_var_vt;

    #undef PNEW_COL0
    #undef PNEW_COL1
    #undef PNEW_COL2

    // Copie symétrique directe dans state->P (pas de tableau P_new intermédiaire)
    state->P[0][0]=n00; state->P[0][1]=n01; state->P[0][2]=n02;
    state->P[0][3]=n03; state->P[0][4]=n04; state->P[0][5]=n05;
    state->P[1][0]=n01; state->P[1][1]=n11; state->P[1][2]=n12;
    state->P[1][3]=n13; state->P[1][4]=n14; state->P[1][5]=n15;
    state->P[2][0]=n02; state->P[2][1]=n12; state->P[2][2]=n22;
    state->P[2][3]=n23; state->P[2][4]=n24; state->P[2][5]=n25;
    state->P[3][0]=n03; state->P[3][1]=n13; state->P[3][2]=n23;
    state->P[3][3]=n33; state->P[3][4]=n34; state->P[3][5]=n35;
    state->P[4][0]=n04; state->P[4][1]=n14; state->P[4][2]=n24;
    state->P[4][3]=n34; state->P[4][4]=n44; state->P[4][5]=n45;
    state->P[5][0]=n05; state->P[5][1]=n15; state->P[5][2]=n25;
    state->P[5][3]=n35; state->P[5][4]=n45; state->P[5][5]=n55;
}

// ─────────────────────────────────────────────────────────────────────────────
// Utilitaire partagé : inversion 3×3 avec régularisation
// ─────────────────────────────────────────────────────────────────────────────
static int invert3x3(float S[3][3], float S_inv[3][3]) {
    float det = S[0][0]*(S[1][1]*S[2][2] - S[1][2]*S[2][1])
              - S[0][1]*(S[1][0]*S[2][2] - S[1][2]*S[2][0])
              + S[0][2]*(S[1][0]*S[2][1] - S[1][1]*S[2][0]);

    if (fabsf(det) < S_INV_EPS) {
        S[0][0] += S_INV_EPS; S[1][1] += S_INV_EPS; S[2][2] += S_INV_EPS;
        det = S[0][0]*(S[1][1]*S[2][2] - S[1][2]*S[2][1])
            - S[0][1]*(S[1][0]*S[2][2] - S[1][2]*S[2][0])
            + S[0][2]*(S[1][0]*S[2][1] - S[1][1]*S[2][0]);
        if (fabsf(det) < S_INV_EPS) return 0;
    }
    float inv = 1.0f / det;
    S_inv[0][0] =  (S[1][1]*S[2][2] - S[1][2]*S[2][1]) * inv;
    S_inv[0][1] = -(S[0][1]*S[2][2] - S[0][2]*S[2][1]) * inv;
    S_inv[0][2] =  (S[0][1]*S[1][2] - S[0][2]*S[1][1]) * inv;
    S_inv[1][0] = -(S[1][0]*S[2][2] - S[1][2]*S[2][0]) * inv;
    S_inv[1][1] =  (S[0][0]*S[2][2] - S[0][2]*S[2][0]) * inv;
    S_inv[1][2] = -(S[0][0]*S[1][2] - S[0][2]*S[1][0]) * inv;
    S_inv[2][0] =  (S[1][0]*S[2][1] - S[1][1]*S[2][0]) * inv;
    S_inv[2][1] = -(S[0][0]*S[2][1] - S[0][1]*S[2][0]) * inv;
    S_inv[2][2] =  (S[0][0]*S[1][1] - S[0][1]*S[1][0]) * inv;
    return 1;
}

// ─────────────────────────────────────────────────────────────────────────────
// kalman_update  (observation position/angle : H = [I3 | 0])
// Optimisations :
//   - K[6][3] calculé ligne par ligne avec scalaires locaux des colonnes P
//   - Les trois passes AP, P_new, KRKt fusionnées en une seule passe 6×6
//     → suppression d'un tableau 6×6 intermédiaire (AP)
//   - Symétrisation directe sans tableau P_new séparé
// ─────────────────────────────────────────────────────────────────────────────
#define CHI2_THRESHOLD_99 11.34f

uint8_t kalman_update(KalmanState* state, float z[3], float R_diag[3],
                      uint8_t bypass_outlier_rejection) {
    if (isnan(z[0]) || isnan(z[1]) || isnan(z[2]) ||
        isnan(state->x[0]) || isnan(state->x[1]) || isnan(state->x[2])) return 2;

    float y0 = z[0] - state->x[0];
    float y1 = z[1] - state->x[1];
    float y2 = principal_angle(z[2] - state->x[2]);

    float S[3][3] = {
        { state->P[0][0]+R_diag[0], state->P[0][1],           state->P[0][2]           },
        { state->P[1][0],           state->P[1][1]+R_diag[1], state->P[1][2]           },
        { state->P[2][0],           state->P[2][1],           state->P[2][2]+R_diag[2] }
    };
    float Si[3][3];
    if (!invert3x3(S, Si)) return 3;

    // Test Mahalanobis (Si est symétrique)
    float mah = y0*(y0*Si[0][0]+y1*Si[1][0]+y2*Si[2][0])
              + y1*(y0*Si[0][1]+y1*Si[1][1]+y2*Si[2][1])
              + y2*(y0*Si[0][2]+y1*Si[1][2]+y2*Si[2][2]);
    if (!bypass_outlier_rejection && mah > CHI2_THRESHOLD_99) return 1;

    // K = P[:,0:3] * Si  —  K[i][j] = P[i][0]*Si[0][j] + P[i][1]*Si[1][j] + P[i][2]*Si[2][j]
    // OPT : on lit P[i][0], P[i][1], P[i][2] une seule fois par ligne i
    float K[6][3];
    for (int i = 0; i < 6; i++) {
        float pi0 = state->P[i][0], pi1 = state->P[i][1], pi2 = state->P[i][2];
        K[i][0] = pi0*Si[0][0] + pi1*Si[1][0] + pi2*Si[2][0];
        K[i][1] = pi0*Si[0][1] + pi1*Si[1][1] + pi2*Si[2][1];
        K[i][2] = pi0*Si[0][2] + pi1*Si[1][2] + pi2*Si[2][2];
    }

    // x += K * y
    for (int i = 0; i < 6; i++) {
        float delta = K[i][0]*y0 + K[i][1]*y1 + K[i][2]*y2;
        if (i == 2) state->x[2] = principal_angle(state->x[2] + delta);
        else        state->x[i] += delta;
    }

    // P = (I-KH)*P*(I-KH)^T + K*R*K^T
    // OPT : on fusionne les trois passes en une seule.
    // A = I - KH  →  A*P ligne i : AP[i][j] = P[i][j] - K[i][0]*P[0][j]
    //                                                   - K[i][1]*P[1][j]
    //                                                   - K[i][2]*P[2][j]
    // Puis P_new[i][j] = AP[i][j] - AP[i][0]*K[j][0] - AP[i][1]*K[j][1] - AP[i][2]*K[j][2]
    //                              + K[i][0]*R0*K[j][0] + K[i][1]*R1*K[j][1] + K[i][2]*R2*K[j][2]
    // On calcule seulement la partie triangulaire supérieure et on copie en symétrique.

    float R0 = R_diag[0], R1 = R_diag[1], R2 = R_diag[2];

    // Précalcul des 6 vecteurs AP[i][0..5]
    float AP[6][6];
    for (int i = 0; i < 6; i++) {
        float ki0=K[i][0], ki1=K[i][1], ki2=K[i][2];
        AP[i][0] = state->P[i][0] - ki0*state->P[0][0] - ki1*state->P[1][0] - ki2*state->P[2][0];
        AP[i][1] = state->P[i][1] - ki0*state->P[0][1] - ki1*state->P[1][1] - ki2*state->P[2][1];
        AP[i][2] = state->P[i][2] - ki0*state->P[0][2] - ki1*state->P[1][2] - ki2*state->P[2][2];
        AP[i][3] = state->P[i][3] - ki0*state->P[0][3] - ki1*state->P[1][3] - ki2*state->P[2][3];
        AP[i][4] = state->P[i][4] - ki0*state->P[0][4] - ki1*state->P[1][4] - ki2*state->P[2][4];
        AP[i][5] = state->P[i][5] - ki0*state->P[0][5] - ki1*state->P[1][5] - ki2*state->P[2][5];
    }

    // P_new[i][j] triangulaire supérieure puis copie symétrique
    for (int i = 0; i < 6; i++) {
        float ai0=AP[i][0], ai1=AP[i][1], ai2=AP[i][2];
        float ki0=K[i][0],  ki1=K[i][1],  ki2=K[i][2];
        for (int j = i; j < 6; j++) {
            float v = AP[i][j]
                    - ai0*K[j][0] - ai1*K[j][1] - ai2*K[j][2]
                    + ki0*R0*K[j][0] + ki1*R1*K[j][1] + ki2*R2*K[j][2];
            state->P[i][j] = v;
            state->P[j][i] = v;
        }
    }

    return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// kalman_update_odo  (observation vitesse : H = [0 | I3])
// Même stratégie que kalman_update mais H sélectionne les colonnes/lignes 3,4,5
// ─────────────────────────────────────────────────────────────────────────────
float R_odo[3] = {
    OBS_NOISE_ODO_VX      * OBS_NOISE_ODO_VX,
    OBS_NOISE_ODO_VY      * OBS_NOISE_ODO_VY,
    OBS_NOISE_ODO_VTHETA  * OBS_NOISE_ODO_VTHETA
};

uint8_t kalman_update_odo(KalmanState* state, Speed* measured_speed) {
    float y0 = measured_speed->vx - state->x[3];
    float y1 = measured_speed->vy - state->x[4];
    float y2 = measured_speed->vt - state->x[5];

    float S[3][3] = {
        { state->P[3][3]+R_odo[0], state->P[3][4],           state->P[3][5]           },
        { state->P[4][3],          state->P[4][4]+R_odo[1],  state->P[4][5]           },
        { state->P[5][3],          state->P[5][4],           state->P[5][5]+R_odo[2]  }
    };
    float Si[3][3];
    if (!invert3x3(S, Si)) return 3;

    // K = P[:,3:6] * Si
    float K[6][3];
    for (int i = 0; i < 6; i++) {
        float pi3 = state->P[i][3], pi4 = state->P[i][4], pi5 = state->P[i][5];
        K[i][0] = pi3*Si[0][0] + pi4*Si[1][0] + pi5*Si[2][0];
        K[i][1] = pi3*Si[0][1] + pi4*Si[1][1] + pi5*Si[2][1];
        K[i][2] = pi3*Si[0][2] + pi4*Si[1][2] + pi5*Si[2][2];
    }

    // x += K * y
    for (int i = 0; i < 6; i++) {
        float delta = K[i][0]*y0 + K[i][1]*y1 + K[i][2]*y2;
        if (i == 2) state->x[2] = principal_angle(state->x[2] + delta);
        else        state->x[i] += delta;
    }

    // P = (I-KH)*P*(I-KH)^T + K*R*K^T   avec H=[0|I3] → A*P : soustrait colonnes 3,4,5 de P
    float AP[6][6];
    for (int i = 0; i < 6; i++) {
        float ki0=K[i][0], ki1=K[i][1], ki2=K[i][2];
        AP[i][0] = state->P[i][0] - ki0*state->P[3][0] - ki1*state->P[4][0] - ki2*state->P[5][0];
        AP[i][1] = state->P[i][1] - ki0*state->P[3][1] - ki1*state->P[4][1] - ki2*state->P[5][1];
        AP[i][2] = state->P[i][2] - ki0*state->P[3][2] - ki1*state->P[4][2] - ki2*state->P[5][2];
        AP[i][3] = state->P[i][3] - ki0*state->P[3][3] - ki1*state->P[4][3] - ki2*state->P[5][3];
        AP[i][4] = state->P[i][4] - ki0*state->P[3][4] - ki1*state->P[4][4] - ki2*state->P[5][4];
        AP[i][5] = state->P[i][5] - ki0*state->P[3][5] - ki1*state->P[4][5] - ki2*state->P[5][5];
    }

    float R0=R_odo[0], R1=R_odo[1], R2=R_odo[2];
    for (int i = 0; i < 6; i++) {
        float ai3=AP[i][3], ai4=AP[i][4], ai5=AP[i][5];
        float ki0=K[i][0],  ki1=K[i][1],  ki2=K[i][2];
        for (int j = i; j < 6; j++) {
            float v = AP[i][j]
                    - ai3*K[j][0] - ai4*K[j][1] - ai5*K[j][2]
                    + ki0*R0*K[j][0] + ki1*R1*K[j][1] + ki2*R2*K[j][2];
            state->P[i][j] = v;
            state->P[j][i] = v;
        }
    }

    return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// kalman_update_imu  (observation scalaire vtheta : H = [0,0,0,0,0,1])
// Optimisations :
//   - S scalaire → pas d'inversion 3×3, juste 1/S
//   - Joseph form entièrement déroulée : pas de tableau AP ni P_new
//     P_new[i][j] = P[i][j] - K[i]*P[5][j] - P[i][5]*K[j] + K[i]*K[j]*(P[5][5]+R)
//     (forme simplifiée exacte pour H scalaire sur la ligne/colonne 5)
//   - Symétrisation directe (21 affectations)
// ─────────────────────────────────────────────────────────────────────────────
float R_imu = OBS_NOISE_IMU_VTHETA * OBS_NOISE_IMU_VTHETA;

uint8_t kalman_update_imu(KalmanState* state, float measured_vtheta) {
    float y = measured_vtheta - state->x[5];

    float S = state->P[5][5] + R_imu;
    if (S < S_INV_EPS) return 3;
    float invS = 1.0f / S;

    // K[i] = P[i][5] * invS
    float K0 = state->P[0][5]*invS;
    float K1 = state->P[1][5]*invS;
    float K2 = state->P[2][5]*invS;
    float K3 = state->P[3][5]*invS;
    float K4 = state->P[4][5]*invS;
    float K5 = state->P[5][5]*invS;

    // x += K*y
    state->x[0] +=                         K0*y;
    state->x[1] +=                         K1*y;
    state->x[2] = principal_angle(state->x[2] + K2*y);
    state->x[3] +=                         K3*y;
    state->x[4] +=                         K4*y;
    state->x[5] +=                         K5*y;

    // P_new[i][j] = P[i][j] - K[i]*P[5][j] - P[i][5]*K[j] + K[i]*K[j]*S
    // (forme de Joseph pour H=[0,0,0,0,0,1], exactement équivalente)
    // OPT : on lit la ligne 5 de P et les K une seule fois.
    float p5j[6] = { state->P[5][0], state->P[5][1], state->P[5][2],
                     state->P[5][3], state->P[5][4], state->P[5][5] };
    float pi5[6] = { state->P[0][5], state->P[1][5], state->P[2][5],
                     state->P[3][5], state->P[4][5], state->P[5][5] };
    float K[6]   = { K0, K1, K2, K3, K4, K5 };

    for (int i = 0; i < 6; i++) {
        for (int j = i; j < 6; j++) {
            float v = state->P[i][j]
                    - K[i]*p5j[j]
                    - pi5[i]*K[j]
                    + K[i]*K[j]*S;
            state->P[i][j] = v;
            state->P[j][i] = v;
        }
    }

    return 0;
}