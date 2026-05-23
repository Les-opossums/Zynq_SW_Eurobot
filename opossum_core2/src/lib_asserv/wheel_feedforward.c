#include "lib_asserv.h"

extern volatile int Timer_ms1;

// Valeurs par défaut — à remplacer après calibration
// Format : {I_static, B, v_deadzone}
WheelFF wheel_ff[4] = {
    {228.0f, 1576.0f, 0.04f},
    {161.0f, 1633.6f, 0.04f},
    {188.0f, 1598.3f, 0.04f},
    {344.6f, 1630.1f, 0.04f},
};

float compute_feedforward(WheelFF* ff, float v_cmd) {
    if (fabsf(v_cmd) < 1e-4f) return 0.0f;

    float sign    = (v_cmd > 0.0f) ? 1.0f : -1.0f;
    float abs_v   = fabsf(v_cmd);

    // Transition linéaire dans la zone morte pour éviter les à-coups
    float ff_static = (abs_v >= ff->v_deadzone)
        ? sign * ff->I_static
        : sign * ff->I_static * (abs_v / ff->v_deadzone);

    float ff_viscous = ff->B * v_cmd;

    return ff_static + ff_viscous;
}

// ----------------------------------------------------------------
// Calibration automatique d'une roue (robot sur chandelles)
//
// Principe :
//   Pour chaque palier de courant, on attend la vitesse stable
//   et on logue (I_cmd, v_mesurée). Fit linéaire ensuite.
//
// Usage : appeler wheel_ff_calibrate(0) ... (3) via commande série
// ----------------------------------------------------------------

// --- Variables globales pour la machine à états de calibration ---
typedef enum {
    CALIB_IDLE = 0,
    CALIB_INIT,
    CALIB_APPLY_CURRENT,
    CALIB_WAIT_SETTLE,
    CALIB_SAMPLING,
    CALIB_DONE_SAMPLE,
    CALIB_PAUSE,
    CALIB_FINISH
} FF_Calib_State;

static FF_Calib_State calib_state = CALIB_IDLE;
static int wheel_idx = 0;
static int current_idx = 0;
static int calib_sign = 1;
static int t_start = 0;

static float v_sum = 0.0f;
static int n_samples = 0;

#define CALIB_SETTLE_MS     800    // temps d'attente stabilisation
#define CALIB_SAMPLE_MS     200    // durée de mesure
#define CALIB_N_POINTS      10     // nombre de paliers de courant

// Paliers de courant positifs testés (unités C610, roue libre)
static const int calib_currents[CALIB_N_POINTS] = {
    150, 250, 350, 500, 700, 1000, 1400, 1800, 2500, 3500
};

// Fonction à appeler pour lancer la procédure depuis le RPi
void start_wheel_ff_calibration(void) {
    if (calib_state == CALIB_IDLE) {
        calib_state = CALIB_INIT;
        motion_off(); // Désactive le PID pour ne pas interférer
    }
}

// Machine à états à exécuter dans la Slow Loop
void step_wheel_ff_calibration(void) {
    if (calib_state == CALIB_IDLE) return;

    switch (calib_state) {
        case CALIB_IDLE:
            break;
        case CALIB_INIT:
            wheel_idx = 0;
            current_idx = 0;
            calib_sign = 1;
            printf("FF_CAL_START_ALL\n");
            printf("FF_CAL # Format: FF_CAL wheel I_cmd v_ms\n");
            calib_state = CALIB_APPLY_CURRENT;
            break;

        case CALIB_APPLY_CURRENT:
        {
            ESC_Command forced = {0, 0, 0, 0};
            int current_val = calib_sign * calib_currents[current_idx];
            
            // On applique le courant sur la roue ciblée
            if (wheel_idx == 0) forced.command1 = current_val;
            else if (wheel_idx == 1) forced.command2 = current_val;
            else if (wheel_idx == 2) forced.command3 = current_val;
            else if (wheel_idx == 3) forced.command4 = current_val;

            Wanted_Forced_Consigne = forced;
            t_start = Timer_ms1;
            calib_state = CALIB_WAIT_SETTLE;
            break;
        }

        case CALIB_WAIT_SETTLE:
            if ((Timer_ms1 - t_start) >= CALIB_SETTLE_MS) {
                v_sum = 0.0f;
                n_samples = 0;
                t_start = Timer_ms1; // Reset timer pour l'échantillonnage
                calib_state = CALIB_SAMPLING;
            }
            break;

        case CALIB_SAMPLING:
        {
            float v = 0.0f;
            if (wheel_idx == 0) v = Speed_1;
            else if (wheel_idx == 1) v = Speed_2;
            else if (wheel_idx == 2) v = Speed_3;
            else if (wheel_idx == 3) v = Speed_4;

            v_sum += v;
            n_samples++;

            if ((Timer_ms1 - t_start) >= CALIB_SAMPLE_MS) {
                calib_state = CALIB_DONE_SAMPLE;
            }
            break;
        }

        case CALIB_DONE_SAMPLE:
        {
            float v_mean = (n_samples > 0) ? (v_sum / n_samples) : 0.0f;
            int current_val = calib_sign * calib_currents[current_idx];
            printf("FF_CAL %d %d %.5f\n", wheel_idx, current_val, (double)v_mean);

            // Logique de passage à l'étape suivante
            if (calib_sign == 1) {
                calib_sign = -1; // Test en négatif
                calib_state = CALIB_APPLY_CURRENT;
            } else {
                calib_sign = 1;
                current_idx++; // Palier de courant suivant
                
                if (current_idx >= CALIB_N_POINTS) {
                    current_idx = 0;
                    Wanted_Forced_Consigne.command1 = 0;
                    Wanted_Forced_Consigne.command2 = 0;
                    Wanted_Forced_Consigne.command3 = 0;
                    Wanted_Forced_Consigne.command4 = 0;
                    
                    printf("FF_CAL_END wheel=%d\n", wheel_idx);
                    wheel_idx++; // Roue suivante
                    
                    if (wheel_idx >= 4) {
                        calib_state = CALIB_FINISH;
                    } else {
                        t_start = Timer_ms1;
                        calib_state = CALIB_PAUSE;
                    }
                } else {
                    calib_state = CALIB_APPLY_CURRENT;
                }
            }
            break;
        }

        case CALIB_PAUSE:
            if ((Timer_ms1 - t_start) >= 2000) { // 2s entre chaque roue
                calib_state = CALIB_APPLY_CURRENT;
            }
            break;

        case CALIB_FINISH:
            Wanted_Forced_Consigne.command1 = 0;
            Wanted_Forced_Consigne.command2 = 0;
            Wanted_Forced_Consigne.command3 = 0;
            Wanted_Forced_Consigne.command4 = 0;
            printf("FF_CALIBRATION_ALL_END\n");
            calib_state = CALIB_IDLE;
            break;
        
        default:
            calib_state = CALIB_IDLE;
            break;
    }
}