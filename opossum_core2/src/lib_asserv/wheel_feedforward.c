#include "lib_asserv.h"
#include "../Timer.h"

// Valeurs par défaut — à remplacer après calibration
// Format : {I_static, B, v_deadzone}
WheelFF wheel_ff[4] = {
    {350.0f, 180.0f, 0.04f},  // roue 0 : avant-droite
    {350.0f, 180.0f, 0.04f},  // roue 1 : arrière-droite
    {350.0f, 180.0f, 0.04f},  // roue 2 : arrière-gauche
    {350.0f, 180.0f, 0.04f},  // roue 3 : avant-gauche
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

#define CALIB_SETTLE_MS     800    // temps d'attente stabilisation
#define CALIB_SAMPLE_MS     200    // durée de mesure
#define CALIB_N_POINTS      10     // nombre de paliers de courant

// Paliers de courant positifs testés (unités C610, roue libre)
static const int calib_currents[CALIB_N_POINTS] = {
    150, 250, 350, 500, 700, 1000, 1400, 1800, 2500, 3500
};

void wheel_ff_calibrate(uint8_t wheel_id) {
    if (wheel_id > 3) return;

    printf("FF_CAL_START wheel=%d\n", wheel_id);
    printf("FF_CAL # Format: FF_CAL wheel I_cmd v_ms\n");

    for (int p = 0; p < CALIB_N_POINTS; p++) {

        // --- Commande directe de courant (bypass PID complet) ---
        ESC_Command forced = {0, 0, 0, 0};
        switch (wheel_id) {
            case 0: forced.command1 = calib_currents[p]; break;
            case 1: forced.command2 = calib_currents[p]; break;
            case 2: forced.command3 = calib_currents[p]; break;
            case 3: forced.command4 = calib_currents[p]; break;
        }
        // On s'appuie sur le mécanisme existant de forced consigne
        // (les autres roues à 0 = freinées par leur ESC → robot immobile sur stand)
        extern ESC_Command Wanted_Forced_Consigne;
        Wanted_Forced_Consigne = forced;

        // --- Attente de stabilisation ---
        int t_start = Timer_ms1;
        while (Timer_ms1 - t_start < CALIB_SETTLE_MS);

        // --- Mesure vitesse moyenne sur CALIB_SAMPLE_MS ---
        float v_sum = 0.0f;
        int   n_samples = 0;
        t_start = Timer_ms1;

        while (Timer_ms1 - t_start < CALIB_SAMPLE_MS) {
            float v = 0.0f;
            switch (wheel_id) {
                case 0: v = Speed_1; break;
                case 1: v = Speed_2; break;
                case 2: v = Speed_3; break;
                case 3: v = Speed_4; break;
            }
            v_sum += v;
            n_samples++;
            // Petite attente pour ne pas sur-échantillonner
            int t_w = Timer_ms1;
            while (Timer_ms1 - t_w < 5);
        }

        float v_mean = (n_samples > 0) ? v_sum / n_samples : 0.0f;

        printf("FF_CAL %d %d %.5f\n", wheel_id, calib_currents[p], (double)v_mean);

        // --- Test négatif (symétrie) ---
        switch (wheel_id) {
            case 0: forced.command1 = -calib_currents[p]; break;
            case 1: forced.command2 = -calib_currents[p]; break;
            case 2: forced.command3 = -calib_currents[p]; break;
            case 3: forced.command4 = -calib_currents[p]; break;
        }
        Wanted_Forced_Consigne = forced;

        t_start = Timer_ms1;
        while (Timer_ms1 - t_start < CALIB_SETTLE_MS);

        v_sum = 0.0f; n_samples = 0;
        t_start = Timer_ms1;
        while (Timer_ms1 - t_start < CALIB_SAMPLE_MS) {
            float v = 0.0f;
            switch (wheel_id) {
                case 0: v = Speed_1; break;
                case 1: v = Speed_2; break;
                case 2: v = Speed_3; break;
                case 3: v = Speed_4; break;
            }
            v_sum += v;
            n_samples++;
            int t_w = Timer_ms1;
            while (Timer_ms1 - t_w < 5);
        }
        v_mean = (n_samples > 0) ? v_sum / n_samples : 0.0f;

        printf("FF_CAL %d %d %.5f\n", wheel_id, -calib_currents[p], (double)v_mean);
    }

    // Remise à zéro
    ESC_Command zero = {0, 0, 0, 0};
    Wanted_Forced_Consigne = zero;

    printf("FF_CAL_END wheel=%d\n", wheel_id);
}

void wheel_ff_calibrate_all(void) {
    for (int i = 0; i < 4; i++) {
        wheel_ff_calibrate(i);
        // Pause entre les roues
        int t = Timer_ms1;
        while (Timer_ms1 - t < 2000);
    }
}