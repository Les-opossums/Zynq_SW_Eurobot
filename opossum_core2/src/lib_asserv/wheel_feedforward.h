#ifndef WHEEL_FEEDFORWARD_H
#define WHEEL_FEEDFORWARD_H

typedef struct {
    float I_static;    // frottement statique (unités C610, 0–10000)
    float B;           // frottement visqueux (unités C610 par m/s)
    float v_deadzone;  // seuil de vitesse pour la transition douce (m/s)
} WheelFF;

extern WheelFF wheel_ff[4];

float compute_feedforward(WheelFF* ff, float v_cmd);
void  wheel_ff_calibrate(uint8_t wheel_id);
void  wheel_ff_calibrate_all(void);

#endif