#ifndef __ASSERV_MASTER_H
#define __ASSERV_MASTER_H


// calcul d'un step d'odometrie toutes les 2 ms
#define ODO_EVERY_MS 1
// asserv tous les 5 steps d'odo
#define ASSERV_EVERY 5

#define PWM_BLOCK_LIMIT 90      // % de PWM qui declenche la limite
#define BLOCK_MAX_TIME  2000    // temps en ms au bout duquel il se verouille
extern uint16_t Asserv_Full_Count;  // variable a remettre a 0 pour recuperer le droit de bouger en cas de pb..

extern float Consigne1, Consigne2, Consigne3;
extern int16_t Rotor_RPM1;
extern int16_t Rotor_RPM2;
extern int16_t Rotor_RPM3;


void Init_Asserv(void);

void Asserv_Loop(void);
uint8_t Activate_Position_Sending_Func(void);

uint8_t Set_Odo_Spacing_Cmd(void);


#endif


