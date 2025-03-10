#ifndef __PID_SPEED_H_
#define __PID_SPEED_H_

// coefficient du PID (proportionnel, integrale, derivee) et leurs coefs de moyennage pour l'asserv en vitesse lineaire
extern PID_speed pid_speed;

extern uint8_t Pid_Speed_En;

/******************************    Fonctions    *******************************/
void pid_vitesse_init(void);

void pid_vitesse_reset(void); //reset des erreurs integrales des pid

/******************************    Fonctions Utilitaires   *******************************/
ESC_Command pid_speed_processing(PID_speed *pid, float err1, float err2, float err3);



#endif
