
#include "lib_asserv.h"

/******************************    Variables    *******************************/
// coefficient du PID (proportionnel, integrale, derivee) et leurs coefs de moyennage pour l'asserv en position (dist)
PID_pos pid_dist;

//coefficient du PID (proportionnel, integrale, derivee) et leurs coefs de moyennage pour l'asserv en position (angle)
PID_pos pid_angle;

/******************************    Fonctions    *******************************/
void pid_position_init(void) {
    // PID pour l'asserv en position (dist)
    pid_dist.coef.kp = DEFAULT_PID_DIST_KP;
    pid_dist.coef.ki = DEFAULT_PID_DIST_KI;
    pid_dist.coef.kd = DEFAULT_PID_DIST_KD;

    // PID pour l'asserv en position angulaire (angle)
    pid_angle.coef.kp = DEFAULT_PID_ANGLE_KP;
    pid_angle.coef.ki = DEFAULT_PID_ANGLE_KI;
    pid_angle.coef.kd = DEFAULT_PID_ANGLE_KD;

    pid_position_reset();
}

void pid_position_reset(void) {
    // PID pour l'asserv en position (dist)
    pid_dist.err.err = 0;
    pid_dist.err.err_int = 0;
    pid_dist.err.err_der = 0;
    // PID pour l'asserv en position angulaire (angle)
    pid_angle.err.err = 0;
    pid_angle.err.err_int = 0;
    pid_angle.err.err_der = 0;
}

/******************************    Fonctions Utilitaires   *******************************/
float pid_position_processing (PID_pos *pid, float err) {
	// maj de la derivee de l'erreur du PID
    float pid_distx_derr = err - pid->err.err;

    // maj de l'erreur du PID
    pid->err.err = err;

	// maj de l'integrale
	if (pid->coef.ki != 0) {
        pid->err.err_int += err;
	} else {
        pid->err.err_int = 0;
	}
    return (pid->err.err * pid->coef.kp) + (pid->err.err_int * pid->coef.ki) + (pid_distx_derr * pid->coef.kd);
}
