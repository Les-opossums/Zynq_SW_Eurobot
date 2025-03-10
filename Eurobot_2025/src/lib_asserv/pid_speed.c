#include "lib_asserv.h"

/******************************    Variables    *******************************/
PID_speed pid_speed;

uint8_t Pid_Speed_En = 0;

/******************************    Fonctions    *******************************/
void pid_vitesse_init (void) {
    // PID pour l'asserv en vitesse
    pid_speed.coef.kp = DEFAULT_PID_V_LIN_KP;
    pid_speed.coef.ki = DEFAULT_PID_V_LIN_KI;
    pid_speed.coef.kd = DEFAULT_PID_V_LIN_KD;
}

void pid_vitesse_reset (void) {
    pid_speed.err1.err = 0;
    pid_speed.err1.err_int = 0;
    pid_speed.err1.err_der = 0;

    pid_speed.err2.err = 0;
    pid_speed.err2.err_int = 0;
    pid_speed.err2.err_der = 0;

    pid_speed.err3.err = 0;
    pid_speed.err3.err_int = 0;
    pid_speed.err3.err_der = 0;
}

/******************************    Fonctions Utilitaires   *******************************/
ESC_Command pid_speed_processing (PID_speed *pid, float err1, float err2, float err3) {
    ESC_Command commande;
    if (Pid_Speed_En) {
        //maj de la derivee de l'erreur du PID
        pid->err1.err_der = err1 - pid->err1.err;
        pid->err2.err_der = err2 - pid->err2.err;
        pid->err3.err_der = err3 - pid->err3.err;

        //maj de l'erreur du PID
        pid->err1.err = err1;
        pid->err2.err = err2;
        pid->err3.err = err3;
        // maj de l'integrale de l'erreur du PID
        // avec limite pour que KP+KI soit toujours entre 100 et - 100 (% de PWM)
        // ne gere que si KI != 0
        if (pid->coef.ki != 0) {
            pid->err1.err_int += err1;
            float max_int = (MOTOR_POWER_MAX - (pid->err1.err * pid->coef.kp)) / pid->coef.ki;
            float min_int = (MOTOR_POWER_MIN - (pid->err1.err * pid->coef.kp)) / pid->coef.ki;
            pid->err1.err_int = limit_float(pid->err1.err_int, min_int, max_int);
        }

        if (pid->coef.ki != 0) {
            pid->err2.err_int += err2;
            float max_int = (MOTOR_POWER_MAX - (pid->err2.err * pid->coef.kp)) / pid->coef.ki;
            float min_int = (MOTOR_POWER_MIN - (pid->err2.err * pid->coef.kp)) / pid->coef.ki;
            pid->err2.err_int = limit_float(pid->err2.err_int, min_int, max_int);
        }

        if (pid->coef.ki != 0) {
            pid->err3.err_int += err3;
            float max_int = (MOTOR_POWER_MAX - (pid->err3.err * pid->coef.kp)) / pid->coef.ki;
            float min_int = (MOTOR_POWER_MIN - (pid->err3.err * pid->coef.kp)) / pid->coef.ki;
            pid->err3.err_int = limit_float(pid->err3.err_int, min_int, max_int);
        }
        commande.command1 = pid->coef.kp * pid->err1.err + pid->coef.ki * pid->err1.err_int + pid->coef.kd * pid->err1.err_der;
        commande.command2 = pid->coef.kp * pid->err2.err + pid->coef.ki * pid->err2.err_int + pid->coef.kd * pid->err2.err_der;
        commande.command3 = pid->coef.kp * pid->err3.err + pid->coef.ki * pid->err3.err_int + pid->coef.kd * pid->err3.err_der;
        return commande;
    } else {
        pid_vitesse_reset();
        commande.command1 = 0;
        commande.command2 = 0;
        commande.command3 = 0;
        return commande;
    }
}


