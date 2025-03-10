#include "lib_asserv.h"


void Asserv_PWM_calculator(ESC_Command *commande) {
    // maj des consignes des PID
	float err1 = Speed_Order_1 - Speed_1;
	float err2 = Speed_Order_2 - Speed_2;
	float err3 = Speed_Order_3 - Speed_3;

	// calcul des commandes
	*commande = pid_speed_processing(&pid_speed, err1, err2, err3);
}
