#include "lib_asserv.h"

void Asserv_PWM_calculator(ESC_Command *commande) {
    // maj des consignes des PID
	float err1 = Speed_Order_1 - Speed_1;
	float err2 = Speed_Order_2 - Speed_2;
	float err3 = Speed_Order_3 - Speed_3;
	float err4 = Speed_Order_4 - Speed_4;

	// calcul des commandes
	ESC_Command pid_out = pid_speed_processing(&pid_speed, err1, err2, err3, err4);
	
 	commande->command1 = pid_out.command1 + compute_feedforward(&wheel_ff[0], Speed_Order_1);
    commande->command2 = pid_out.command2 + compute_feedforward(&wheel_ff[1], Speed_Order_2);
    commande->command3 = pid_out.command3 + compute_feedforward(&wheel_ff[2], Speed_Order_3);
    commande->command4 = pid_out.command4 + compute_feedforward(&wheel_ff[3], Speed_Order_4);
}
