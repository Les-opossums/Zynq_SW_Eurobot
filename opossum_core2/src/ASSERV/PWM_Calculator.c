#include "asserv.h"

void Asserv_PWM_calculator(ESC_Command *commande)
{
    const float err1 = Speed_Order_1 - wheel_speed[0];
    const float err2 = Speed_Order_2 - wheel_speed[1];
    const float err3 = Speed_Order_3 - wheel_speed[2];
    const float err4 = Speed_Order_4 - wheel_speed[3];

    *commande = pid_speed_processing(&pid_speed, err1, err2, err3, err4);

    commande->command1 *= ODO_MOTOR_1_SIGN;
    commande->command2 *= ODO_MOTOR_2_SIGN;
    commande->command3 *= ODO_MOTOR_3_SIGN;
    commande->command4 *= ODO_MOTOR_4_SIGN;
}
