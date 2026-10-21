#ifndef COMMON_TYPE_H
#define COMMON_TYPE_H

#include <stdint.h>

/* Types de l'asservissement et du Kalman (Position, Speed, PID_*, ESC_Command,
 * Set_lidar, Set_camera...) : definis par la bibliotheque opossum_control,
 * commune au firmware et a la simulation PC. */
#include "../libs/opossum_control/src/control_types.h"

typedef struct __attribute__((packed)){
    int odo_step_1; // step 1: calcul de la vitesse du robot
    int odo_step_2; // step 2: calcul du kalman + history
    int odo_step_3; // step 3: calcul de la vitesse du robot
    int motion_step; // step 4: calcul de la position du robot
    int speed_constrain_step; // step 5: contrainte de vitesse
    int acceleration_constrain_step; // step 6: contrainte d'acceleration
    int consigne_step; // step 7: calcul de la consigne
    int pwm_step; // step 8: calcul du PWM
    int transmit_step; // step 9: transmission des ordres aux moteurs
} Asserv_Step_Timing;

#endif /* COMMON_TYPE_H */