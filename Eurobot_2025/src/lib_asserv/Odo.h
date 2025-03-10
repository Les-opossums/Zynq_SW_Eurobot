#ifndef __ODO_H_
#define __ODO_H_


extern float Speed_1, Speed_2, Speed_3;

extern float robot_wheel_distance;

/******************************    Fonctions    *******************************/

// initialiser l'odometrie
void odo_init(void);

// assigner des valeurs aux coefs (relations tic/metre et entraxe)
void odo_set_tic_by_meter(double param_tic_by_meter);

// assigner une valeur a l'ecart entre les roues d'odometrie
void odo_set_spacing(float param_spacing);

// maj de la position du robot
void odo_position_step(int16_t delta_angle_motor_1, int16_t delta_angle_motor_2, int16_t delta_angle_motor_3);

// maj de la vitesse/acceleration  du robot
void odo_speed_step(int16_t Rotor_RPM1, int16_t Rotor_RPM2, int16_t Rotor_RPM3);

float odo_dist_roue(int16_t delta_angle_motor);

// connaitre l'etat du robot
Position get_position(void);
Speed get_speed(void);
Acceleration get_acceleration(void);


// assigner des valeurs a la position (x, y et theta)
void set_position(Position pos);
void set_position_x(float x);
void set_position_y(float y);
void set_position_t(float t);

#endif // _ODO_H_
