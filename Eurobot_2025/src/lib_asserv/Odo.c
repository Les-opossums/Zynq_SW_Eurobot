#include "lib_asserv.h"


/******************************    Variables    *******************************/
float robot_wheel_distance;

Position position_robot;    // en repere absolu
Speed speed_robot;                  // en repere relatif
Acceleration acceleration_robot;    // en repere relatif

float Speed_1, Speed_2, Speed_3;

/******************************    Fonctions    *******************************/

// initialiser l'odometrie
void odo_init(void) {
    odo_set_spacing(DEFAULT_ODO_SPACING);

    position_robot.x = 0;
    position_robot.y = 0;
    position_robot.t = 0;

    speed_robot.vx = 0;
    speed_robot.vy = 0;
    speed_robot.vt = 0;

    acceleration_robot.ax = 0;
    acceleration_robot.ay = 0;
    acceleration_robot.at = 0;

    xil_printf("Odo init done\n");
}

// assigner une valeur e l'ecart entre les roues d'odometrie
void odo_set_spacing(float param_spacing) {
    robot_wheel_distance = param_spacing;
}

void odo_position_step(int16_t delta_angle_motor_1, int16_t delta_angle_motor_2, int16_t delta_angle_motor_3) {   
    // calcul de la distance linéaire parcourue par chaque roues
    float dist_motor_1 = odo_dist_roue(delta_angle_motor_1);
    float dist_motor_2 = odo_dist_roue(delta_angle_motor_2);
    float dist_motor_3 = odo_dist_roue(delta_angle_motor_3);

    float dx = -0.5*(dist_motor_2 - dist_motor_3)/sin(PI/3);
    float dy = 0.5*(dist_motor_1 -(dist_motor_2 + dist_motor_3));
    float dt = -(dist_motor_1 + dist_motor_2 + dist_motor_3) / (3*robot_wheel_distance);

    float rdx = dx*cos(position_robot.t) - dy*sin(position_robot.t);
    float rdy = dx*sin(position_robot.t) + dy*cos(position_robot.t);

    position_robot.x += rdx;
    position_robot.y += rdy;
    position_robot.t = principal_angle(position_robot.t + dt);

}

float odo_dist_roue(int16_t delta_angle_motor) {
    static const float facteur_conversion = TWO_PI / (MOTOR_ANGLE_CODEUR_MAX * 36); // Conversion en radian

    // Calcul de la variation d'angle normalisée
    if (delta_angle_motor > (MOTOR_ANGLE_CODEUR_MAX / 2)) {
        delta_angle_motor -= MOTOR_ANGLE_CODEUR_MAX;
    } else if (delta_angle_motor < -(MOTOR_ANGLE_CODEUR_MAX / 2)) {
        delta_angle_motor += MOTOR_ANGLE_CODEUR_MAX;
    }

    float deltaAngleRad = delta_angle_motor * facteur_conversion; // Conversion en radian
    float distance = DEFAULT_WHEEL_RADIUS * deltaAngleRad; // Calcul de la distance parcourue
    return distance; // Retourne la distance parcourue
}

void odo_speed_step(int16_t Rotor_RPM1, int16_t Rotor_RPM2, int16_t Rotor_RPM3) {
    // vitesse roues en m/s
    Speed_1 = (float)((Rotor_RPM1*PI*DEFAULT_SIZE_WHEEL)/(36.0*60.0)); //36 reducteur du moteur
    Speed_2 = (float)((Rotor_RPM2*PI*DEFAULT_SIZE_WHEEL)/(36.0*60.0));
    Speed_3 = (float)((Rotor_RPM3*PI*DEFAULT_SIZE_WHEEL)/(36.0*60.0));

    // sauvegarde des anciennes vitesses
    float vx = speed_robot.vx;
    float vy = speed_robot.vy;
    float vt = speed_robot.vt;

    // maj des vitesses
    speed_robot.vx = -0.5*(Speed_2 - Speed_3)/sin(PI/3);
    speed_robot.vy = 0.5*(Speed_1 -(Speed_2 + Speed_3));
    speed_robot.vt = -(Speed_1 + Speed_2 + Speed_3) / (3*robot_wheel_distance);

    // maj des accelerations
    acceleration_robot.ax = speed_robot.vx - vx;
    acceleration_robot.ay = speed_robot.vy - vy;
    acceleration_robot.at = speed_robot.vt - vt;
}


void set_position(Position pos) {
    position_robot = pos;
}

void set_position_x(float x) {
    position_robot.x = x;
}

void set_position_y(float y) {
    position_robot.y = y;
}

void set_position_t(float t) {
    position_robot.t = t;
}


Position get_position(void) {
    return position_robot;
}

Speed get_speed(void) {
    return speed_robot;
}

Acceleration get_acceleration(void) {
    return acceleration_robot;
}
