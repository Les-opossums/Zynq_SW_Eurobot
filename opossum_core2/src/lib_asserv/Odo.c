#include "lib_asserv.h"


/******************************    Variables    *******************************/
float robot_wheel_distance;

Speed speed_robot_asserv;                  // en repere relatif
Speed speed_robot_odom;
Speed cumulated_speed;    
Position position_robot; // Position odometrique
Position position_robot_odom; // Position odometrique pur
Acceleration acceleration_robot;    // en repere relatif

float Speed_1, Speed_2, Speed_3, Speed_4; // vitesse moyenne des roues en m/s
float Cumulated_Speed_1, Cumulated_Speed_2, Cumulated_Speed_3, Cumulated_Speed_4; // vitesse cumulee des roues en m/s


/******************************    Fonctions    *******************************/
void odo_init(void) {
    odo_set_spacing(DEFAULT_ODO_SPACING);

    position_robot_odom.x = 0;
    position_robot_odom.y = 0;
    position_robot_odom.t = 0;

    position_robot.x = 0;
    position_robot.y = 0;
    position_robot.t = 0;

    speed_robot_asserv.vx = 0;
    speed_robot_asserv.vy = 0;
    speed_robot_asserv.vt = 0;

    acceleration_robot.ax = 0;
    acceleration_robot.ay = 0;
    acceleration_robot.at = 0;
}

void odo_set_spacing(float param_spacing) {
    robot_wheel_distance = param_spacing;
}

void odo_speed_step(int16_t Rotor_RPM1, int16_t Rotor_RPM2, int16_t Rotor_RPM3, int16_t Rotor_RPM4) {
    // vitesse roues en m/s
    float Odo_Speed_1 = (float)((Rotor_RPM1*PI*DEFAULT_SIZE_WHEEL)/(36.0*60.0)); //36 reducteur du moteur
    float Odo_Speed_2 = (float)((Rotor_RPM2*PI*DEFAULT_SIZE_WHEEL)/(36.0*60.0));
    float Odo_Speed_3 = (float)((Rotor_RPM3*PI*DEFAULT_SIZE_WHEEL)/(36.0*60.0));
    float Odo_Speed_4 = (float)((Rotor_RPM4*PI*DEFAULT_SIZE_WHEEL)/(36.0*60.0));

    // vitesse cumulee roues
    Cumulated_Speed_1 += Odo_Speed_1;
    Cumulated_Speed_2 += Odo_Speed_2;
    Cumulated_Speed_3 += Odo_Speed_3;
    Cumulated_Speed_4 += Odo_Speed_4;
    
    // maj des vitesses odometrique pur pour le calcul de la position
    float inv_sqrt2 = 0.70710678f; // 1/sqrt(2)
    speed_robot_odom.vx = inv_sqrt2 * (-Odo_Speed_1 + Odo_Speed_2 + Odo_Speed_3 - Odo_Speed_4) / 2.0f;
    speed_robot_odom.vy = inv_sqrt2 * (Odo_Speed_1 + Odo_Speed_2 - Odo_Speed_3 - Odo_Speed_4) / 2.0f;
    speed_robot_odom.vt = -(Odo_Speed_1 + Odo_Speed_2 + Odo_Speed_3 + Odo_Speed_4) / (4.0f * robot_wheel_distance);

    // maj des vitesses odometrique cumulé pour le calcul de la vitesse
    cumulated_speed.vx += speed_robot_odom.vx;
    cumulated_speed.vy += speed_robot_odom.vy;
    cumulated_speed.vt += speed_robot_odom.vt;
}

void odo_position_step(float period) {   
    float dx_local = (speed_robot_odom.vx * period);
    float dy_local = (speed_robot_odom.vy * period);
    float dt_angle = speed_robot_odom.vt * period;
    float angle_mid = principal_angle(position_robot_odom.t + dt_angle * 0.5f);

    float cos_t = cosf(angle_mid);
    float sin_t = sinf(angle_mid);

    float dx_global = dx_local * cos_t - dy_local * sin_t;
    float dy_global = dx_local * sin_t + dy_local * cos_t;

    position_robot_odom.x += dx_global;
    position_robot_odom.y += dy_global;
    position_robot_odom.t = principal_angle(position_robot_odom.t + dt_angle);
}

void odo_speed_cumulate_step(float nbr_step) {
    Speed_1 = (Cumulated_Speed_1 / nbr_step);
    Speed_2 = (Cumulated_Speed_2 / nbr_step);
    Speed_3 = (Cumulated_Speed_3 / nbr_step);
    Speed_4 = (Cumulated_Speed_4 / nbr_step);

    Cumulated_Speed_1 = 0;
    Cumulated_Speed_2 = 0;
    Cumulated_Speed_3 = 0;
    Cumulated_Speed_4 = 0;
    
    speed_robot_asserv.vx = (cumulated_speed.vx / nbr_step);
    speed_robot_asserv.vy = (cumulated_speed.vy / nbr_step);
    speed_robot_asserv.vt = (cumulated_speed.vt / nbr_step);

    cumulated_speed.vx = 0;
    cumulated_speed.vy = 0;
    cumulated_speed.vt = 0;
}


void set_position(Position pos) {
    kalman_current_state.x[0] = pos.x;
    kalman_current_state.x[1] = pos.y;
    kalman_current_state.x[2] = pos.t;
    position_robot = pos;

    // Propager dans toute la FIFO pour éviter le rebond à la prochaine repropagate
    for (int i = 0; i < KALMAN_FIFO_LEN; i++) {
        kalman_fifo.buffer[i].x[0] = pos.x;
        kalman_fifo.buffer[i].x[1] = pos.y;
        kalman_fifo.buffer[i].x[2] = principal_angle(pos.t);
        // Remettre les vitesses à 0 et réinitialiser les observations
        kalman_fifo.buffer[i].x[3] = 0.0f;
        kalman_fifo.buffer[i].x[4] = 0.0f;
        kalman_fifo.buffer[i].x[5] = 0.0f;
        kalman_fifo.observations[i].has_lidar = 0;
        for (int c = 0; c < 3; c++)
            kalman_fifo.observations[i].has_camera[c] = 0;
        kalman_fifo.has_imu[i] = 0;
    }
    kalman_fifo.head  = 0;
    kalman_fifo.count = 0;
}

void set_position_x(float x) {
    kalman_current_state.x[0] = x;
    position_robot.x = x;
}

void set_position_y(float y) {
    kalman_current_state.x[1] = y;
    position_robot.y = y;
}

void set_position_t(float t) {
    kalman_current_state.x[2] = t;
    position_robot.t = t;
}


Position get_position(void) {
    return position_robot;
}

Speed get_speed(void) {
    return speed_robot_asserv;
}

Acceleration get_acceleration(void) {
    return acceleration_robot;
}
