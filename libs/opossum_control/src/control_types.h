#ifndef CONTROL_TYPES_H
#define CONTROL_TYPES_H

/* ------------------------------------------------------------------------
 * Types de la bibliotheque opossum_control (asservissement + Kalman).
 * Partages avec le reste du firmware via opossum_common/common_type.h.
 * C pur (stdint uniquement) : compile aussi sur PC (tests, simulation SIL).
 * ------------------------------------------------------------------------ */

#include <stdint.h>

/*****************************    Odometrie    *******************************/

// Position absolue du robot (x, y, et theta)
typedef struct __attribute__((packed)) {
    float x; // en metre
    float y; // en metre
    float t; // en radian
} Position;

// Vitesse et vitesse angulaire du robot
typedef struct __attribute__((packed)) {
    float vx; // en m/s
    float vy; // en m/s
    float vt; // en rad/s
} Speed;

// acceleration du robot (dv/dt,  d2theta/dt2   et   v*(dtheta/dt))
typedef struct __attribute__((packed)) {    float ax; // en m/s2
    float ay; // en m/s2
    float at; // en rad/s2
} Acceleration;


/**************************** PID  *****************************/
typedef struct __attribute__((packed)) {    float kp;
    float ki;
    float kd;
} PID_coef;

typedef struct __attribute__((packed)) {    float err;
    float err_int;
    float err_der;
} PID_err;
typedef struct __attribute__((packed)){
    PID_coef coef;
    PID_err err1;
    PID_err err2;
    PID_err err3;
    PID_err err4;
} PID_speed;

typedef struct __attribute__((packed)){
    float command1;
    float command2;
    float command3;
    float command4;
} ESC_Command;


typedef struct __attribute__((packed)){
    float lidar_position_x; // position of the robot according to the lidar
    float lidar_position_y; // position of the robot according to the lidar
    float lidar_position_t; // position of the robot according to the lidar
    uint32_t delay; // calculation delay in ms 
} Set_lidar;

typedef struct __attribute__((packed)){
    float camera_position_x; // position of the robot according to the camera
    float camera_position_y; // position of the robot according to the camera
    float camera_position_t; // position of the robot according to the camera
    uint32_t delay; // calculation delay in ms
    float noise_x; // estimation of the noise on the x measurement (standard deviation in m)
    float noise_y; // estimation of the noise on the y measurement (standard deviation in m)
    float noise_t; // estimation of the noise on the theta measurement (standard deviation in rad)
} Set_camera;

typedef struct __attribute__((packed)){
    float process_noise_lidar_x; // estimation of the process noise on the x measurement from the lidar (standard deviation in m)
    float process_noise_lidar_y; // estimation of the process noise on the y measurement from the lidar (standard deviation in m)
    float process_noise_lidar_t; // estimation of the process noise on the theta measurement from
} Set_lidar_noise;

typedef struct __attribute__((packed)){
    int enable_lidar_kalman; // 1 to take into account the lidar measurements in the kalman, 0 to ignore them
    int enable_camera_kalman; // 1 to take into account the camera measurements in the kalman, 0 to ignore them
} Enable_Kalman;

#endif /* CONTROL_TYPES_H */
