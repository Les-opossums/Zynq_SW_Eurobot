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

/* Echantillon du log de calibration (cf control_get_calib_sample).
 * Meme disposition que le message CALIB_SAMPLE de l'ICD Ethernet
 * (opossum_interfaces/protocol.yaml) : ne pas modifier l'un sans l'autre.
 * Les grandeurs integrees (roues, gyro) ne perdent rien si un echantillon
 * n'est pas emis : l'analyse travaille sur leurs differences. */
typedef struct __attribute__((packed)) {
    uint32_t tick_ms;        /* compteur de pas rapides (1 ms) depuis control_init */
    float    wheel_dist[4];  /* distance parcourue par chaque roue (m), rayon nominal, signe odometrie */
    float    gyro_z;         /* derniere mesure gyro brute (rad/s) */
    float    gyro_angle;     /* integrale du gyro brut (rad, non repliee, biais compris) */
    uint32_t gyro_seq;       /* compteur de mesures gyro (imu_seq) */
    float    odo_x, odo_y, odo_t;        /* odometrie pure */
    float    kal_x, kal_y, kal_t;        /* estimation du Kalman */
    float    lidar_x, lidar_y, lidar_t;  /* derniere mesure lidar recue (SETLIDAR) */
    uint32_t lidar_delay;    /* retard annonce par cette mesure (ms) */
    uint32_t lidar_rx_ms;    /* tick_ms a sa reception */
    uint32_t lidar_count;    /* nombre de mesures lidar recues */
    uint8_t  motion_done;    /* 0 arrive, 1 en mouvement, 2 freine */
    uint8_t  flags;          /* bit0 : Kalman initialise, bit1 : arret d'urgence */
} Calib_sample;

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
