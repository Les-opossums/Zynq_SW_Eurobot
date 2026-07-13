#ifndef ROBOT_MESSAGES_H
#define ROBOT_MESSAGES_H

#include <stdint.h>

/* * Ce fichier definit UNIQUEMENT les structures de donnees du robot.
 * Ces structures DOIVENT etre synchronisees (identiques) entre le Zynq et le Raspberry Pi.
 */

typedef struct __attribute__((packed)) {
    float x;
    float y;
    float theta;
    float vx;
    float vy;
    float omega;
} eth_payload_odom_t;

typedef struct __attribute__((packed)) {
    float qw, qx, qy, qz;
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
} eth_payload_imu_t;

typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;
    uint8_t  state;
    uint8_t  flags;
} eth_payload_heartbeat_t;

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    float x, y, theta;
    float speed_linear, speed_direction, speed_angular;
    uint8_t motion_done;
} eth_payload_robot_state_t;


#endif /* ROBOT_MESSAGES_H */