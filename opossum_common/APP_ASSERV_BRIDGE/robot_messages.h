
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

/* Etat de l'arret d'urgence (ETH_MSG_AU, 0x14). Emis sur chaque changement
 * d'etat (cf core0_loop.c).
 * ATTENTION : `state` DOIT rester le 1er octet du payload. Le node ROS2
 * (opossum_comm/comm_node.cpp) lit l'etat via payload[0] ("AU " + payload[0]),
 * il n'interprete pas le timestamp. Ne pas remettre le timestamp devant. */
typedef struct __attribute__((packed)) {
    uint8_t  state;        /* payload[0] : 0 = AU relache, 1 = AU appuye */
    uint32_t timestamp_ms;
} eth_payload_au_t;

/* Etat de la laisse (ETH_MSG_LEASH, 0x15). Emis sur chaque changement d'etat
 * (cf core0_loop.c).
 * ATTENTION : `state` DOIT rester le 1er octet du payload. comm_node.cpp lit
 * payload[0] et ne publie le message ROS "LEASH" (depart du match) que si
 * payload[0] == 1. */
typedef struct __attribute__((packed)) {
    uint8_t  state;        /* payload[0] : 0 = laisse relachee, 1 = laisse attachee/tiree */
    uint32_t timestamp_ms;
} eth_payload_leash_t;


#endif /* ROBOT_MESSAGES_H */