#ifndef ETH_PROTOCOL_H
#define ETH_PROTOCOL_H

#include <stdint.h>

/* ------------------------------------------------------------------------
 * Protocole Zynq <-> Raspberry Pi
 *
 * Meme philosophie que les trames JeVois (HEARTBEAT/ARUCO/ROBOTPOS) :
 * un header fixe court + payload type, en little-endian natif.
 * Zynq7000 PS (Cortex-A9) et Raspberry Pi (ARM) sont tous les deux
 * little-endian -> pas de htons/htonl. A revoir si un jour un des deux
 * cotes change d'architecture.
 * ------------------------------------------------------------------------ */

#define ETH_FRAME_MAGIC      0xC0DE
#define ETH_PROTOCOL_VERSION 1

#define ETH_DEBUG_PORT       5000   /* Zynq -> Pi, texte libre (printf-like)      */
#define ETH_TELEMETRY_PORT   5001   /* Zynq -> Pi, trames structurees periodiques */
#define ETH_CMD_PORT         5002   /* Pi -> Zynq, commandes                      */
#define ETH_RAW_CMD_PORT     5003   /* Pi -> Zynq, commandes brutes (debug)       */

#define ETH_MAX_PAYLOAD      512    /* marge large vs MTU 1500, a ajuster si besoin */

typedef enum {
    ETH_MSG_HEARTBEAT   = 0x01,
    ETH_MSG_DEBUG_TEXT  = 0x02,

    ETH_MSG_ODOM        = 0x10,
    ETH_MSG_IMU         = 0x11,
    ETH_MSG_MOTOR_STATE = 0x12,
    ETH_MSG_ROBOT_STATE = 0x13,

    ETH_MSG_CMD_GENERIC = 0x20,
} eth_msg_type_t;

typedef struct __attribute__((packed)) {
    uint16_t magic;        /* ETH_FRAME_MAGIC, permet de resynchroniser le flux */
    uint8_t  version;      /* ETH_PROTOCOL_VERSION */
    uint8_t  msg_type;     /* eth_msg_type_t */
    uint16_t seq;          /* compteur roulant, un par canal (debug/telemetry/cmd) */
    uint32_t timestamp_us; /* horodatage Zynq, cf eth_get_timestamp_us() */
    uint16_t payload_len;  /* longueur du payload qui suit */
    uint16_t crc16;        /* CRC16-CCITT sur (header, crc16 mis a 0) + payload */
} eth_frame_header_t;      /* 14 octets */

/* Payloads structures pour la telemetrie -- a etendre selon besoin */

typedef struct __attribute__((packed)) {
    float x, y, theta;
    float vx, vy, omega;
} eth_payload_odom_t;

typedef struct __attribute__((packed)) {
    float qw, qx, qy, qz;
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
} eth_payload_imu_t;

typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;
    uint8_t  state;   /* etat machine haut niveau si tu veux l'exposer */
    uint8_t  flags;   /* bitfield libre: match_started, emergency_stop, etc. */
} eth_payload_heartbeat_t;

#endif /* ETH_PROTOCOL_H */