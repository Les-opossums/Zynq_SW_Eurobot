#include "main.h"

void create_frame(uint8_t *frame, uint8_t device_id, uint8_t cmd) {
    frame[0] = (PACKET_HEADER >> 24) & 0xFF;
    frame[1] = (PACKET_HEADER >> 16) & 0xFF;
    frame[2] = (PACKET_HEADER >> 8) & 0xFF;
    frame[3] = (PACKET_HEADER) & 0xFF;
    frame[4] = device_id;
    frame[5] = cmd;
    frame[6] = 0;
    frame[7] = 0;
    frame[8] = Compute_Checksum(frame, 8);
}

uint8_t Compute_Checksum(uint8_t *frame, uint8_t size) {
    uint8_t checksum = 0;
    for (uint8_t i = 4; i < size; i++) {
        checksum ^= frame[i];
    }
    return checksum;
}

void send_lidar_cmd(uint8_t cmd) {
    uint8_t frame[9];
    create_frame(frame, GS2_DEVICE1_ADDRESS, cmd);
    Send_Uart1_Buff_Cmd(frame, sizeof(frame));
}

    
