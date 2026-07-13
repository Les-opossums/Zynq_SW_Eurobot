#include "main.h"

// Fonction appelée par le driver Ethernet quand une commande valide est reçue
void on_eth_command_received(eth_msg_type_t type, const uint8_t *payload, uint16_t len) {
    
    switch ((uint8_t)type) {
        // 1. LES COMMANDES TEXTES (Debug humain ou vieux scripts)
        case ETH_MSG_CMD_GENERIC:
        case ETH_MSG_RAW_TEXT:
            for(uint16_t i=0; i < len; i++) {
                Interp((char)payload[i]);
            }
            if (len > 0 && payload[len-1] != '\n' && payload[len-1] != '\r') {
                Interp('\n');
            }
            break;

        // 2. LES COMMANDES STRUCTUREES
        case ETH_MSG_CMD_GOAL_POSITION:
            if (len == sizeof(Position)) {
                Position *consigne_reseau = (Position *)payload;
                send_to_other_core(
                    consigne_reseau,
                    sizeof(Position),
                    &(shared_mem->cmd_position),
                    &(shared_mem->flag_cmd_position_valid),
                    &(shared_mem->flag_cmd_position_ack)
                );
            }
            break;

        case ETH_MSG_CMD_SET_LIDAR:
            if (len == sizeof(Set_lidar)) {
                Set_lidar *lidar_data = (Set_lidar *)payload;
                send_to_other_core(
                    lidar_data,
                    sizeof(Set_lidar),
                    &(shared_mem->set_lidar),
                    &(shared_mem->flag_set_lidar_valid),
                    &(shared_mem->flag_set_lidar_ack)
                );
            }
            break;

        case ETH_MSG_CMD_SET_CAMERA_1:
            if (len == sizeof(Set_camera)) {
                Set_camera *camera_data = (Set_camera *)payload;
                send_to_other_core(
                    camera_data,
                    sizeof(Set_camera),
                    &(shared_mem->set_camera_1),
                    &(shared_mem->flag_set_camera_1_valid),
                    &(shared_mem->flag_set_camera_1_ack)
                );
            }
            break;

        case ETH_MSG_CMD_SET_CAMERA_2:
            if (len == sizeof(Set_camera)) {
                Set_camera *camera_data = (Set_camera *)payload;
                send_to_other_core(
                    camera_data,
                    sizeof(Set_camera),
                    &(shared_mem->set_camera_2),
                    &(shared_mem->flag_set_camera_2_valid),
                    &(shared_mem->flag_set_camera_2_ack)
                );
            }
            break;

        case ETH_MSG_CMD_SET_CAMERA_3:
            if (len == sizeof(Set_camera)) {
                Set_camera *camera_data = (Set_camera *)payload;
                send_to_other_core(
                    camera_data,
                    sizeof(Set_camera),
                    &(shared_mem->set_camera_3),
                    &(shared_mem->flag_set_camera_3_valid),
                    &(shared_mem->flag_set_camera_3_ack)
                );
            }
            break;

        case ETH_MSG_CMD_SET_SPEED_MAX:
            // if (len == sizeof(Speed)) {
            //     Speed *speed_data = (Speed *)payload;
            //     send_to_other_core(
            //         speed_data,
            //         sizeof(Speed),
            //         &(shared_mem->cmd_speed),
            //         &(shared_mem->flag_cmd_speed_valid),
            //         &(shared_mem->flag_cmd_speed_ack)
            //     );
            // }
            break;

        case ETH_MSG_CMD_BLOCK:
            if (len == sizeof(uint8_t)) {
                uint8_t block_data = 4; // asser_mode = 4 pour bloquer le robot
                    send_to_other_core(
                    &block_data,
                    sizeof(uint8_t),
                    &(shared_mem->asserv_mode),
                    &(shared_mem->flag_asserv_mode_valid),
                    &(shared_mem->flag_asserv_mode_ack)
                );
            }
            break;

        case ETH_MSG_CMD_FREE:
            if (len == sizeof(uint8_t)) {
                uint8_t free_data = 0; // asser_mode = 0 pour free le robot
                    send_to_other_core(
                    &free_data,
                    sizeof(uint8_t),
                    &(shared_mem->asserv_mode),
                    &(shared_mem->flag_asserv_mode_valid),
                    &(shared_mem->flag_asserv_mode_ack)
                );
            }
            break;

    }
}