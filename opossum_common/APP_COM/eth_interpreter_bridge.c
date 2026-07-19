#include "eth_interpreter_bridge.h"
#include "interpreteur.h"
#include "../IO_MANAGER/DRIVER_ETH/driver_eth_io.h"
#include "ETH_protocol.h"
#include "../IPC_MANAGER/IPC_manager.h"

static interp_ctx_t Eth_Interp_Ctx = {0};

/**
 * @brief Callback appele par le driver Ethernet (depuis eth_driver_poll(),
 * donc en contexte boucle principale CPU0, jamais en ISR) quand une trame
 * valide arrive sur le canal CMD.
 *
 * Port de l'ancien handler `on_eth_command_received` : les commandes texte
 * repassent par l'interpreteur habituel (meme logique que l'UART), les
 * commandes structurees (binaire) sont transmises directement a CORE1 via
 * IPC, sans repasser par le parseur ASCII.
 */
static void ETH_Cmd_Handler(eth_msg_type_t type, const uint8_t *payload, uint16_t len) {
    switch ((uint8_t)type) {

    /* --- 1. Commandes texte (debug humain / anciens scripts) --- */
    case ETH_MSG_CMD_GENERIC:
    case ETH_MSG_RAW_TEXT:
        for (uint16_t i = 0; i < len; i++) {
            Interp(&Eth_Interp_Ctx, (char)payload[i]);
        }
        // Comme sur UART : si la trame n'est pas deja terminee par un saut
        // de ligne, on force la validation de la commande en cours.
        if (len > 0 && payload[len - 1] != '\n' && payload[len - 1] != '\r') {
            Interp(&Eth_Interp_Ctx, '\n');
        }
        break;

    /* --- 2. Commandes structurees : envoi direct vers CORE1 via IPC --- */
    case ETH_MSG_CMD_GOAL_POSITION:
        if (len == sizeof(Position)) {
            IPC_SendToOtherCore(payload, sizeof(Position),
                                 &IPC_DATA->cmd_position,
                                 &IPC_DATA->flag_cmd_position_valid,
                                 &IPC_DATA->flag_cmd_position_ack);
        }
        break;

    case ETH_MSG_CMD_SET_LIDAR:
        if (len == sizeof(Set_lidar)) {
            IPC_SendToOtherCore(payload, sizeof(Set_lidar),
                                 &IPC_DATA->set_lidar,
                                 &IPC_DATA->flag_set_lidar_valid,
                                 &IPC_DATA->flag_set_lidar_ack);
        }
        break;

    case ETH_MSG_CMD_SET_CAMERA_1:
        if (len == sizeof(Set_camera)) {
            IPC_SendToOtherCore(payload, sizeof(Set_camera),
                                 &IPC_DATA->set_camera_1,
                                 &IPC_DATA->flag_set_camera_1_valid,
                                 &IPC_DATA->flag_set_camera_1_ack);
        }
        break;

    case ETH_MSG_CMD_SET_CAMERA_2:
        if (len == sizeof(Set_camera)) {
            IPC_SendToOtherCore(payload, sizeof(Set_camera),
                                 &IPC_DATA->set_camera_2,
                                 &IPC_DATA->flag_set_camera_2_valid,
                                 &IPC_DATA->flag_set_camera_2_ack);
        }
        break;

    case ETH_MSG_CMD_SET_CAMERA_3:
        if (len == sizeof(Set_camera)) {
            IPC_SendToOtherCore(payload, sizeof(Set_camera),
                                 &IPC_DATA->set_camera_3,
                                 &IPC_DATA->flag_set_camera_3_valid,
                                 &IPC_DATA->flag_set_camera_3_ack);
        }
        break;

    case ETH_MSG_CMD_SET_SPEED_MAX:
        // Pas cable : IPC_DATA->vmax est un simple float (vitesse lineaire
        // max), pas une structure Speed -- le format de payload cote
        // Raspberry n'est pas encore defini pour cette commande (deja
        // desactive pour la meme raison dans l'ancien firmware). A cabler
        // une fois le format retenu, ex :
        //   if (len == sizeof(float)) {
        //       IPC_SendToOtherCore(payload, sizeof(float), &IPC_DATA->vmax,
        //                            &IPC_DATA->flag_vmax_valid, &IPC_DATA->flag_vmax_ack);
        //   }
        break;

    case ETH_MSG_CMD_BLOCK:
        if (len == sizeof(uint8_t)) {
            int block_mode = 4; // asserv_mode = 4 : bloque le robot
            IPC_SendToOtherCore(&block_mode, sizeof(block_mode),
                                 &IPC_DATA->asserv_mode,
                                 &IPC_DATA->flag_asserv_mode_valid,
                                 &IPC_DATA->flag_asserv_mode_ack);
        }
        break;

    case ETH_MSG_CMD_FREE:
        if (len == sizeof(uint8_t)) {
            int free_mode = 0; // asserv_mode = 0 : libere le robot
            IPC_SendToOtherCore(&free_mode, sizeof(free_mode),
                                 &IPC_DATA->asserv_mode,
                                 &IPC_DATA->flag_asserv_mode_valid,
                                 &IPC_DATA->flag_asserv_mode_ack);
        }
        break;

    default:
        break;
    }
}

void ETH_Interpreter_Bridge_Init(void) {
    eth_driver_set_cmd_handler(ETH_Cmd_Handler);
}