#include "eth_interpreter_bridge.h"
#include "interpreteur.h"
#include "../IO_MANAGER/DRIVER_ETH/driver_eth_io.h"
#include "ETH_protocol.h"

static interp_ctx_t Eth_Interp_Ctx = {0};

static void ETH_Cmd_Handler(eth_msg_type_t type, const uint8_t *payload, uint16_t len) {
    if (type == ETH_MSG_CMD_GENERIC || type == ETH_MSG_RAW_TEXT) {
        for (uint16_t i = 0; i < len; i++) {
            Interp(&Eth_Interp_Ctx, (char)payload[i]);
        }
    }
    // Autres types (CMD_GOAL_POSITION, etc.) : dispatch vers des handlers
    // structures dedies, au fur et a mesure de leur portage.
}

void ETH_Interpreter_Bridge_Init(void) {
    eth_driver_set_cmd_handler(ETH_Cmd_Handler);
}