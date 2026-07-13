#ifndef ETH_RECEIVER_H
#define ETH_RECEIVER_H

void on_eth_command_received(eth_msg_type_t type, const uint8_t *payload, uint16_t len);

#endif