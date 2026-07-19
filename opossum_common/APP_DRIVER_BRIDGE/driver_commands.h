#ifndef DRIVER_COMMANDS_H
#define DRIVER_COMMANDS_H

#include "xil_types.h"

/**
 * @brief Commandes generiques de pilotage des peripheriques IO_Manager,
 * utilisables sur n'importe quel peripherique de IO_DEVICE_TABLE (cf
 * IO_config.h), quel que soit le driver concerne (FEETECH, WS2812B,
 * ETHERNET...). Utile par exemple pour couper le bus FEETECH ou le bandeau
 * LED sans reflasher, ou pour verifier rapidement l'etat de tous les
 * drivers d'un cœur.
 *
 * Syntaxe (le nom est celui du champ .name dans IO_DEVICE_TABLE,
 * insensible a la casse, entre guillemets — convention de
 * Get_Param_String) :
 *   DRVEN "FEETECH"
 *   DRVDIS "FEETECH"
 *   DRVLIST
 */
uint8_t Driver_Enable_Cmd(void);
uint8_t Driver_Disable_Cmd(void);
uint8_t Driver_List_Cmd(void);

#endif // DRIVER_COMMANDS_H
