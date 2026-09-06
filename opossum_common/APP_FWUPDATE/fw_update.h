#ifndef FW_UPDATE_H
#define FW_UPDATE_H

#include "xil_types.h"

/*
 * Commande interpreteur "FWUPDATE <taille> <crc32>" (CPU0 uniquement).
 * Recoit un BOOT.bin par UART_COMM, l'ecrit en QSPI a l'offset 0, verifie le
 * CRC32 (donnees recues ET relecture flash), puis rend la main. Le host
 * enchaine avec "REBOOT". Cote host : zynq_ota.sh (role "send").
 *
 * Protocole (cf en-tete de zynq_ota.sh pour le detail) :
 *   host -> "FWUPDATE <taille_dec> <crc32_dec>\r"
 *   zynq -> "+READY <bloc>\n"           (apres effacement de la zone)
 *   [pour chaque bloc] host -> <octets bruts> ; zynq -> "+ACK <total>\n"
 *   zynq -> "+DONE\n"                    (image ecrite et CRC verifie)
 *   sinon zynq -> "-ERR <raison>\n"
 */
uint8_t FW_Update_Cmd(void);

#endif /* FW_UPDATE_H */
