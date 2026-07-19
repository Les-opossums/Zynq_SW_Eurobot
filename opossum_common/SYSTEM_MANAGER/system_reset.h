#ifndef SYSTEM_RESET_H
#define SYSTEM_RESET_H

#include "xil_types.h"

/**
 * @brief Redemarrage logiciel complet du Zynq : reset "chaud" du PS
 * (Processing System) entier via le registre SLCR PSS_RST_CTRL. Equivalent
 * logiciel d'un appui sur le bouton reset PS_SRST_B : les DEUX cœurs
 * (CPU0 + CPU1), le cache, la MMU et tous les peripheriques PS repartent
 * de zero, et le BootROM/FSBL s'executent a nouveau exactement comme a la
 * mise sous tension.
 *
 * Ne retourne jamais (le reset survient en quelques cycles d'horloge).
 * Peut etre appelee depuis n'importe quel cœur : le registre SLCR est un
 * registre materiel partage, pas une ressource propre a un cœur.
 */
void System_Reboot(void) __attribute__((noreturn));

/* Wrapper pour l'interpreteur de commandes (cf command_list.c). */
uint8_t Reboot_Cmd(void);

#endif // SYSTEM_RESET_H
