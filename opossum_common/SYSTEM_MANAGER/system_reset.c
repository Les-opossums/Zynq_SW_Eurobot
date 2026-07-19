#include "system_reset.h"
#include "xil_io.h"
#include "xparameters.h"
#include "xil_printf.h"
#include "sleep.h"

/*
 * Registres SLCR (System Level Control Registers), cf Zynq-7000 TRM
 * (UG585) section B.28. Adresses fixees materiellement par le silicium :
 * independantes du block design / de l'export hardware, donc stables
 * quelle que soit la regeneration du BSP (contrairement aux XPAR_* du
 * bloc design). XPS_SYS_CTRL_BASEADDR (= 0xF8000000, alias SLCR) vient de
 * xparameters_ps.h, deja inclus transitivement via xparameters.h.
 */
#define SLCR_UNLOCK_OFFSET       0x008U
#define SLCR_PSS_RST_CTRL_OFFSET 0x200U
#define SLCR_UNLOCK_KEY          0xDF0DU

void System_Reboot(void) {
    xil_printf("\r\n[SYSTEM] Redemarrage demande : reset PS complet dans 100 ms...\r\n");
    usleep(100000); // laisse le temps a l'UART de vider le message ci-dessus

    // 1. Deverrouille les registres SLCR (proteges en ecriture par defaut)
    Xil_Out32(XPS_SYS_CTRL_BASEADDR + SLCR_UNLOCK_OFFSET, SLCR_UNLOCK_KEY);

    // 2. Demande un reset "chaud" complet du PS (CPU0 + CPU1, caches, MMU,
    // peripheriques) : equivalent logiciel d'un appui sur PS_SRST_B. Le
    // BootROM puis le FSBL redemarrent comme a la mise sous tension.
    Xil_Out32(XPS_SYS_CTRL_BASEADDR + SLCR_PSS_RST_CTRL_OFFSET, 0x1U);

    // Ne devrait jamais etre atteint : le reset survient en quelques
    // cycles d'horloge. Boucle de securite si jamais l'ecriture est
    // ignoree (ex: SLCR deja verrouille par un autre mecanisme).
    while (1) { }
}

uint8_t Reboot_Cmd(void) {
    System_Reboot();
    return 0; // jamais atteint
}
