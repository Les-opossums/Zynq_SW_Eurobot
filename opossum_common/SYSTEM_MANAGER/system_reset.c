#include "system_reset.h"
#include "xil_io.h"
#include "xparameters.h"
#include "xil_printf.h"
#include "xpseudo_asm.h"   // dsb()
#include "sleep.h"

/*
 * Registres SLCR (System Level Control Registers), cf Zynq-7000 TRM
 * (UG585) section B.28. Adresses fixees materiellement par le silicium :
 * independantes du block design / de l'export hardware, donc stables
 * quelle que soit la regeneration du BSP (contrairement aux XPAR_* du
 * bloc design). XPS_SYS_CTRL_BASEADDR (= 0xF8000000, alias SLCR) vient de
 * xparameters_ps.h, deja inclus transitivement via xparameters.h.
 */
#define SLCR_UNLOCK_OFFSET         0x008U
#define SLCR_LQSPI_RST_CTRL_OFFSET 0x230U   // reset du controleur QSPI
#define SLCR_PSS_RST_CTRL_OFFSET   0x200U
#define SLCR_UNLOCK_KEY            0xDF0DU

void System_Reboot(void) {
    // NB : on n'imprime rien ici. Sur ce projet, xil_printf empile dans le
    // ring buffer TX vide uniquement par la boucle principale (cf
    // driver_uart_ps.c) ; or on ne va jamais y revenir. Un message serait
    // donc perdu, et pourrait retarder/parasiter le reset.
    usleep(2000); // laisse le dernier octet utile partir du FIFO hardware

    // 1. Deverrouille les registres SLCR (proteges en ecriture par defaut)
    Xil_Out32(XPS_SYS_CTRL_BASEADDR + SLCR_UNLOCK_OFFSET, SLCR_UNLOCK_KEY);

    // 2. Reset du controleur QSPI AVANT le soft reset.
    //    Indispensable apres un FWUPDATE : l'appli a laisse le controleur QSPI
    //    en mode manuel / CS force (cf APP_FWUPDATE/qspi_flash.c). Le soft
    //    reset PS seul ne le remet pas dans l'etat attendu par le BootROM, qui
    //    n'arrive alors plus a relire la flash au redemarrage -> carte muette,
    //    obligeant a un POR (debranchement). Un pulse sur LQSPI_RST_CTRL
    //    (bit0 = QSPI_REF_RST, bit1 = QSPI_CPU1X_RST) le remet a zero.
    Xil_Out32(XPS_SYS_CTRL_BASEADDR + SLCR_LQSPI_RST_CTRL_OFFSET, 0x3U); // assert
    for (volatile int i = 0; i < 10000; i++) { }                        // petit delai
    Xil_Out32(XPS_SYS_CTRL_BASEADDR + SLCR_LQSPI_RST_CTRL_OFFSET, 0x0U); // deassert
    dsb();

    // 3. Soft reset "chaud" complet du PS (CPU0 + CPU1, caches, MMU,
    //    peripheriques) : equivalent logiciel de PS_SRST_B. Le BootROM
    //    re-execute dans le mode de boot latche au dernier POR (QSPI ici) et
    //    recharge l'image depuis la flash.
    Xil_Out32(XPS_SYS_CTRL_BASEADDR + SLCR_PSS_RST_CTRL_OFFSET, 0x1U);

    // Ne devrait jamais etre atteint : le reset survient en quelques cycles.
    while (1) { }
}

uint8_t Reboot_Cmd(void) {
    System_Reboot();
    return 0; // jamais atteint
}
