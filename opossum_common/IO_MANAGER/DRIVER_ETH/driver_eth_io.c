#include "driver_eth_io.h"
#include "../../IRQ_MANAGER/IRQ_manager.h"
#include "netif/xadapter.h"
#include "xstatus.h"
#include "xil_printf.h"

int ETH_IO_Init(void *instance) {
    eth_io_context_t *ctx = (eth_io_context_t *)instance;

    if (eth_driver_init(&ctx->config) != 0) {
        xil_printf("[ETH] Echec initialisation\n");
        return XST_FAILURE;
    }

    /* Connexion explicite de l'IRQ EMAC via notre IRQ_Manager commun --
     * NE PAS laisser xemac_add()/le BSP gerer sa propre instance XScuGic.
     * Nom de struct/champ a confirmer selon ta version du portage lwIP
     * (voir netif/xadapter.h de ton BSP) : */
    xemacif_s *xemac = (xemacif_s *)(eth_driver_get_netif()->state);

    int Status = IRQ_Manager_Connect(xemac->emacps.Config.IntrId,
                                      (Xil_InterruptHandler)XEmacPs_IntrHandler,
                                      &xemac->emacps);
    if (Status != XST_SUCCESS) {
        xil_printf("[ETH] Erreur connexion IRQ EMAC\n");
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

void ETH_IO_Update(void *instance) {
    (void)instance;
    eth_driver_poll();
}