#include "driver_eth_io.h"
#include "../../IRQ_MANAGER/IRQ_manager.h"
#include "netif/xadapter.h"
#include "netif/xemacpsif.h" 
#include "xstatus.h"
#include "xil_printf.h"
#include "xemacps.h" 
#include "xparameters.h" // Ajout de l'en-tête pour lire les identifiants matériels (XPAR_...)

int ETH_IO_Init(void *instance) {
    eth_io_context_t *ctx = (eth_io_context_t *)instance;

    if (eth_driver_init(&ctx->config) != 0) {
        xil_printf("[ETH] Echec initialisation\n");
        return XST_FAILURE;
    }

    /* Connexion explicite de l'IRQ EMAC via notre IRQ_Manager commun. */
    xemacpsif_s *xemac = (xemacpsif_s *)(eth_driver_get_netif()->state);

    // Remplacement du champ inexistant par la constante matérielle de l'EMAC0
    int Status = IRQ_Manager_Connect(XPAR_XEMACPS_0_INTR,
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