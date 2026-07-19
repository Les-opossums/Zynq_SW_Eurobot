#include "driver_eth_io.h"
#include "../../IRQ_MANAGER/IRQ_manager.h"
#include "netif/xadapter.h"
#include "netif/xemacpsif.h"
#include "xstatus.h"
#include "xemacps.h"
#include "xparameters.h" // Ajout de l'en-tête pour lire les identifiants matériels (XPAR_...)

/*
 * netif->state ne pointe PAS directement vers la structure xemacpsif_s
 * (celle qui contient le champ .emacps) : il pointe vers un "struct xemac_s"
 * (wrapper generique lwIP/Xilinx, cf netif/xadapter.h), dont le CHAMP .state
 * a lui pointe vers le vrai xemacpsif_s (cf xemacpsif.c: "netif->state =
 * (void *)xemac;" ou xemac est un struct xemac_s*, et xemacpsif.c utilise
 * ensuite "xemacpsif = (xemacpsif_s *)(xemac->state)" en interne, voir
 * HandleEmacPsError()).
 *
 * Le code prenait directement netif->state pour un xemacpsif_s*, ce qui
 * calculait &xemac->emacps sur le mauvais layout memoire : le pointeur
 * passe a XEmacPs_IntrHandler() ne visait donc pas la vraie instance
 * XEmacPs initialisee par XEmacPs_CfgInitialize(), d'ou l'assertion
 * "InstancePtr->IsReady == XIL_COMPONENT_IS_READY" qui echouait des la
 * premiere interruption EMAC (trouve via bt : Xil_Assert() <-
 * XEmacPs_IntrHandler():131 <- XScuGic_InterruptHandler() <- IRQHandler()).
 */
static xemacpsif_s *eth_get_xemacpsif(void) {
    struct xemac_s *xemac = (struct xemac_s *)(eth_driver_get_netif()->state);
    return (xemacpsif_s *)(xemac->state);
}

int ETH_IO_Init(void *instance) {
    eth_io_context_t *ctx = (eth_io_context_t *)instance;

    if (eth_driver_init(&ctx->config) != 0) {
        ETH_IO_LOG("[ETH] Echec initialisation\n");
        return XST_FAILURE;
    }

    /* Connexion explicite de l'IRQ EMAC via notre IRQ_Manager commun. */
    xemacpsif_s *xemacpsif = eth_get_xemacpsif();

    // Remplacement du champ inexistant par la constante matérielle de l'EMAC0
    int Status = IRQ_Manager_Connect(XPAR_XEMACPS_0_INTR,
                                      (Xil_InterruptHandler)XEmacPs_IntrHandler,
                                      &xemacpsif->emacps);
    if (Status != XST_SUCCESS) {
        ETH_IO_LOG("[ETH] Erreur connexion IRQ EMAC\n");
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

void ETH_IO_Update(void *instance) {
    (void)instance;
    eth_driver_poll();
}