#ifndef DRIVER_ETH_IO_H
#define DRIVER_ETH_IO_H

#include "ETH_driver.h"

/* ─── Debug ──────────────────────────────────────────────────────────────
 * Decommenter pour reactiver les prints de diagnostic de l'init ETH
 * (echec eth_driver_init(), echec connexion IRQ EMAC...). Desactive par
 * defaut : aucun cout en flash/CPU/UART en fonctionnement normal.
 */
// #define ETH_IO_DEBUG

#if defined(ETH_IO_DEBUG)
#include "xil_printf.h"
#define ETH_IO_LOG(...) xil_printf(__VA_ARGS__)
#else
#define ETH_IO_LOG(...) do {} while (0)
#endif

typedef struct {
    eth_driver_config_t config;
} eth_io_context_t;

int  ETH_IO_Init(void *instance);
void ETH_IO_Update(void *instance);

#endif /* DRIVER_ETH_IO_H */