#include "lidar_ld19.h"
#include "xil_io.h"
#include "xil_cache.h"
#include "xstatus.h"
#include "xil_printf.h"

/* Horloge milliseconde partagee, entretenue par la boucle 1ms (fast loop). */
extern volatile u32 Timer_ms1;

/* ==================================================================
 * HELPER : (re)armement d'une reception DMA S2MM d'un paquet complet
 * ================================================================== */
static int lidar_ld19_arm_rx(lidar_ld19_context_t *ctx)
{
    int status = XAxiDma_SimpleTransfer(&ctx->axi_dma, (UINTPTR)ctx->rx_raw,
                                         LIDAR_LD19_POINTS_PER_PKT * sizeof(uint32_t),
                                         XAXIDMA_DEVICE_TO_DMA);
    if (status != XST_SUCCESS) {
        LIDAR_LD19_LOG("[LD19] Armement reception DMA echoue (%d)\r\n", status);
    }
    return status;
}

/* ==================================================================
 * 1. Initialisation du driver
 * ================================================================== */
int LIDAR_LD19_Init(void *instance)
{
    lidar_ld19_context_t *ctx = (lidar_ld19_context_t *)instance;

    XAxiDma_Config *dma_cfg = XAxiDma_LookupConfig(ctx->dma_device_id);
    if (dma_cfg == NULL) {
        LIDAR_LD19_LOG("[LD19] Config AXI DMA introuvable (device_id=%lu)\r\n",
                       (unsigned long)ctx->dma_device_id);
        return XST_FAILURE;
    }

    int status = XAxiDma_CfgInitialize(&ctx->axi_dma, dma_cfg);
    if (status != XST_SUCCESS) {
        LIDAR_LD19_LOG("[LD19] XAxiDma_CfgInitialize a echoue (%d)\r\n", status);
        return status;
    }

    if (XAxiDma_HasSg(&ctx->axi_dma)) {
        /* Le hardware a ete genere en mode Scatter-Gather : ce driver ne
         * gere que le mode Direct Register (Simple Transfer). */
        LIDAR_LD19_LOG("[LD19] AXI DMA en mode Scatter-Gather, non supporte\r\n");
        return XST_FAILURE;
    }

    /* Configuration par defaut du filtre/clustering (cf lidar_filter_regs.vhd) :
     * filtre inactif (on veut voir le nuage brut pour ce premier test) et
     * mode nuage complet (pas de clustering). Ce sont deja les valeurs de
     * reset materiel, on les reecrit explicitement par robustesse. */
    Xil_Out32(ctx->regs_base + LIDAR_LD19_REG_CTRL, 0);
    Xil_Out32(ctx->regs_base + LIDAR_LD19_REG_CL_CTRL, 0);

    ctx->packet_count = 0;

    /* Amorce la reception du premier paquet. */
    status = lidar_ld19_arm_rx(ctx);
    if (status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

/* ==================================================================
 * 2. Mise a jour periodique (appelee par l'IO_Manager)
 *    Paquet complet (12 points) recu par DMA -> decodage -> print
 *    (throttle) -> reamorcage immediat de la reception suivante.
 * ================================================================== */
void LIDAR_LD19_Update(void *instance)
{
    lidar_ld19_context_t *ctx = (lidar_ld19_context_t *)instance;

    if (XAxiDma_Busy(&ctx->axi_dma, XAXIDMA_DEVICE_TO_DMA)) {
        return; /* paquet pas encore complet */
    }

    /* Le CPU peut avoir en cache une version perimee de ce que le DMA
     * vient d'ecrire en DDR : invalidation obligatoire avant lecture. */
    Xil_DCacheInvalidateRange((INTPTR)ctx->rx_raw, sizeof(ctx->rx_raw));

    for (int i = 0; i < LIDAR_LD19_POINTS_PER_PKT; i++) {
        uint32_t raw = ctx->rx_raw[i];
        ctx->last_packet[i].distance_mm = (uint16_t)(raw >> 16);
        ctx->last_packet[i].angle_cdeg  = (uint16_t)(raw & 0xFFFFU);
    }
    ctx->packet_count++;

#if LIDAR_LD19_PRINT_POINTS
    static uint32_t last_print_ms = 0;
    if ((uint32_t)Timer_ms1 - last_print_ms > LIDAR_LD19_PRINT_PERIOD_MS) {
        last_print_ms = (uint32_t)Timer_ms1;
        xil_printf("[LD19] paquet #%lu :\r\n", (unsigned long)ctx->packet_count);
        for (int i = 0; i < LIDAR_LD19_POINTS_PER_PKT; i++) {
            uint16_t d = ctx->last_packet[i].distance_mm;
            uint16_t a = ctx->last_packet[i].angle_cdeg;
            if (d == 0U) {
                xil_printf("  pt%2d : invalide (filtre)\r\n", i);
            } else {
                xil_printf("  pt%2d : dist=%5u mm  angle=%3u.%02u deg\r\n",
                           i, d, a / 100U, a % 100U);
            }
        }
    }
#endif

    /* Reamorce immediatement la reception du paquet suivant. */
    (void)lidar_ld19_arm_rx(ctx);
}
