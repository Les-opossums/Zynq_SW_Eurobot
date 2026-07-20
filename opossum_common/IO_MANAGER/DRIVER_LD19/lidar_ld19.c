#include "lidar_ld19.h"
#include "xil_io.h"
#include "xil_cache.h"
#include "xstatus.h"
#include "xil_printf.h"
#include <math.h>

/* pi/18000 : conversion angle centiemes de degre -> radians. */
#define LIDAR_LD19_CDEG_TO_RAD 0.00017453293f

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

    /* Filtre distance active cote materiel (lidar_filter_regs) : coupe le
     * bruit tres proche et les points lointains parasites directement dans
     * le PL (cf LIDAR_LD19_FILTER_DIST_MIN/MAX_MM dans lidar_ld19.h) -- les
     * points hors plage arrivent avec distance=0 et sont donc deja ignores
     * par la suite. Mode nuage complet (pas de clustering). */
    Xil_Out32(ctx->regs_base + LIDAR_LD19_REG_DIST_MIN, LIDAR_LD19_FILTER_DIST_MIN_MM);
    Xil_Out32(ctx->regs_base + LIDAR_LD19_REG_DIST_MAX, LIDAR_LD19_FILTER_DIST_MAX_MM);
    Xil_Out32(ctx->regs_base + LIDAR_LD19_REG_CTRL, 0x1);
    Xil_Out32(ctx->regs_base + LIDAR_LD19_REG_CL_CTRL, 0);

    ctx->packet_count = 0;

    /* Amorce la reception du premier paquet. */
    status = lidar_ld19_arm_rx(ctx);
    if (status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    LIDAR_LD19_LOG("[LD19] Init OK (dma_device_id=%lu, regs_base=0x%08lX)\r\n",
                   (unsigned long)ctx->dma_device_id, (unsigned long)ctx->regs_base);

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
#if defined(LIDAR_LD19_DEBUG)
        /* Aucun paquet complet depuis le dernier armement : on lit les
         * compteurs de statut du parseur/UART (registres RO, cf
         * lidar_filter_regs.vhd) pour distinguer "rien ne sort du LD19/LD06"
         * (frames=0, uart_ferr=0 : probleme cablage/alimentation/UART) de
         * "des trames sont bien parsees mais le DMA ne se termine jamais"
         * (frames>0 mais toujours ce message : probleme cote AXI-Stream/DMA
         * dans le block design). */
        static uint32_t last_status_ms = 0;
        if ((uint32_t)Timer_ms1 - last_status_ms > 1000U) {
            last_status_ms = (uint32_t)Timer_ms1;
            uint32_t frame_count = Xil_In32(ctx->regs_base + LIDAR_LD19_REG_FRAME_COUNT);
            uint32_t error_count = Xil_In32(ctx->regs_base + LIDAR_LD19_REG_ERROR_COUNT);
            uint32_t speed       = Xil_In32(ctx->regs_base + LIDAR_LD19_REG_SPEED);
            uint32_t uart_ferr   = Xil_In32(ctx->regs_base + LIDAR_LD19_REG_UART_FERR);
            LIDAR_LD19_LOG("[LD19] DMA en attente... frames=%lu err=%lu speed=%lu deg/s uart_ferr=%lu\r\n",
                           (unsigned long)frame_count, (unsigned long)error_count,
                           (unsigned long)speed, (unsigned long)uart_ferr);
        }
#endif
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
    /* Format Teleplot (https://teleplot.fr) : chaque point est envoye comme
     * ">nom:x:y|xy" (xil_printf ne supporte pas %f -> conversion mm/rad
     * faite en float en interne, mais l'affichage reste entier, en mm).
     * Ouvrir la console UART dans Teleplot pour voir le nuage se dessiner
     * en direct. Un point a distance=0 est filtre/invalide : on ne le
     * trace pas. */
    static uint32_t last_print_ms = 0;
    if ((uint32_t)Timer_ms1 - last_print_ms > LIDAR_LD19_PRINT_PERIOD_MS) {
        last_print_ms = (uint32_t)Timer_ms1;
        for (int i = 0; i < LIDAR_LD19_POINTS_PER_PKT; i++) {
            uint16_t d = ctx->last_packet[i].distance_mm;
            uint16_t a = ctx->last_packet[i].angle_cdeg;
            if (d == 0U) {
                continue;
            }
            float angle_rad = (float)a * LIDAR_LD19_CDEG_TO_RAD;
            int32_t x_mm = (int32_t)((float)d * cosf(angle_rad));
            int32_t y_mm = (int32_t)((float)d * sinf(angle_rad));
            xil_printf(">lidar:%d:%d|xy\r\n", (int)x_mm, (int)y_mm);
        }
    }
#endif

    /* Reamorce immediatement la reception du paquet suivant. */
    (void)lidar_ld19_arm_rx(ctx);
}
