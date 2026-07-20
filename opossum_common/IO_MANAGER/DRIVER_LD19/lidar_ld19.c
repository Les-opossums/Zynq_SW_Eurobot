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
 * HELPER : accumulation d'un point dans le scan en cours de construction,
 * avec detection de fin de tour (wraparound d'angle) et bascule du
 * double-buffer. Appelee pour chaque point recu, valide ou non (un point
 * filtre a distance=0 ne rentre pas dans le scan, mais participe quand
 * meme a la detection de tour puisque son angle reste correct).
 * ================================================================== */
static void lidar_ld19_accumulate_point(lidar_ld19_context_t *ctx, uint16_t distance_mm, uint16_t angle_cdeg)
{
    lidar_scan_t *building = &ctx->scan_buf[ctx->building_idx];

    /* Fin de tour : l'angle vient de sauter fortement en arriere
     * (35999 -> ~0). On ignore le tout premier point recu depuis le
     * demarrage (has_last_angle==0) pour ne pas declencher une fausse fin
     * de tour avant meme d'avoir un historique d'angle. */
    if (ctx->has_last_angle && angle_cdeg < ctx->last_angle_cdeg &&
        (uint16_t)(ctx->last_angle_cdeg - angle_cdeg) > LIDAR_LD19_WRAP_THRESHOLD_CDEG) {

        building->timestamp_ms = (uint32_t)Timer_ms1;
        building->scan_id       = ctx->next_scan_id++;

        /* Le scan qu'on vient de terminer devient le scan "pret" ; on
         * bascule sur l'autre buffer pour commencer le tour suivant. */
        ctx->ready_idx    = ctx->building_idx;
        ctx->building_idx = (uint8_t)(1U - ctx->building_idx);

        building = &ctx->scan_buf[ctx->building_idx];
        building->count = 0U; /* nouveau tour : on repart de zero */
    }

    ctx->has_last_angle  = 1U;
    ctx->last_angle_cdeg = angle_cdeg;

    if (distance_mm != 0U && building->count < LIDAR_LD19_MAX_POINTS_PER_SCAN) {
        building->points[building->count].distance_mm = distance_mm;
        building->points[building->count].angle_cdeg  = angle_cdeg;
        building->count++;
    }
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

    /* Scan / double-buffer : aucun tour complet encore recu. */
    ctx->scan_buf[0].count   = 0U;
    ctx->scan_buf[0].scan_id = 0U;
    ctx->scan_buf[1].count   = 0U;
    ctx->scan_buf[1].scan_id = 0U;
    ctx->building_idx   = 0U;
    ctx->ready_idx       = 0U;
    ctx->has_last_angle  = 0U;
    ctx->last_angle_cdeg = 0U;
    ctx->next_scan_id    = 1U; /* 0 reserve pour "aucun scan" (cf lidar_ld19.h) */

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
        uint16_t d   = (uint16_t)(raw >> 16);
        uint16_t a   = (uint16_t)(raw & 0xFFFFU);
        ctx->last_packet[i].distance_mm = d;
        ctx->last_packet[i].angle_cdeg  = a;

        lidar_ld19_accumulate_point(ctx, d, a);
    }
    ctx->packet_count++;

#if LIDAR_LD19_PRINT_POINTS
    /* --- Ancien mode : streaming continu paquet par paquet -------------
     * Format Teleplot (https://teleplot.fr) : chaque point envoye est
     * ">nom:x:y|xy" (xil_printf ne supporte pas %f -> conversion mm/rad
     * faite en float en interne, affichage entier en mm). Decimation
     * globale et continue (compteur qui ne se remet jamais a zero par
     * paquet) : evite l'effet "paquet de points puis trou" d'un throttle
     * temporel, pour un balayage qui avance regulierement a l'ecran.
     * Un point a distance=0 est filtre/invalide : on ne le trace pas (et
     * ne compte pas dans la decimation, pour ne pas gaspiller de cran sur
     * du vide).
     * Gardee ici en reference/secours : decommenter (et commenter le
     * nouveau mode juste en dessous) pour revenir a l'ancien comportement.
     *
    static uint32_t point_seq = 0;
    for (int i = 0; i < LIDAR_LD19_POINTS_PER_PKT; i++) {
        uint16_t d = ctx->last_packet[i].distance_mm;
        uint16_t a = ctx->last_packet[i].angle_cdeg;
        if (d == 0U) {
            continue;
        }
        point_seq++;
        if ((point_seq % LIDAR_LD19_PRINT_DECIMATION) != 0U) {
            continue;
        }
        float angle_rad = (float)a * LIDAR_LD19_CDEG_TO_RAD;
        int32_t x_mm = (int32_t)((float)d * cosf(angle_rad));
        int32_t y_mm = (int32_t)((float)d * sinf(angle_rad));
        xil_printf(">lidar:%d:%d|xy\r\n", (int)x_mm, (int)y_mm);
    }
    * --- fin ancien mode --- */

    /* --- Nouveau mode : envoi du scan complet (tour a 360 deg) des qu'il
     * vient de se terminer, plutot qu'un flux continu paquet par paquet.
     * scan_id permet de ne declencher l'envoi qu'une seule fois par tour
     * (et pas a chaque paquet tant que le scan "pret" n'a pas change).
     * Decimation dediee (LIDAR_LD19_SCAN_TELEPLOT_DECIMATION) car ici tout
     * part en une seule rafale au lieu d'etre lisse dans le temps.
     * Seul le nuage de points est envoye sur l'UART (pas de scan_id/
     * timestamp en traces numeriques Teleplot, pour ne pas polluer la
     * console) : scan_id/timestamp_ms restent disponibles cote code via
     * LIDAR_LD19_GetLastScan() pour qui en a besoin. */
    static uint32_t last_sent_scan_id = 0;
    const lidar_scan_t *scan = LIDAR_LD19_GetLastScan(ctx);
    if (scan != NULL && scan->scan_id != last_sent_scan_id) {
        last_sent_scan_id = scan->scan_id;

        for (uint16_t i = 0; i < scan->count; i += LIDAR_LD19_SCAN_TELEPLOT_DECIMATION) {
            uint16_t d = scan->points[i].distance_mm;
            uint16_t a = scan->points[i].angle_cdeg;
            float angle_rad = (float)a * LIDAR_LD19_CDEG_TO_RAD;
            int32_t x_mm = (int32_t)((float)d * cosf(angle_rad));
            int32_t y_mm = (int32_t)((float)d * sinf(angle_rad));
            xil_printf(">lidar:%d:%d|xy\r\n", (int)x_mm, (int)y_mm);
        }
    }
    /* --- fin nouveau mode --- */
#endif

    /* Reamorce immediatement la reception du paquet suivant. */
    (void)lidar_ld19_arm_rx(ctx);
}

/* ==================================================================
 * 3. API de consultation du dernier scan complet (cf lidar_ld19.h)
 * ================================================================== */
const lidar_scan_t *LIDAR_LD19_GetLastScan(lidar_ld19_context_t *ctx)
{
    if (ctx->scan_buf[ctx->ready_idx].scan_id == 0U) {
        return NULL; /* aucun tour complet recu depuis le demarrage */
    }
    return &ctx->scan_buf[ctx->ready_idx];
}
