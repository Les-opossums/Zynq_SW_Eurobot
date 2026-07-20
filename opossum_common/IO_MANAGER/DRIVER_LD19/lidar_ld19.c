#include "lidar_ld19.h"
#include "xil_io.h"
#include "xil_cache.h"
#include "xstatus.h"
#include "xil_printf.h"
#include "../DRIVER_ETH/ETH_driver.h"
#include <math.h>

/* pi/18000 : conversion angle centiemes de degre -> radians. */
#define LIDAR_LD19_CDEG_TO_RAD 0.00017453293f

/* Horloge milliseconde partagee, entretenue par la boucle 1ms (fast loop). */
extern volatile u32 Timer_ms1;

/* Nombre d'essais bornes en boucle d'attente de fin de reset DMA (utilise
 * a l'init, en cas d'erreur IRQ, et au deinit) : le reset est cense etre
 * quasi-immediat (meme ordre de grandeur que la boucle interne de
 * XAxiDma_CfgInitialize), une borne large evite tout blocage infini en cas
 * de souci materiel. */
#define LIDAR_LD19_RESET_MAX_TRIES 10000

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

/* Reset DMA + attente bornee de fin de reset (cf lidar_ld19_arm_rx pour la
 * suite, appelee juste apres par l'appelant). Factorise entre l'ISR
 * (erreur DMA) et LIDAR_LD19_Deinit(). */
static void lidar_ld19_reset_dma(lidar_ld19_context_t *ctx)
{
    XAxiDma_Reset(&ctx->axi_dma);
    int tries = 0;
    while (!XAxiDma_ResetIsDone(&ctx->axi_dma) && tries < LIDAR_LD19_RESET_MAX_TRIES) {
        tries++;
    }
}

/* ==================================================================
 * HELPER : accumulation d'un point dans le scan en cours de construction,
 * avec detection de fin de tour (wraparound d'angle) et bascule du
 * double-buffer. Appelee pour chaque point recu, valide ou non (un point
 * filtre a distance=0 ne rentre pas dans le scan, mais participe quand
 * meme a la detection de tour puisque son angle reste correct).
 *
 * Appelee depuis l'ISR de fin de transfert DMA (cf LIDAR_LD19_IntrHandler) :
 * ne fait que des acces memoire simples, pas d'E/S -- ISR-safe.
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

/* Decodage d'un paquet DMA brut (12 mots) + accumulation dans le scan en
 * cours. Appelee depuis l'ISR (IOC) : invalidation de cache + acces
 * memoire uniquement, pas d'E/S -- ISR-safe. */
static void lidar_ld19_process_packet(lidar_ld19_context_t *ctx)
{
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
     * par la suite. Mode nuage complet (pas de clustering). Reglable a
     * chaud ensuite via LIDAR_LD19_SetConfig(). */
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

    ctx->last_consumed_scan_id = 0U;
    /* eth_streaming_enabled n'est PAS remis a 0 ici : un DRVDIS/DRVEN (cf
     * IO_Manager_SetDeviceStateByName) rappelle Init() et ne doit pas
     * silencieusement couper un streaming deja active par l'application. */

    /* Interruption de fin de transfert DMA (IOC) + erreurs -- le polling
     * XAxiDma_Busy() a ete remplace par ce mecanisme (cf
     * LIDAR_LD19_IntrHandler, branchee via IO_DEVICE_TABLE dans
     * IO_config.h). */
    XAxiDma_IntrEnable(&ctx->axi_dma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);

    /* Amorce la reception du premier paquet. */
    status = lidar_ld19_arm_rx(ctx);
    if (status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    LIDAR_LD19_LOG("[LD19] Init OK (id=%u, dma_device_id=%lu, regs_base=0x%08lX)\r\n",
                   (unsigned)ctx->lidar_id, (unsigned long)ctx->dma_device_id,
                   (unsigned long)ctx->regs_base);

    return XST_SUCCESS;
}

/* ==================================================================
 * 2. Handler d'interruption (fin de transfert DMA S2MM)
 *    Travail "temps reel" uniquement : ack IRQ, gestion d'erreur (reset),
 *    decodage + accumulation, reamorcage. Aucune E/S (xil_printf/
 *    eth_send_frame) : c'est LIDAR_LD19_Update(), appelee depuis la
 *    boucle principale, qui s'en charge (cf plus bas).
 * ================================================================== */
void LIDAR_LD19_IntrHandler(void *CallBackRef)
{
    lidar_ld19_context_t *ctx = (lidar_ld19_context_t *)CallBackRef;

    u32 irq_status = XAxiDma_IntrGetIrq(&ctx->axi_dma, XAXIDMA_DEVICE_TO_DMA);
    XAxiDma_IntrAckIrq(&ctx->axi_dma, irq_status, XAXIDMA_DEVICE_TO_DMA);

    if (!(irq_status & XAXIDMA_IRQ_ALL_MASK)) {
        return; /* rien pour nous */
    }

    if (irq_status & XAXIDMA_IRQ_ERROR_MASK) {
        LIDAR_LD19_LOG("[LD19] Erreur IRQ DMA (0x%08lX), reset...\r\n", (unsigned long)irq_status);
        lidar_ld19_reset_dma(ctx);
        (void)lidar_ld19_arm_rx(ctx);
        return;
    }

    if (irq_status & XAXIDMA_IRQ_IOC_MASK) {
        lidar_ld19_process_packet(ctx);
        (void)lidar_ld19_arm_rx(ctx);
    }
}

/* ==================================================================
 * HELPER : envoi du scan complet par Ethernet, decoupe en morceaux
 * (ETH_MAX_PAYLOAD=512 octets, un scan ~450 points en ferait ~1800 -- ne
 * tient pas en un seul datagramme). Appelee UNIQUEMENT depuis la boucle
 * principale (LIDAR_LD19_Update()), jamais depuis l'ISR : eth_send_frame()
 * s'appuie sur un tampon interne non reentrant (meme contrainte que
 * eth_printf(), cf ETH_driver.h).
 * ================================================================== */
#define LIDAR_LD19_ETH_POINTS_PER_CHUNK 100U

typedef struct __attribute__((packed)) {
    uint8_t  lidar_id;
    uint32_t scan_id;
    uint32_t timestamp_ms;
    uint16_t chunk_index;
    uint16_t chunk_count;
    uint16_t point_count;
    lidar_point_t points[LIDAR_LD19_ETH_POINTS_PER_CHUNK];
} lidar_ld19_eth_chunk_t;

static void lidar_ld19_send_scan_ethernet(lidar_ld19_context_t *ctx, const lidar_scan_t *scan)
{
    lidar_ld19_eth_chunk_t chunk;
    uint16_t chunk_count = (uint16_t)((scan->count + LIDAR_LD19_ETH_POINTS_PER_CHUNK - 1U) / LIDAR_LD19_ETH_POINTS_PER_CHUNK);
    if (chunk_count == 0U) {
        chunk_count = 1U; /* scan vide : on envoie quand meme un chunk 0 point (horodatage) */
    }

    for (uint16_t c = 0; c < chunk_count; c++) {
        uint16_t start = (uint16_t)(c * LIDAR_LD19_ETH_POINTS_PER_CHUNK);
        uint16_t n = (uint16_t)(scan->count - start);
        if (n > LIDAR_LD19_ETH_POINTS_PER_CHUNK) {
            n = LIDAR_LD19_ETH_POINTS_PER_CHUNK;
        }

        chunk.lidar_id     = ctx->lidar_id;
        chunk.scan_id      = scan->scan_id;
        chunk.timestamp_ms = scan->timestamp_ms;
        chunk.chunk_index  = c;
        chunk.chunk_count  = chunk_count;
        chunk.point_count  = n;
        for (uint16_t i = 0; i < n; i++) {
            chunk.points[i] = scan->points[start + i];
        }

        uint16_t payload_len = (uint16_t)(sizeof(chunk) - sizeof(chunk.points) + (size_t)n * sizeof(lidar_point_t));
        (void)eth_send_frame(ETH_MSG_LIDAR_SCAN_CHUNK, &chunk, payload_len);
    }
}

/* Affichage Teleplot (https://teleplot.fr) d'un scan : chaque point envoye
 * est ">lidar:x:y|xy" (xil_printf ne supporte pas %f -> conversion mm/rad
 * faite en float en interne, affichage entier en mm). Decimation dediee
 * (LIDAR_LD19_SCAN_TELEPLOT_DECIMATION) car tout le scan part en une seule
 * rafale. Seul le nuage de points est envoye sur l'UART. Factorisee entre
 * l'envoi automatique (LIDAR_LD19_Update()) et l'appel manuel de test
 * (LIDAR_LD19_PrintScanTeleplot(), cf lidar_ld19.h). */
static void lidar_ld19_teleplot_print_scan(const lidar_scan_t *scan)
{
    for (uint16_t i = 0; i < scan->count; i += LIDAR_LD19_SCAN_TELEPLOT_DECIMATION) {
        uint16_t d = scan->points[i].distance_mm;
        uint16_t a = scan->points[i].angle_cdeg;
        float angle_rad = (float)a * LIDAR_LD19_CDEG_TO_RAD;
        int32_t x_mm = (int32_t)((float)d * cosf(angle_rad));
        int32_t y_mm = (int32_t)((float)d * sinf(angle_rad));
        xil_printf(">lidar:%d:%d|xy\r\n", (int)x_mm, (int)y_mm);
    }
}

/* ==================================================================
 * 3. Mise a jour periodique (appelee par l'IO_Manager, boucle principale)
 *    La reception DMA est desormais pilotee par interruption (cf
 *    LIDAR_LD19_IntrHandler) : cette fonction ne fait plus que consommer
 *    le dernier scan complet des qu'il change (print Teleplot + streaming
 *    Ethernet), c'est-a-dire tout ce qui touche a de l'E/S et ne doit donc
 *    pas se faire depuis l'ISR.
 * ================================================================== */
void LIDAR_LD19_Update(void *instance)
{
    lidar_ld19_context_t *ctx = (lidar_ld19_context_t *)instance;

    const lidar_scan_t *scan = LIDAR_LD19_GetLastScan(ctx);
    if (scan == NULL || scan->scan_id == ctx->last_consumed_scan_id) {
        return; /* rien de nouveau depuis le dernier appel */
    }
    ctx->last_consumed_scan_id = scan->scan_id;

#if LIDAR_LD19_PRINT_POINTS
    lidar_ld19_teleplot_print_scan(scan);
#endif

    if (ctx->eth_streaming_enabled) {
        lidar_ld19_send_scan_ethernet(ctx, scan);
    }
}

/* ==================================================================
 * 4. Desinitialisation (DRVDIS / IO_Manager_SetDeviceStateByName)
 * ================================================================== */
void LIDAR_LD19_Deinit(void *instance)
{
    lidar_ld19_context_t *ctx = (lidar_ld19_context_t *)instance;

    XAxiDma_IntrDisable(&ctx->axi_dma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);
    lidar_ld19_reset_dma(ctx);

    LIDAR_LD19_LOG("[LD19] Deinit (id=%u) : IRQ DMA coupees, DMA reset\r\n", (unsigned)ctx->lidar_id);
}

/* ==================================================================
 * 5. API de consultation du dernier scan complet (cf lidar_ld19.h)
 * ================================================================== */
const lidar_scan_t *LIDAR_LD19_GetLastScan(lidar_ld19_context_t *ctx)
{
    if (ctx->scan_buf[ctx->ready_idx].scan_id == 0U) {
        return NULL; /* aucun tour complet recu depuis le demarrage */
    }
    return &ctx->scan_buf[ctx->ready_idx];
}

/* ==================================================================
 * 6. Configuration a chaud des filtres (cf lidar_ld19_config_t)
 * ================================================================== */
void LIDAR_LD19_SetConfig(lidar_ld19_context_t *ctx, const lidar_ld19_config_t *cfg)
{
    Xil_Out32(ctx->regs_base + LIDAR_LD19_REG_DIST_MIN, cfg->dist_min_mm);
    Xil_Out32(ctx->regs_base + LIDAR_LD19_REG_DIST_MAX, cfg->dist_max_mm);
    Xil_Out32(ctx->regs_base + LIDAR_LD19_REG_ANGLE_MIN, cfg->angle_min_cdeg);
    Xil_Out32(ctx->regs_base + LIDAR_LD19_REG_ANGLE_MAX, cfg->angle_max_cdeg);
    Xil_Out32(ctx->regs_base + LIDAR_LD19_REG_INTENSITY_MIN, cfg->intensity_min);
    Xil_Out32(ctx->regs_base + LIDAR_LD19_REG_CTRL, cfg->filter_enabled ? 0x1U : 0x0U);

    /* cf avertissement sur lidar_ld19_config_t.cluster_mode dans le .h :
     * le decodage cote C ne gere que le nuage complet. On ecrit quand meme
     * le registre (fonctionnement "materiel" demande explicitement par
     * l'appelant), mais on previent en debug si on s'ecarte du mode
     * supporte. */
    Xil_Out32(ctx->regs_base + LIDAR_LD19_REG_CL_CTRL, cfg->cluster_mode ? 0x1U : 0x0U);
#if defined(LIDAR_LD19_DEBUG)
    if (cfg->cluster_mode) {
        LIDAR_LD19_LOG("[LD19] ATTENTION (id=%u) : cluster_mode active, non decode cote C\r\n",
                       (unsigned)ctx->lidar_id);
    }
#endif
}

void LIDAR_LD19_GetConfig(lidar_ld19_context_t *ctx, lidar_ld19_config_t *cfg_out)
{
    cfg_out->dist_min_mm      = (uint16_t)Xil_In32(ctx->regs_base + LIDAR_LD19_REG_DIST_MIN);
    cfg_out->dist_max_mm      = (uint16_t)Xil_In32(ctx->regs_base + LIDAR_LD19_REG_DIST_MAX);
    cfg_out->angle_min_cdeg   = (uint16_t)Xil_In32(ctx->regs_base + LIDAR_LD19_REG_ANGLE_MIN);
    cfg_out->angle_max_cdeg   = (uint16_t)Xil_In32(ctx->regs_base + LIDAR_LD19_REG_ANGLE_MAX);
    cfg_out->intensity_min    = (uint8_t)Xil_In32(ctx->regs_base + LIDAR_LD19_REG_INTENSITY_MIN);
    cfg_out->filter_enabled   = (uint8_t)(Xil_In32(ctx->regs_base + LIDAR_LD19_REG_CTRL) & 0x1U);
    cfg_out->cluster_mode     = (uint8_t)(Xil_In32(ctx->regs_base + LIDAR_LD19_REG_CL_CTRL) & 0x1U);
}

/* ==================================================================
 * 7. Activation/desactivation du streaming Ethernet (cf lidar_ld19.h)
 * ================================================================== */
void LIDAR_LD19_SetEthernetStreaming(lidar_ld19_context_t *ctx, uint8_t enable)
{
    ctx->eth_streaming_enabled = (enable != 0U) ? 1U : 0U;
    LIDAR_LD19_LOG("[LD19] Streaming Ethernet (id=%u) : %s\r\n",
                   (unsigned)ctx->lidar_id, ctx->eth_streaming_enabled ? "ON" : "OFF");
}

/* ==================================================================
 * 8. Print de test/bring-up (cf lidar_ld19.h) -- inconditionnel
 *    (pas de LIDAR_LD19_LOG/LIDAR_LD19_DEBUG ici : appele explicitement
 *    par l'application pour un test manuel, throttle par l'appelant).
 * ================================================================== */
void LIDAR_LD19_PrintScanUart(lidar_ld19_context_t *ctx)
{
    const lidar_scan_t *scan = LIDAR_LD19_GetLastScan(ctx);
    if (scan == NULL) {
        xil_printf("[LIDAR %u] Aucun scan complet recu pour l'instant\r\n", (unsigned)ctx->lidar_id);
        return;
    }
    xil_printf("[LIDAR %u] scan #%lu : %u points, t=%lu ms\r\n",
               (unsigned)ctx->lidar_id, (unsigned long)scan->scan_id,
               (unsigned)scan->count, (unsigned long)scan->timestamp_ms);
}

/* ==================================================================
 * 9. Print de test/bring-up, variante Teleplot (cf lidar_ld19.h) --
 *    reutilise le meme formattage que l'envoi automatique de
 *    LIDAR_LD19_Update() (cf lidar_ld19_teleplot_print_scan), mais peut
 *    etre appelee independamment (ex: sans attendre un nouveau scan_id)
 *    pour un test manuel visuel sur https://teleplot.fr.
 * ================================================================== */
void LIDAR_LD19_PrintScanTeleplot(lidar_ld19_context_t *ctx)
{
    const lidar_scan_t *scan = LIDAR_LD19_GetLastScan(ctx);
    if (scan == NULL) {
        return; /* rien a tracer pour l'instant */
    }
    lidar_ld19_teleplot_print_scan(scan);
}
