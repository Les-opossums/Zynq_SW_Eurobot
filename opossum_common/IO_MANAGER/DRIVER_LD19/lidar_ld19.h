#ifndef DRIVER_LD19_LIDAR_H
#define DRIVER_LD19_LIDAR_H

#include "xil_types.h"
#include "xaxidma.h"

/* ─── Debug ──────────────────────────────────────────────────────────────
 * Decommenter pour les messages de diagnostic du driver (erreurs d'init,
 * de (re)armement DMA...). Le print des points recus (test demande) est
 * gere separement par LIDAR_LD19_PRINT_POINTS ci-dessous, toujours actif.
 */
// #define LIDAR_LD19_DEBUG

#if defined(LIDAR_LD19_DEBUG)
#include "xil_printf.h"
#define LIDAR_LD19_LOG(...) xil_printf(__VA_ARGS__)
#else
#define LIDAR_LD19_LOG(...) do {} while (0)
#endif

/* Mettre a 0 pour couper l'affichage des points (une fois la chaine
 * validee) sans toucher au reste du driver. */
#define LIDAR_LD19_PRINT_POINTS 1
#define LIDAR_LD19_PRINT_PERIOD_MS 300U /* limite le flot de prints (LD19 ~ plusieurs 100aines de paquets/s) */

/* Format DMA "nuage complet" (cf lidar_top.vhd / lidar_filter_regs.vhd,
 * CL_CTRL=0, reglage par defaut au reset) : 1 mot 32 bits par point,
 * [31:16] distance (mm) | [15:0] angle (0,01 deg, 0..35999), 12
 * points/paquet, tlast au dernier point du paquet. */
#define LIDAR_LD19_POINTS_PER_PKT 12

/* --- Registres AXI4-Lite de lidar_filter_regs (offsets en octets) ---
 * cf opossum_hw (VHDL) : lidar_filter_regs.vhd pour le detail complet. */
#define LIDAR_LD19_REG_DIST_MIN      0x00 /* RW distance min (mm) */
#define LIDAR_LD19_REG_DIST_MAX      0x04 /* RW distance max (mm) */
#define LIDAR_LD19_REG_ANGLE_MIN     0x08 /* RW angle min (0,01 deg) */
#define LIDAR_LD19_REG_ANGLE_MAX     0x0C /* RW angle max (0,01 deg) */
#define LIDAR_LD19_REG_INTENSITY_MIN 0x10 /* RW intensite min */
#define LIDAR_LD19_REG_CTRL          0x14 /* RW bit0 = filtre actif */
#define LIDAR_LD19_REG_FRAME_COUNT   0x18 /* RO trames CRC OK (parseur) */
#define LIDAR_LD19_REG_ERROR_COUNT   0x1C /* RO erreurs CRC/VerLen/timeout */
#define LIDAR_LD19_REG_SPEED         0x20 /* RO vitesse rotation (deg/s) */
#define LIDAR_LD19_REG_UART_FERR     0x24 /* RO erreurs de framing UART */
#define LIDAR_LD19_REG_CL_CTRL       0x28 /* RW bit0 : 0=nuage complet, 1=clusters */
#define LIDAR_LD19_REG_CL_BREAK      0x2C /* RW seuil de rupture clustering */
#define LIDAR_LD19_REG_CL_WALL       0x30 /* RW largeur mur (mm) */
#define LIDAR_LD19_REG_CL_PARAMS     0x34 /* RW parametres clustering */
#define LIDAR_LD19_REG_CLUSTER_COUNT 0x38 /* RO clusters emis */

/* Un point du nuage, deja decode depuis le mot DMA brut. */
typedef struct {
    uint16_t distance_mm; /* 0 = point invalide/filtre */
    uint16_t angle_cdeg;  /* angle en centiemes de degre, 0..35999 */
} lidar_point_t;

/* --- Contexte de l'instance (cf IO_config.h / IO_globals.c) ---
 * dma_device_id / regs_base sont a renseigner avant l'appel a
 * LIDAR_LD19_Init() (cf IO_config.h). rx_raw est le tampon DMA brut
 * (aligne cache), last_packet le dernier paquet decode (12 points). */
typedef struct {
    u32     dma_device_id; /* XPAR_AXI_DMA_0_DEVICE_ID */
    UINTPTR regs_base;     /* XPAR_LIDAR_TOP_FOR_DMA_0_BASEADDR (AXI-Lite config/statut) */

    XAxiDma axi_dma;

    uint32_t rx_raw[LIDAR_LD19_POINTS_PER_PKT] __attribute__((aligned(64)));
    lidar_point_t last_packet[LIDAR_LD19_POINTS_PER_PKT];

    uint32_t packet_count;
} lidar_ld19_context_t;

/* --- Prototypes standards pour l'IO_Manager --- */
int  LIDAR_LD19_Init(void *instance);
void LIDAR_LD19_Update(void *instance);

#endif /* DRIVER_LD19_LIDAR_H */
