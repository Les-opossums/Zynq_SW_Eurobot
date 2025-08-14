#include "main.h"


XAxiDma AxiDma;                  // Instance DMA
u8 RxBuf[NUM_BUFFERS][FRAME_BYTES] __attribute__ ((aligned(DMA_ALIGN)));


int DMA_Timer = 0;

LD19Register lidar_1_reg = {
    .dist_min = 0,
    .dist_max = 3000,
    .angle_min = 0,
    .angle_max = 360,
    .intensity_min = 5000,
    .ctrl = 0, // filter deactivated by default
    .frame_count = 0,
    .error_count = 0
};

// ---------------------
// Init DMA pour réception Lidar
// ---------------------
int init_dma(void) {
    XAxiDma_Config *CfgPtr;
    int Status;

    // Récupérer la config DMA
    CfgPtr = XAxiDma_LookupConfig(DMA_DEV_ID);
    if (!CfgPtr) {
        xil_printf("Erreur: Impossible de trouver la config DMA\r\n");
        return XST_FAILURE;
    }

    // Initialiser DMA
    Status = XAxiDma_CfgInitialize(&AxiDma, CfgPtr);
    if (Status != XST_SUCCESS) {
        xil_printf("Erreur: Init DMA échouée\r\n");
        return XST_FAILURE;
    }

    // Vérifier que le DMA est bien en mode Scatter-Gather désactivé
    if (XAxiDma_HasSg(&AxiDma)) {
        xil_printf("Erreur: DMA configuré en mode SG, attendu mode simple\r\n");
        return XST_FAILURE;
    }

    // Reset du DMA
    XAxiDma_Reset(&AxiDma);
    while (!XAxiDma_ResetIsDone(&AxiDma));

    xil_printf("DMA initialisé avec succès\r\n");
    return XST_SUCCESS;
}

void init_lidar( LD19Register *reg ) {
    // Initialiser les registres Lidar avec les valeurs par défaut
    xil_printf("Init lidar reg...\r\n");
    *((volatile uint32_t*)(LIDAR_REG_BASE + REG_DIST_MIN)) = reg->dist_min;
    *((volatile uint32_t*)(LIDAR_REG_BASE + REG_DIST_MAX)) = reg->dist_max;
    *((volatile uint32_t*)(LIDAR_REG_BASE + REG_ANGLE_MIN)) = reg->angle_min;
    *((volatile uint32_t*)(LIDAR_REG_BASE + REG_ANGLE_MAX)) = reg->angle_max;
    *((volatile uint32_t*)(LIDAR_REG_BASE + REG_INTENSITY_MIN)) = reg->intensity_min;
    *((volatile uint32_t*)(LIDAR_REG_BASE + REG_CTRL)) = reg->ctrl;
    xil_printf("Init lidar reg done\r\n");
}

int dma_recv_frame_blocking(u8 *dst, u32 len_bytes)
{
    int status;

    // Invalider le cache de la zone destination avant que le DMA écrive
    Xil_DCacheInvalidateRange((UINTPTR)dst, len_bytes);

    // Lancer la réception S2MM
    status = XAxiDma_SimpleTransfer(&AxiDma, (UINTPTR)dst, len_bytes, XAXIDMA_DEVICE_TO_DMA);
    if (status != XST_SUCCESS) {
        // xil_printf("S2MM SimpleTransfer failed \r\n");
        return status;
    }

    DMA_Timer = Timer_ms1; // Enregistrer le temps de début
    // Attendre que S2MM termine (poll sur Idle)
    while (XAxiDma_Busy(&AxiDma, XAXIDMA_DEVICE_TO_DMA)) {
        if (Timer_ms1 - DMA_Timer > DMA_TIMEOUT) {
            xil_printf("DMA receive timeout\r\n");
            return XST_FAILURE;
        }
    }

    // À ce stade, dst contient len_bytes de données de la trame
    // Invalider à nouveau par prudence si le cache a été préchargé
    Xil_DCacheInvalidateRange((UINTPTR)dst, len_bytes);

    return XST_SUCCESS;
}


void dump_frame(const u8 *buf, u32 len)
{
    const LidarPoint *p = (const LidarPoint *)buf;
    const int n = len / BYTES_PER_POINT;

    xil_printf("Frame %d points\r\n", n);
    for (int i = 0; i < n; i++) {
        xil_printf("  #%02d  dist=%4u mm  ang8=%3u  I=%3u\r\n",
            i, (unsigned)p[i].dist_mm, (unsigned)p[i].angle_deg, (unsigned)p[i].intensity);
    }
}

void lidar_wr32(u32 off, u32 val) { 
    Xil_Out32(LIDAR_REG_BASE + off, val); 
}
u32  lidar_rd32(u32 off)          { 
    return Xil_In32 (LIDAR_REG_BASE + off); 
}

int lidar_dma_recv(void *dst, u32 len_bytes) {
    int status;

    // Vérif alignement adresse et taille
    if ((((UINTPTR)dst) & 0x3) != 0) {
        xil_printf("DMA ERROR: dst pas aligné sur 32 bits\r\n");
        return XST_FAILURE;
    }
    if ((len_bytes & 0x3) != 0) {
        xil_printf("DMA ERROR: len_bytes pas multiple de 4\r\n");
        return XST_FAILURE;
    }

    // Vérifier si le canal est occupé
    if (XAxiDma_Busy(&AxiDma, XAXIDMA_DEVICE_TO_DMA)) {
        xil_printf("DMA busy, reset canal...\r\n");
        u32 s2mm_status = XAxiDma_ReadReg(XPAR_AXIDMA_0_BASEADDR, 0x34);
        xil_printf("S2MM status before reset: 0x%08X\r\n", s2mm_status);
        XAxiDma_Reset(&AxiDma);
        while (!XAxiDma_ResetIsDone(&AxiDma));
    }

    // Invalider le cache pour la zone avant que le périph écrive
    Xil_DCacheInvalidateRange((UINTPTR)dst, len_bytes);

    // Lancer le transfert
    status = XAxiDma_SimpleTransfer(&AxiDma, (UINTPTR)dst, len_bytes, XAXIDMA_DEVICE_TO_DMA);
    if (status != XST_SUCCESS) {
        xil_printf("DMA ERROR: SimpleTransfer failed (%d)\r\n", status);
        return status;
    }

    // Attendre que ce soit terminé
    while (XAxiDma_Busy(&AxiDma, XAXIDMA_DEVICE_TO_DMA)) {
        // On pourrait aussi mettre un timeout
    }

    // Lire le registre de statut S2MM pour détecter une erreur
    u32 dmasr = XAxiDma_ReadReg(AxiDma.RegBase, 0x34);
    if (dmasr & 0x00007000) { // Bits d'erreurs
        xil_printf("DMA S2MM error: 0x%08X\r\n", dmasr);
        XAxiDma_Reset(&AxiDma);
        while (!XAxiDma_ResetIsDone(&AxiDma));
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}