#include "main.h"

static XAxiDma AxiDma;                  // Instance DMA
static u32 RxBuffer[RX_BUFFER_SIZE/4] __attribute__ ((aligned(64))); // Aligné pour le cache

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

    xil_printf("DMA initialisé avec succès\r\n");
    return XST_SUCCESS;
}

// ---------------------
// Lecture d'un bloc de données Lidar
// ---------------------
int lidar_read_block(u32 nb_bytes) {
    int Status;

    if (nb_bytes > RX_BUFFER_SIZE) {
        xil_printf("Erreur: Taille demande > taille buffer\r\n");
        return XST_FAILURE;
    }

    // Nettoyer cache avant réception
    Xil_DCacheFlushRange((UINTPTR)RxBuffer, nb_bytes);

    // Lancer réception S2MM
    Status = XAxiDma_SimpleTransfer(&AxiDma, (UINTPTR)RxBuffer, nb_bytes, XAXIDMA_DEVICE_TO_DMA);
    if (Status != XST_SUCCESS) {
        xil_printf("Erreur: SimpleTransfer DMA échoué\r\n");
        return XST_FAILURE;
    }

    // Attendre fin transfert
    while (XAxiDma_Busy(&AxiDma, XAXIDMA_DEVICE_TO_DMA));

    // Invalider cache après réception
    Xil_DCacheInvalidateRange((UINTPTR)RxBuffer, nb_bytes);

    return XST_SUCCESS;
}

// ---------------------
// Exemple de parsing points
// Format attendu: [31:16] = angle*100 (centi-deg), [15:4] = distance mm, [3:0] = intensité
// ---------------------
void parse_lidar_data(u32 nb_words) {
    for (u32 i = 0; i < nb_words; i++) {
        u32 raw = RxBuffer[i];
        float angle_deg = ((raw >> 16) & 0xFFFF) / 100.0f;
        u16 distance_mm = (raw >> 4) & 0x0FFF;
        u8 intensity = raw & 0x0F;

        xil_printf("Point %lu: Angle=%.2f°, Dist=%u mm, Intens=%u\r\n",
                   i, angle_deg, distance_mm, intensity);
    }
}

