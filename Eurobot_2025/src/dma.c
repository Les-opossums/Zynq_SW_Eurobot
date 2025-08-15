#include "main.h"

static XAxiDma AxiDma; // Instance globale DMA

u8 DMA_tx_buffer[DMA_BUFFER_SIZE] __attribute__((aligned(32)));  // Buffer pour l'envoi
u8 DMA_rx_buffer[DMA_BUFFER_SIZE] __attribute__((aligned(32)));  // Buffer pour la réception

// Fonction d'initialisation du DMA
int dma_init(void)
{
    XAxiDma_Config *CfgPtr;
    int status;

    // Lookup config
    CfgPtr = XAxiDma_LookupConfig(DMA_DEV_ID);
    if (!CfgPtr) {
        xil_printf("Erreur : Impossible de trouver la config DMA\r\n");
        return XST_FAILURE;
    }

    // Initialisation
    status = XAxiDma_CfgInitialize(&AxiDma, CfgPtr);
    if (status != XST_SUCCESS) {
        xil_printf("Erreur : Initialisation DMA echouee\r\n");
        return XST_FAILURE;
    }

    // Vérifier que le DMA est en mode simple (pas SG)
    if (XAxiDma_HasSg(&AxiDma)) {
        xil_printf("Erreur : DMA compile en mode Scatter-Gather\r\n");
        return XST_FAILURE;
    }

    xil_printf("DMA initialisé avec succès\r\n");
    return XST_SUCCESS;
}

// Fonction pour envoyer des données (Memory → FIFO via MM2S)
int dma_send(void *buffer, u32 length)
{
    int status;

    // Démarre transfert MM2S
    status = XAxiDma_SimpleTransfer(&AxiDma, (UINTPTR)buffer, length, XAXIDMA_DMA_TO_DEVICE);
    if (status != XST_SUCCESS) {
        xil_printf("Erreur : echec transfert MM2S\r\n");
        return XST_FAILURE;
    }

    // Attendre fin transfert
    while (XAxiDma_Busy(&AxiDma, XAXIDMA_DMA_TO_DEVICE));

    return XST_SUCCESS;
}

// Fonction pour recevoir des données (FIFO → Memory via S2MM)
int dma_receive(void *buffer, u32 length)
{
    int status;

    // Démarre transfert S2MM
    status = XAxiDma_SimpleTransfer(&AxiDma, (UINTPTR)buffer, length, XAXIDMA_DEVICE_TO_DMA);
    if (status != XST_SUCCESS) {
        xil_printf("Erreur : echec transfert S2MM\r\n");
        return XST_FAILURE;
    }

    // Attendre fin transfert
    while (XAxiDma_Busy(&AxiDma, XAXIDMA_DEVICE_TO_DMA));

    return XST_SUCCESS;
}
