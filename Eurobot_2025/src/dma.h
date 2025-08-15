#ifndef DMA_H
#define DMA_H

#define DMA_DEV_ID      XPAR_AXI_DMA_0_DEVICE_ID  // ID dans xparameters.h

#define DMA_BUFFER_SIZE  1024  // Taille du buffer DMA

extern u8 DMA_tx_buffer[DMA_BUFFER_SIZE] __attribute__((aligned(32)));  // Buffer pour l'envoi
extern u8 DMA_rx_buffer[DMA_BUFFER_SIZE] __attribute__((aligned(32)));  // Buffer pour la réception

// Prototypes
int dma_init(void);
int dma_send(void *buffer, u32 length);
int dma_receive(void *buffer, u32 length);

#endif