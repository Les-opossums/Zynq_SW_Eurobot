#ifndef DRIVER_UART_PS_H
#define DRIVER_UART_PS_H

#include "xuartps.h"
#include "xil_types.h"

#define UART_PS_RING_BUFFER_SIZE 300

// --- Buffer circulaire générique (RX et TX) ---
typedef struct {
    u8            buf[UART_PS_RING_BUFFER_SIZE];
    volatile u16  i_todo;  // index d'écriture
    volatile u16  i_done;  // index de lecture
} uart_ring_buffer_t;

// --- Contexte du driver ---
typedef struct {
    XUartPs instance;
    u32     device_id;
    u32     baudrate;

    uart_ring_buffer_t rx_buf;  // rempli par l'ISR, consommé par l'appli
    uart_ring_buffer_t tx_buf;  // rempli par l'appli, vidé vers le hardware par Update()

    u8 is_console;  // 1 si ce port doit recevoir la sortie de printf/xil_printf via write()
} uart_ps_context_t;

// --- Prototypes standards pour l'IO_Manager ---
int  UART_PS_Init(void *instance);
void UART_PS_Update(void *instance);

// --- API publique pour la couche applicative ---
void UART_PS_SendByte(uart_ps_context_t *ctx, u8 symbol);
void UART_PS_SendBuffer(uart_ps_context_t *ctx, const u8 *data, u16 len);
u8   UART_PS_GetByte(uart_ps_context_t *ctx, u8 *c);       // 1 si un octet a été lu, 0 sinon
u16  UART_PS_TxFreeSpace(uart_ps_context_t *ctx);

#endif /* DRIVER_UART_PS_H */