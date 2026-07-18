#include "driver_uart_ps.h"
#include "xstatus.h"
#include "xil_printf.h"

// --- Un seul port désigné comme "console" pour la redirection printf ---
static uart_ps_context_t *ConsoleCtx = NULL;

/* ═══════════════════════════════════════════════════════════════════════
 * Helpers buffer circulaire
 * ═══════════════════════════════════════════════════════════════════════ */

static void ring_push(uart_ring_buffer_t *rb, u8 byte) {
    u16 next = (u16)(rb->i_todo + 1);
    if (next >= UART_PS_RING_BUFFER_SIZE) next = 0;
    if (next == rb->i_done) return; // buffer plein : on jette l'octet (protection overflow)
    rb->buf[rb->i_todo] = byte;
    rb->i_todo = next;
}

static u8 ring_pop(uart_ring_buffer_t *rb, u8 *out) {
    if (rb->i_done == rb->i_todo) return 0; // rien à lire
    *out = rb->buf[rb->i_done];
    rb->i_done++;
    if (rb->i_done >= UART_PS_RING_BUFFER_SIZE) rb->i_done = 0;
    return 1;
}

static u16 ring_free_space(uart_ring_buffer_t *rb) {
    u16 used = (u16)(UART_PS_RING_BUFFER_SIZE + rb->i_todo - rb->i_done) % UART_PS_RING_BUFFER_SIZE;
    return (u16)(UART_PS_RING_BUFFER_SIZE - 1 - used);
}

/* ═══════════════════════════════════════════════════════════════════════
 * ISR — appelée par XUartPs_InterruptHandler via IRQ_Manager
 * ═══════════════════════════════════════════════════════════════════════ */

static void UART_PS_Handler(void *CallBackRef, u32 Event, unsigned int EventData) {
    uart_ps_context_t *ctx = (uart_ps_context_t *)CallBackRef;

    if (Event == XUARTPS_EVENT_RECV_DATA || Event == XUARTPS_EVENT_RECV_TOUT) {
        while (XUartPs_IsReceiveData(ctx->instance.Config.BaseAddress)) {
            u8 byte = (u8)XUartPs_ReadReg(ctx->instance.Config.BaseAddress, XUARTPS_FIFO_OFFSET);
            ring_push(&ctx->rx_buf, byte);
        }
    }
    // XUARTPS_EVENT_SENT_DATA / erreurs : rien de spécifique à faire ici pour l'instant
}

/* ═══════════════════════════════════════════════════════════════════════
 * 1. Initialisation
 * ═══════════════════════════════════════════════════════════════════════ */

int UART_PS_Init(void *instance) {
    uart_ps_context_t *ctx = (uart_ps_context_t *)instance;
    XUartPs_Config *Config;
    int Status;

    ctx->rx_buf.i_todo = 0; ctx->rx_buf.i_done = 0;
    ctx->tx_buf.i_todo = 0; ctx->tx_buf.i_done = 0;

    Config = XUartPs_LookupConfig(ctx->device_id);
    if (Config == NULL) {
        return XST_FAILURE;
    }

    Status = XUartPs_CfgInitialize(&ctx->instance, Config, Config->BaseAddress);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    Status = XUartPs_SelfTest(&ctx->instance);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    XUartPs_SetBaudRate(&ctx->instance, ctx->baudrate);
    XUartPs_SetRecvTimeout(&ctx->instance, 8);
    XUartPs_SetHandler(&ctx->instance, (XUartPs_Handler)UART_PS_Handler, ctx);

    XUartPs_SetInterruptMask(&ctx->instance,
        XUARTPS_IXR_TOUT | XUARTPS_IXR_PARITY | XUARTPS_IXR_FRAMING |
        XUARTPS_IXR_OVER | XUARTPS_IXR_TXEMPTY | XUARTPS_IXR_RXFULL |
        XUARTPS_IXR_RXOVR);

    XUartPs_SetOperMode(&ctx->instance, XUARTPS_OPER_MODE_NORMAL);

    if (ctx->is_console) {
        ConsoleCtx = ctx;
    }

    return XST_SUCCESS;
}

/* ═══════════════════════════════════════════════════════════════════════
 * 2. Mise à jour périodique — vide le buffer TX vers le hardware
 * ═══════════════════════════════════════════════════════════════════════ */

void UART_PS_Update(void *instance) {
    uart_ps_context_t *ctx = (uart_ps_context_t *)instance;

    if (ctx->tx_buf.i_done == ctx->tx_buf.i_todo) return; // rien à envoyer

    // Regroupe les octets contigus jusqu'au bout du buffer ou jusqu'à i_todo
    u8 chunk[64];
    u16 n = 0;
    while (n < sizeof(chunk) && ctx->tx_buf.i_done != ctx->tx_buf.i_todo) {
        chunk[n++] = ctx->tx_buf.buf[ctx->tx_buf.i_done];
        ctx->tx_buf.i_done++;
        if (ctx->tx_buf.i_done >= UART_PS_RING_BUFFER_SIZE) ctx->tx_buf.i_done = 0;
    }

    // XUartPs_Send est non-bloquant : il renvoie le nombre d'octets réellement
    // acceptés (peut être < n si un envoi précédent est encore en cours).
    u32 sent = XUartPs_Send(&ctx->instance, chunk, n);

    // Si tout n'a pas été accepté, on remet le surplus en tête de buffer
    // en reculant i_done (les octets non envoyés restent donc à renvoyer
    // au prochain cycle, dans l'ordre).
    if (sent < n) {
        u16 not_sent = (u16)(n - sent);
        ctx->tx_buf.i_done = (u16)((ctx->tx_buf.i_done + UART_PS_RING_BUFFER_SIZE - not_sent) % UART_PS_RING_BUFFER_SIZE);
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * 3. API publique
 * ═══════════════════════════════════════════════════════════════════════ */

void UART_PS_SendByte(uart_ps_context_t *ctx, u8 symbol) {
    ring_push(&ctx->tx_buf, symbol);
}

void UART_PS_SendBuffer(uart_ps_context_t *ctx, const u8 *data, u16 len) {
    for (u16 i = 0; i < len; i++) {
        ring_push(&ctx->tx_buf, data[i]);
    }
}

u8 UART_PS_GetByte(uart_ps_context_t *ctx, u8 *c) {
    return ring_pop(&ctx->rx_buf, c);
}

u16 UART_PS_TxFreeSpace(uart_ps_context_t *ctx) {
    return ring_free_space(&ctx->tx_buf);
}

/* ═══════════════════════════════════════════════════════════════════════
 * 4. Redirection printf/xil_printf vers le port désigné "console"
 * ═══════════════════════════════════════════════════════════════════════ */

int write(int handle, void *buffer, unsigned int len) {
    (void)handle;
    if (ConsoleCtx != NULL) {
        UART_PS_SendBuffer(ConsoleCtx, (const u8 *)buffer, (u16)len);
    }
    return (int)len;
}