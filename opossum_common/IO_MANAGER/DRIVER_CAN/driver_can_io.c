#include "driver_can_io.h"
#include "xstatus.h"
#include "xil_printf.h"
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════
 * Config bas niveau
 * ═══════════════════════════════════════════════════════════════════════ */

static int CAN_IO_Config(can_io_context_t *ctx) {
    int status;

    XCanPs_EnterMode(&ctx->instance, XCANPS_MODE_CONFIG);
    while (XCanPs_GetMode(&ctx->instance) != XCANPS_MODE_CONFIG);

    status = XCanPs_SetBaudRatePrescaler(&ctx->instance, ctx->baud_prescaler);
    if (status != XST_SUCCESS) return XST_FAILURE;

    status = XCanPs_SetBitTiming(&ctx->instance, ctx->btr_sjw, ctx->btr_ts2, ctx->btr_ts1);
    if (status != XST_SUCCESS) return XST_FAILURE;

    return XST_SUCCESS;
}

/*
 * Filtrage exact par ID : mask == id (idiome standard Xilinx, voir
 * xcanps_intr_example.c). Chaque abonne obtient un match parfait sur son
 * propre ID, independamment des autres IDs choisis (contrairement a un
 * masque partiel qui dependrait de la proximite binaire des IDs).
 */
static int CAN_IO_ConfigureFilters(can_io_context_t *ctx) {
    static const u32 filter_masks[CAN_IO_MAX_SUBSCRIBERS] = {
        XCANPS_AFR_UAF1_MASK, XCANPS_AFR_UAF2_MASK,
        XCANPS_AFR_UAF3_MASK, XCANPS_AFR_UAF4_MASK
    };

    if (XCanPs_IsAcceptFilterBusy(&ctx->instance) != XST_SUCCESS) {
        return XST_FAILURE;
    }

    for (u32 i = 0; i < ctx->num_subscribers; i++) {
        u32 idval = (u32)XCanPs_CreateIdValue(ctx->subscriber_table[i].id, 0, 0, 0, 0);

        XCanPs_AcceptFilterDisable(&ctx->instance, filter_masks[i]);
        if (XCanPs_AcceptFilterSet(&ctx->instance, filter_masks[i], idval, idval) != XST_SUCCESS) {
            return XST_FAILURE;
        }
        XCanPs_AcceptFilterEnable(&ctx->instance, filter_masks[i]);
    }

    return XST_SUCCESS;
}

/* ═══════════════════════════════════════════════════════════════════════
 * Handlers d'interruption — generiques, dispatch par table de subscribers
 * ═══════════════════════════════════════════════════════════════════════ */

static void CAN_IO_SendHandler(void *CallBackRef) {
    (void)CallBackRef;
    // Rien de specifique : l'emission est bloquante cote appelant (voir
    // CAN_IO_Send, attente XCanPs_IsTxFifoFull).
}

static void CAN_IO_RecvHandler(void *CallBackRef) {
    can_io_context_t *ctx = (can_io_context_t *)CallBackRef;

    if (XCanPs_Recv(&ctx->instance, ctx->rx_frame) != XST_SUCCESS) {
        return;
    }

    u8 dlc = (u8)((ctx->rx_frame[1] & XCANPS_DLCR_DLC_MASK) >> XCANPS_DLCR_DLC_SHIFT);

    for (u32 i = 0; i < ctx->num_subscribers; i++) {
        if (ctx->rx_frame[0] == ctx->subscriber_id_words[i]) {
            if (ctx->subscriber_table[i].callback != NULL) {
                ctx->subscriber_table[i].callback(ctx->subscriber_table[i].app_ctx, ctx->rx_frame, dlc);
            }
            break; // un seul abonne par ID (filtrage exact)
        }
    }
}

static void CAN_IO_ErrorHandler(void *CallBackRef, u32 ErrorMask) {
    can_io_context_t *ctx = (can_io_context_t *)CallBackRef;

    if (ErrorMask & XCANPS_ESR_ACKER_MASK) ctx->error_stats.ack_error_count++;
    if (ErrorMask & XCANPS_ESR_BERR_MASK)  ctx->error_stats.bit_error_count++;
    if (ErrorMask & XCANPS_ESR_STER_MASK)  ctx->error_stats.stuff_error_count++;
    if (ErrorMask & XCANPS_ESR_FMER_MASK)  ctx->error_stats.form_error_count++;
    if (ErrorMask & XCANPS_ESR_CRCER_MASK) ctx->error_stats.crc_error_count++;

    ctx->error_stats.total_error_count++;
    ctx->error_stats.last_esr_value = ErrorMask;

    XCanPs_ClearBusErrorStatus(&ctx->instance, ErrorMask);
}

static void CAN_IO_EventHandler(void *CallBackRef, u32 IntrMask) {
    can_io_context_t *ctx = (can_io_context_t *)CallBackRef;

    if (IntrMask & XCANPS_IXR_BSOFF_MASK)  ctx->error_stats.bus_off_count++;
    if (IntrMask & XCANPS_IXR_RXOFLW_MASK) ctx->error_stats.rx_fifo_overflow_count++;
    if (IntrMask & XCANPS_IXR_RXUFLW_MASK) ctx->error_stats.rx_fifo_underflow_count++;
    if (IntrMask & XCANPS_IXR_ARBLST_MASK) ctx->error_stats.arbitration_lost_count++;
}

/* ═══════════════════════════════════════════════════════════════════════
 * API standard IO_Manager
 * ═══════════════════════════════════════════════════════════════════════ */

int CAN_IO_Init(void *instance) {
    can_io_context_t *ctx = (can_io_context_t *)instance;

    if (ctx->first_init_done) {
        CAN_IO_Enable(ctx);
        return XST_SUCCESS;
    }

    if (ctx->num_subscribers > CAN_IO_MAX_SUBSCRIBERS) {
        xil_printf("[CAN] Erreur : %lu abonnes demandes, max %d (filtres materiels)\n",
                   (unsigned long)ctx->num_subscribers, CAN_IO_MAX_SUBSCRIBERS);
        return XST_FAILURE;
    }

    XCanPs_Config *Config = XCanPs_LookupConfig(ctx->device_id);
    if (Config == NULL) {
        xil_printf("[CAN] Erreur : peripherique introuvable (device_id=%lu)\n", (unsigned long)ctx->device_id);
        return XST_FAILURE;
    }

    XCanPs_CfgInitialize(&ctx->instance, Config, Config->BaseAddr);
    if (XCanPs_SelfTest(&ctx->instance) != XST_SUCCESS) {
        xil_printf("[CAN] Erreur : self-test echoue\n");
        return XST_FAILURE;
    }

    if (CAN_IO_Config(ctx) != XST_SUCCESS) return XST_FAILURE;
    if (CAN_IO_ConfigureFilters(ctx) != XST_SUCCESS) return XST_FAILURE;

    for (u32 i = 0; i < ctx->num_subscribers; i++) {
        ctx->subscriber_id_words[i] = (u32)XCanPs_CreateIdValue(ctx->subscriber_table[i].id, 0, 0, 0, 0);
    }

    XCanPs_SetHandler(&ctx->instance, XCANPS_HANDLER_SEND,  (void *)CAN_IO_SendHandler,  (void *)ctx);
    XCanPs_SetHandler(&ctx->instance, XCANPS_HANDLER_RECV,  (void *)CAN_IO_RecvHandler,  (void *)ctx);
    XCanPs_SetHandler(&ctx->instance, XCANPS_HANDLER_ERROR, (void *)CAN_IO_ErrorHandler, (void *)ctx);
    XCanPs_SetHandler(&ctx->instance, XCANPS_HANDLER_EVENT, (void *)CAN_IO_EventHandler, (void *)ctx);

    XCanPs_IntrEnable(&ctx->instance, XCANPS_IXR_ALL);

    XCanPs_EnterMode(&ctx->instance, XCANPS_MODE_NORMAL);
    while (XCanPs_GetMode(&ctx->instance) != XCANPS_MODE_NORMAL);

    ctx->bus_enabled = TRUE;
    CAN_IO_ResetErrorStats(ctx);
    ctx->first_init_done = 1;

    xil_printf("[CAN] Initialise (device_id=%lu, %lu abonnes)\n",
               (unsigned long)ctx->device_id, (unsigned long)ctx->num_subscribers);
    return XST_SUCCESS;
}

void CAN_IO_Update(void *instance) {
    (void)instance;
    // RX pilotee par interruption, TX declenchee explicitement par
    // l'appelant via CAN_IO_Send — rien a faire au rythme generique.
}

void CAN_IO_Deinit(void *instance) {
    CAN_IO_Disable((can_io_context_t *)instance);
}

/* ═══════════════════════════════════════════════════════════════════════
 * API publique
 * ═══════════════════════════════════════════════════════════════════════ */

int CAN_IO_Send(can_io_context_t *ctx, u32 id, const u8 *payload, u8 len) {
    if (!ctx->bus_enabled) return XST_FAILURE;
    if (len > CAN_IO_FRAME_DATA_LENGTH) len = CAN_IO_FRAME_DATA_LENGTH;

    ctx->tx_frame[0] = (u32)XCanPs_CreateIdValue(id, 0, 0, 0, 0);
    ctx->tx_frame[1] = (u32)XCanPs_CreateDlcValue((u32)len);

    u8 *frame_bytes = (u8 *)&ctx->tx_frame[2];
    memcpy(frame_bytes, payload, len);

    while (XCanPs_IsTxFifoFull(&ctx->instance) == TRUE); // bloquant, comme l'original

    return (XCanPs_Send(&ctx->instance, ctx->tx_frame) == XST_SUCCESS) ? XST_SUCCESS : XST_FAILURE;
}

void CAN_IO_Disable(can_io_context_t *ctx) {
    if (!ctx->bus_enabled) return;

    XCanPs_IntrDisable(&ctx->instance, XCANPS_IXR_ALL);
    XCanPs_EnterMode(&ctx->instance, XCANPS_MODE_CONFIG);
    while (XCanPs_GetMode(&ctx->instance) != XCANPS_MODE_CONFIG);

    ctx->bus_enabled = FALSE;
    ctx->error_stats.bus_enabled = FALSE;

    xil_printf("[CAN] Desactive (device_id=%lu)\n", (unsigned long)ctx->device_id);
}

void CAN_IO_Enable(can_io_context_t *ctx) {
    if (ctx->bus_enabled) return;

    XCanPs_Reset(&ctx->instance);

    if (CAN_IO_Config(ctx) != XST_SUCCESS) {
        xil_printf("[CAN] Erreur : reconfiguration impossible\n");
        return;
    }
    if (CAN_IO_ConfigureFilters(ctx) != XST_SUCCESS) {
        xil_printf("[CAN] Erreur : reconfiguration des filtres impossible\n");
        return;
    }

    XCanPs_IntrEnable(&ctx->instance, XCANPS_IXR_ALL);
    XCanPs_EnterMode(&ctx->instance, XCANPS_MODE_NORMAL);
    while (XCanPs_GetMode(&ctx->instance) != XCANPS_MODE_NORMAL);

    ctx->bus_enabled = TRUE;
    ctx->error_stats.bus_enabled = TRUE;

    xil_printf("[CAN] Reactive (device_id=%lu)\n", (unsigned long)ctx->device_id);
}

uint8_t CAN_IO_IsEnabled(can_io_context_t *ctx) {
    return ctx->bus_enabled;
}

void CAN_IO_ResetErrorStats(can_io_context_t *ctx) {
    u8 was_enabled = ctx->bus_enabled;
    memset(&ctx->error_stats, 0, sizeof(CAN_ErrorStats));
    ctx->error_stats.bus_enabled = was_enabled;
}

void CAN_IO_PrintErrorStats(can_io_context_t *ctx) {
    xil_printf("--- CAN bus status (device_id=%lu) ---\r\n", (unsigned long)ctx->device_id);
    xil_printf("Enabled          : %d\r\n", ctx->error_stats.bus_enabled);
    xil_printf("ACK errors       : %lu\r\n", (unsigned long)ctx->error_stats.ack_error_count);
    xil_printf("Bit errors       : %lu\r\n", (unsigned long)ctx->error_stats.bit_error_count);
    xil_printf("Stuff errors     : %lu\r\n", (unsigned long)ctx->error_stats.stuff_error_count);
    xil_printf("Form errors      : %lu\r\n", (unsigned long)ctx->error_stats.form_error_count);
    xil_printf("CRC errors       : %lu\r\n", (unsigned long)ctx->error_stats.crc_error_count);
    xil_printf("Bus-off events   : %lu\r\n", (unsigned long)ctx->error_stats.bus_off_count);
    xil_printf("RX overflow      : %lu\r\n", (unsigned long)ctx->error_stats.rx_fifo_overflow_count);
    xil_printf("RX underflow     : %lu\r\n", (unsigned long)ctx->error_stats.rx_fifo_underflow_count);
    xil_printf("Arbitration lost : %lu\r\n", (unsigned long)ctx->error_stats.arbitration_lost_count);
    xil_printf("Total errors     : %lu\r\n", (unsigned long)ctx->error_stats.total_error_count);
}