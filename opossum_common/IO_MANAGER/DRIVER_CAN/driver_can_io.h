#ifndef DRIVER_CAN_IO_H
#define DRIVER_CAN_IO_H

#include "xcanps.h"
#include "xil_types.h"

/* Contrainte matérielle du Zynq-7000 : 4 filtres d'acceptance (UAF1-UAF4). */
#define CAN_IO_MAX_SUBSCRIBERS   4
#define CAN_IO_MAX_FRAME_WORDS   (XCANPS_MAX_FRAME_SIZE / sizeof(u32))
#define CAN_IO_FRAME_DATA_LENGTH 8

typedef struct {
    uint32_t ack_error_count;
    uint32_t bit_error_count;
    uint32_t stuff_error_count;
    uint32_t form_error_count;
    uint32_t crc_error_count;
    uint32_t bus_off_count;
    uint32_t rx_fifo_overflow_count;
    uint32_t rx_fifo_underflow_count;
    uint32_t arbitration_lost_count;
    uint32_t total_error_count;
    uint32_t last_esr_value;
    uint8_t  bus_enabled;
} CAN_ErrorStats;

/**
 * @brief Callback appelé en contexte d'interruption sur réception d'une
 * trame correspondant à un ID abonné.
 * @param frame_words  [0]=ID (format registre), [1]=DLC (format registre),
 *                      [2..] = donnees (32-bit words), tel que renvoyé
 *                      par XCanPs_Recv.
 */
typedef void (*can_rx_callback_t)(void *app_ctx, const u32 *frame_words, u8 dlc);

typedef struct {
    u32 id;                    // Identifiant CAN 11-bit a filtrer
    can_rx_callback_t callback;
    void *app_ctx;              // Contexte optionnel transmis au callback
} can_subscriber_t;

/**
 * @brief Contexte d'une instance de controleur CAN PS (CAN0 ou CAN1).
 * Aucune variable globale/statique cote hardware : chaque instance de ce
 * contexte represente un bus physique independant.
 */
typedef struct {
    XCanPs instance;
    u32    device_id;   // XPAR_XCANPS_x_DEVICE_ID
    u32    intr_id;      // XPAR_XCANPS_x_INTR

    u32 baud_prescaler;
    u8  btr_sjw;
    u8  btr_ts2;
    u8  btr_ts1;

    volatile u8 bus_enabled;
    CAN_ErrorStats error_stats;

    u32 tx_frame[CAN_IO_MAX_FRAME_WORDS];
    u32 rx_frame[CAN_IO_MAX_FRAME_WORDS];

    const can_subscriber_t *subscriber_table;
    u32 num_subscribers;   // <= CAN_IO_MAX_SUBSCRIBERS

    u32 subscriber_id_words[CAN_IO_MAX_SUBSCRIBERS]; // precalcule a l'init
    u8  first_init_done;   // interne : distingue 1er init d'une reactivation
} can_io_context_t;

/* --- Prototypes standards pour l'IO_Manager --- */
int  CAN_IO_Init(void *instance);
void CAN_IO_Update(void *instance);
void CAN_IO_Deinit(void *instance);

/* --- API publique --- */
int     CAN_IO_Send(can_io_context_t *ctx, u32 id, const u8 *payload, u8 len);
void    CAN_IO_Disable(can_io_context_t *ctx);
void    CAN_IO_Enable(can_io_context_t *ctx);
uint8_t CAN_IO_IsEnabled(can_io_context_t *ctx);
void    CAN_IO_ResetErrorStats(can_io_context_t *ctx);
void    CAN_IO_PrintErrorStats(can_io_context_t *ctx);

#endif /* DRIVER_CAN_IO_H */