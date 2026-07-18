#include "IO_config.h"


// ==================================================================
// Définition du contexte du driver GPIO PS
// ==================================================================

// --- Table des broches GPIO PS, instanciée à partir de la macro PS_GPIO_PINS ---
static gpio_pin_config_t PsGpio_PinTable[] = PS_GPIO_PINS;
ps_gpio_context_t PsGpio_Ctx = {
    .pin_table = PsGpio_PinTable,
    .num_pins  = sizeof(PsGpio_PinTable) / sizeof(gpio_pin_config_t)
};

// --- Variables globales des états des entrées/sorties ---

volatile int AU_state    = 0;
volatile int leash_state = 0;
volatile int team_state  = 0;
volatile int IO_1_state  = 0;
volatile int IO_2_state  = 0;
volatile int IO_3_state  = 0;
volatile int bno_cs_state   = 0;
volatile int bno_rst_state  = 0;
volatile int bno_int_state  = 0;
volatile int bno_wake_state = 0;

// --- Callback appelé sur interruption de la laisse ---
void leash_Callback(void *callback_ref) {
    (void)callback_ref;
    // rien pour l'instant, juste pour que le link passe
}

// ==================================================================
// Définition du contexte du driver BNO085
// ==================================================================
static bno085_report_config_t Bno085_ReportTable[] = BNO085_REPORTS;

bno085_io_context_t Imu_Ctx = {
    .gpio_ctx      = &PsGpio_Ctx,
    .pin_cs        = 63,
    .pin_rst       = 61,
    .pin_int       = 60,
    .report_table  = Bno085_ReportTable,
    .num_reports   = sizeof(Bno085_ReportTable) / sizeof(bno085_report_config_t)
};

// ==================================================================
// Définition du contexte du driver UART PS
// ==================================================================
uart_ps_context_t UartComm_Ctx = {
    .device_id = UART_COMM_DEVICE_ID,
    .baudrate  = UART_COMM_BAUDRATE,
    .is_console = 1
};

// ==================================================================
// Définition du contexte du driver WS2812B
// ==================================================================
led_color_t Led_Buffer[NBR_LED];
ws2812b_context_t Ws2812b_Ctx = {
    .base_addr = WS2812B_BASEADDR,
    .num_leds = NBR_LED,
    .led_buffer = Led_Buffer,
    .refresh_period_ms = 10
};

// ==================================================================
// Définition du contexte du driver CAN0 (bus moteurs ESC C610)
// ==================================================================
static const can_subscriber_t Can0_SubscriberTable[] = CAN0_SUBSCRIBERS;

can_io_context_t Can0_Ctx = {
    .device_id       = CAN0_DEVICE_ID,
    .intr_id         = CAN0_INTR_ID,
    .baud_prescaler  = CAN0_BAUD_PRESCALER,
    .btr_sjw         = CAN0_BTR_SJW,
    .btr_ts2         = CAN0_BTR_TS2,
    .btr_ts1         = CAN0_BTR_TS1,
    .subscriber_table = Can0_SubscriberTable,
    .num_subscribers  = sizeof(Can0_SubscriberTable) / sizeof(can_subscriber_t)
};

eth_io_context_t Eth_Ctx = {
    .config = {
        .mac_addr   = {0x00, 0x0a, 0x35, 0x00, 0x01, 0x12},
        .local_ip   = (192u<<24) | (168u<<16) | (1u<<8) | 10u,
        .netmask    = (255u<<24) | (255u<<16) | (255u<<8) | 0u,
        .gateway_ip = (192u<<24) | (168u<<16) | (1u<<8) | 1u,
        .peer_ip    = (192u<<24) | (168u<<16) | (1u<<8) | 20u
    }
};