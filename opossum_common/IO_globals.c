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

