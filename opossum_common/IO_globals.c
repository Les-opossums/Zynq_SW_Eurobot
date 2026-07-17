#include "IO_config.h"

// --- Table des broches GPIO PS, instanciée à partir de la macro PS_GPIO_PINS ---
static gpio_pin_config_t PsGpio_PinTable[] = PS_GPIO_PINS;

ps_gpio_context_t PsGpio_Ctx = {
    .pin_table = PsGpio_PinTable,
    .num_pins  = sizeof(PsGpio_PinTable) / sizeof(gpio_pin_config_t)
};

// --- Définition des variables d'état déclarées extern dans IO_config.h ---
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