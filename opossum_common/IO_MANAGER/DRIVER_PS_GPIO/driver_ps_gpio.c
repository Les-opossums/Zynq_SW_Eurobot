#include "driver_ps_gpio.h"
#include "xparameters.h"

// ================================================================== 
// HELPER : retrouve la banque et le bit de la broche du ZYNQ-7000
// ==================================================================
static void Get_Bank_And_Bit(u32 pin_number, u32 *bank, u32 *bit) {
    if      (pin_number <= 31)  { *bank = 0; *bit = pin_number; }
    else if (pin_number <= 53)  { *bank = 1; *bit = pin_number - 32; }
    else if (pin_number <= 85)  { *bank = 2; *bit = pin_number - 54; } // EMIO commencent ici
    else                        { *bank = 3; *bit = pin_number - 86; }
}


// ================================================================== 
// 1. Callback generique des interruptions 
// ==================================================================
static void PS_GPIO_Callback(void *callback_ref, u32 bank, u32 status) {
    ps_gpio_context_t *ctx = (ps_gpio_context_t *)callback_ref;

    // On parcourt la table de configuration des broches pour trouver celle qui correspond à l'interruption
    for (u32 i = 0; i < ctx->num_pins; i++) {
        const gpio_pin_config_t *pin = &ctx->pin_table[i];

        // si pas de variable partagée associée, on passe à la broche suivante
        if (pin->state_var == NULL) {
            continue;
        }

        if(pin->direction == PS_GPIO_DIR_INPUT) {
            // lecture (Uniquement pour les pins sans interruption)
            if (pin->irq_type == PIN_IRQ_NONE) {
                *(pin->state_var) = XGpioPs_ReadPin(&ctx->instance, pin->pin_number);
            }
        }else{
            XGpioPs_WritePin(&ctx->instance, pin->pin_number, *(pin->state_var));
        }
    }
}