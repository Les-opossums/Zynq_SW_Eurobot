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
        if (pin->state_var == NULL) continue;

        u32 pin_bank, pin_bit;
        Get_Bank_And_Bit(pin->pin_number, &pin_bank, &pin_bit);

        // si l'interuption vient de cette banque ET de cette broche
        if (bank == pin_bank && (status & (1 << pin_bit))) {
            // 1- mettre à jour la variable partagée avec l'état actuel de la broche
            if(pin->state_var != NULL) {
                *(pin->state_var) = XGpioPs_ReadPin(&ctx->instance, pin->pin_number);
            }

            // 2- Appeler le callback spécifique si défini
            if (pin->irq_callback != NULL) {
                pin->irq_callback(callback_ref);
            }
        }
    }
}

// ==================================================================
// 2. Initialisation du driver GPIO PS
// ==================================================================
int PS_GPIO_Init(void *instance) {
    ps_gpio_context_t *ctx = (ps_gpio_context_t *)instance;
    XGpioPs_Config *Config;
    int Status;

    // 1 - Initialisation du périphérique GPIO PS
    Config = XGpioPs_LookupConfig(XPAR_XGPIOPS_0_DEVICE_ID);
    Status = XGpioPs_CfgInitialize(&ctx->instance, Config, Config->BaseAddr);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    // 2 - Configurer le callback générique pour les interruptions
    XGpioPs_SetCallback(&ctx->instance, (void *)ctx, PS_GPIO_Callback);

    // 3 - Configurer chaque broche selon la table de configuration
    for (u32 i = 0; i < ctx->num_pins; i++) {
        const gpio_pin_config_t *pin = &ctx->pin_table[i];

        //direction  & output enable
        XGpioPs_SetDirection(&ctx->instance, pin->pin_number, pin->direction);
        if(pin->direction == PS_GPIO_DIR_OUTPUT) {
            XGpioPs_SetOutputEnable(&ctx->instance, pin->pin_number, 1);
        }

        //interruption
        if (pin->direction == PS_GPIO_DIR_INPUT && pin->irq_type != PIN_IRQ_NONE) {
            u8 xil_irq_type;
            switch (pin->irq_type) {
                case PIN_IRQ_EDGE_RISING:
                    xil_irq_type = XGPIOPS_IRQ_TYPE_EDGE_RISING;
                    break;
                case PIN_IRQ_EDGE_FALLING:
                    xil_irq_type = XGPIOPS_IRQ_TYPE_EDGE_FALLING;
                    break;
                case PIN_IRQ_EDGE_BOTH:
                    xil_irq_type = XGPIOPS_IRQ_TYPE_EDGE_BOTH;
                    break;
                default:
                    xil_irq_type = XGPIOPS_IRQ_TYPE_EDGE_BOTH;
                    break;
            }
            XGpioPs_SetIntrTypePin(&ctx->instance, pin->pin_number, xil_irq_type);
            XGpioPs_IntrEnablePin(&ctx->instance, pin->pin_number);
        }
    }

    return XST_SUCCESS;
}


// ==================================================================
// 3. Mise à jour / polling generique
// ==================================================================
void PS_GPIO_Update(void *instance) {
    ps_gpio_context_t *ctx = (ps_gpio_context_t *)instance;

    // On parcourt la table de configuration des broches pour mettre à jour les variables partagées
    for (u32 i = 0; i < ctx->num_pins; i++) {
        const gpio_pin_config_t *pin = &ctx->pin_table[i];

        // si pas de variable partagée associée, on passe à la broche suivante
        if (pin->state_var == NULL) continue;

        if (pin->direction == PS_GPIO_DIR_INPUT) {
            // Lire uniquement quand pas d'interruption, sinon c'est déjà fait dans le callback
            if (pin->irq_type == PIN_IRQ_NONE) {
                *(pin->state_var) = XGpioPs_ReadPin(&ctx->instance, pin->pin_number);
            }
        } else {
        	XGpioPs_WritePin(&ctx->instance, pin->pin_number, *(pin->state_var));
        }
    }
}


