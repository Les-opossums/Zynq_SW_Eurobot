#ifndef DRIVER_PS_GPIO_H
#define DRIVER_PS_GPIO_H

#include "xgpiops.h"

typedef enum {
    PS_GPIO_DIR_INPUT = 0,
    PS_GPIO_DIR_OUTPUT = 1
} pin_direction_t;

typedef enum {
    PIN_IRQ_NONE = 0,
    PIN_IRQ_EDGE_RISING,
    PIN_IRQ_EDGE_FALLING,
    PIN_IRQ_EDGE_BOTH,
} pin_irq_t;

// --- structure de config d'une broche GPIO PS ---
typedef struct {
    u32 pin_number;                             // Numéro de la broche GPIO PS
    pin_direction_t direction;                  // Direction de la broche (entrée ou sortie)
    pin_irq_t irq_type;                         // Type d'interruption (aucune, front montant, front descendant, les deux)
    volatile int *state_var;                    // Pointeur vers la variable partagéeà mettre à jours/lire
    void (*irq_callback)(void *callback_ref);   // Optionnel : Pointeur vers la fonction de callback pour l'interruption
} gpio_pin_config_t;

// --- Structure de Contexte du Driver ---
typedef struct {
    XGpioPs instance;                           // Instance du périphérique GPIO PS
    gpio_pin_config_t *pin_table;               // Tableau de configurations des broches
    u32 num_pins;                               // Nombre de broches configurées
}ps_gpio_context_t;

// --- Prototypes standards pour l'IO_Manager ---
int PS_GPIO_Init(void *instance);
void PS_GPIO_Update(void *instance);


#endif