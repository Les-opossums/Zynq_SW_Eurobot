#ifndef IO_MANAGER_H
#define IO_MANAGER_H

#include "xil_types.h"
#include "../IRQ_MANAGER/IRQ_manager.h"
#include "../CORE_ID/core_id.h"

// --- definition pour l'architecture multi-coeurs ---
typedef enum {
    CORE_CPU0 = 0,
    CORE_CPU1 = 1,
    CORE_BOTH = 2
} core_owner_t;

// --- types de périphériques gérés ---
typedef enum {
    DEV_TYPE_GPIO_PS,
    DEV_TYPE_GPIO_AXI,
    DEV_TYPE_UART_AXI,
    DEV_TYPE_I2C,
    DEV_TYPE_SPI,
    DEV_TYPE_WS2812B,
} dev_type_t;

// ---structure universelle d'un périphérique ---
typedef struct {
    dev_type_t type;       // Type de périphérique
    core_owner_t owner;    // Propriétaire du périphérique
    void *driver_instance; // Pointeur vers l'instance du driver (ex: XGpio pour GPIO)

    // paramètres d'intéruption (optionnels)
    u32 irq_id;            // ID de l'interruption associée (0 si pas d'interruption)
    Xil_InterruptHandler irq_handler; // Pointeur vers la fonction de gestion de l'interruption

    //pointeurs de fonctions (méthode de l'objet)
    int (*init)(void *instance); // Fonction d'initialisation du périphérique
    void(*update)(void *instance); // Fonction de mise à jour (lecture/écriture)
}io_device_t;


int IO_Manager_Init(void);
void IO_Manager_Update(void);


#endif /* IO_MANAGER_H */