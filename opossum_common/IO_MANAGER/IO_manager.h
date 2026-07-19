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
    DEV_TYPE_UART_PS,
    DEV_TYPE_I2C,
    DEV_TYPE_SPI,
    DEV_TYPE_WS2812B,
    DEV_TYPE_IMU_BNO085,
    DEV_TYPE_CAN_MOTORS,
    DEV_TYPE_ETHERNET
} dev_type_t;

// ---structure universelle d'un périphérique ---
typedef struct {
    const char *name;       // Nom du périphérique (pour debug)
    dev_type_t type;       // Type de périphérique
    core_owner_t owner;    // Propriétaire du périphérique
    void *driver_instance; // Pointeur vers l'instance du driver (ex: XGpio pour GPIO)

    // paramètres d'intéruption (optionnels)
    u32 irq_id;            // ID de l'interruption associée (0 si pas d'interruption)
    Xil_InterruptHandler irq_handler; // Pointeur vers la fonction de gestion de l'interruption

    volatile u8 is_active; // Indique si le périphérique est actif (1) ou inactif (0)

    //pointeurs de fonctions (méthode de l'objet)
    int (*init)(void *instance); // Fonction d'initialisation du périphérique
    void(*update)(void *instance); // Fonction de mise à jour (lecture/écriture)
    void (*deinit)(void *instance); // Fonction de désinitialisation du périphérique (optionnelle)
}io_device_t;


int IO_Manager_Init(void);
void IO_Manager_Update(void);

// Impression des temps d'execution par peripherique (min/max/avg/last).
// Toujours declaree/appelable : no-op si TIMING_MEASURE n'est pas active
// (cf opossum_common/TIMER_MANAGER/timing_stats.h).
void IO_Manager_PrintTiming(void);

#endif /* IO_MANAGER_H */