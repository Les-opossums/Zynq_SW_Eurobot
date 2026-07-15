#ifndef IO_MANAGER_H
#define IO_MANAGER_H

#include "xgpiops.h"
#include "xparameters.h"

// Définition des directions pour plus de clarté
typedef enum {
    IO_DIR_INPUT = 0,
    IO_DIR_OUTPUT = 1
} io_direction_t;

// Structure définissant la configuration d'une broche
typedef struct {
    u32 pin_number;           // Le numéro de la broche (ex: 54 pour EMIO 0)
    io_direction_t direction; // Entrée (0) ou Sortie (1)
    int *data_ptr;            // Pointeur vers ta variable métier
} io_pin_config_t;

// Prototypes des fonctions du manager
int IO_Manager_Init(void);
void IO_Manager_Update(void);

#endif /* IO_MANAGER_H */