#ifndef IO_MANAGER_H
#define IO_MANAGER_H

#include "xgpiops.h"
#include "xparameters.h"

// Définition des directions pour plus de clarté
typedef enum {
    IO_DIR_INPUT = 0,
    IO_DIR_OUTPUT = 1
} io_direction_t;

#define CORE_CPU0 0
#define CORE_CPU1 1
#define CORE_BOTH 2

typedef int core_id_t;

// Structure définissant la configuration d'une broche
typedef struct {
    u32 pin_number;           // Le numéro de la broche (ex: 54 pour EMIO 0)
    io_direction_t direction; // Entrée (0) ou Sortie (1)
    int *data_ptr;            // Pointeur vers ta variable métier
    core_id_t owner;          // Indique quel coeur est propriétaire de cette IO
} io_pin_config_t;

/**
 * @brief Initialise le gestionnaire d'entrées/sorties
 * 
 * @note Cette fonction doit être appelée une seule fois au démarrage du système.
 * Suivant le coeur sur lequel elle est exécutée, elle initialise le contrôleur GPIO et configure les broches selon la table de configuration.
 * 
 * @return int 
 */
int IO_Manager_Init(void);

/**
 * @brief Met à jour l'état des broches gérées par le gestionnaire d'entrées/sorties 
 * 
 * @note Cette fonction doit être appelée régulièrement dans la boucle principale du programme. Seul les broches appartenant au coeur courant ou partagées sont traitées.
 */
void IO_Manager_Update(void);

#endif /* IO_MANAGER_H */