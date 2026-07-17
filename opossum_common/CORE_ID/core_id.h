#ifndef CORE_ID_H
#define CORE_ID_H

// THIS_CORE_ID est injecté par les Symbols du projet Vitis :
// -DTHIS_CORE_ID=0 pour app_cpu0, -DTHIS_CORE_ID=1 pour app_cpu1
#ifndef THIS_CORE_ID
    #error "THIS_CORE_ID doit être défini dans les Symbols du projet (0 = CPU0, 1 = CPU1)"
#endif

// --- Pour le PRÉPROCESSEUR (#if / #elif uniquement) ---
// Macros numériques pures, jamais de cast de type ici
#define CORE_ID_CPU0 0
#define CORE_ID_CPU1 1

// --- Pour le CODE C RUNTIME (comparaisons normales, typées) ---
// Nécessite IO_manager.h inclus avant (pour core_owner_t)
#define THIS_CORE ((core_owner_t)THIS_CORE_ID)

#endif /* CORE_ID_H */