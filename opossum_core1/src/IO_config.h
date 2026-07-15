#ifndef IO_CONFIG_H
#define IO_CONFIG_H

#include "IO_MANAGER/IO_manager.h"

/* ================================================================= *
 * DEFINITION DU COEUR ACTUEL ET DU MAITRE MATERIEL
 * ================================================================= */

// Quel coeur a le droit de faire le Reset/Init global du périphérique ?
#define GPIO_MASTER_CORE CORE_CPU0


/* ================================================================= *
 * DÉCLARATION DES VARIABLES & DES LIGNES DE CONFIGURATION
 * ================================================================= */

// --- IO PARTAGÉES (Lues ou écrites par les deux coeurs) ---
extern int AU_state;
#define ROW_AU {55, IO_DIR_INPUT, &AU_state, CORE_BOTH},


// --- IO SPÉCIFIQUES AU CPU 0 ---
#if THIS_CORE == CORE_CPU0
    extern int leash_state;
    extern int team_state;
    extern int IO_1_state;
    extern int IO_2_state;
    extern int IO_3_state;

    #define ROW_LEASH {54, IO_DIR_INPUT, &leash_state, CORE_CPU0},
    #define ROW_TEAM  {56, IO_DIR_INPUT, &team_state,  CORE_CPU0},
    #define ROW_IO1   {57, IO_DIR_INPUT, &IO_1_state,  CORE_CPU0},
    #define ROW_IO2   {58, IO_DIR_INPUT, &IO_2_state,  CORE_CPU0},
    #define ROW_IO3   {59, IO_DIR_INPUT, &IO_3_state,  CORE_CPU0},
#else
    #define ROW_LEASH // Vide pour le CPU1
    #define ROW_TEAM  // Vide pour le CPU1
    #define ROW_IO1   // Vide pour le CPU1
    #define ROW_IO2   // Vide pour le CPU1
    #define ROW_IO3   // Vide pour le CPU1
#endif


// --- IO SPÉCIFIQUES AU CPU 1 ---
#if THIS_CORE == CORE_CPU1
    // (Ajoute tes extern et tes définitions ROW_ ici plus tard)
#else
    // (Ajoute les ROW_ vides ici plus tard)
#endif


/* ================================================================= *
 * ASSEMBLAGE DU TABLEAU FINAL
 * ================================================================= */
// Le préprocesseur va automatiquement ignorer les macros qui sont vides
// selon le coeur défini tout en haut.
#define IO_CONFIG_TABLE { \
    ROW_AU \
    ROW_LEASH \
    ROW_TEAM \
    ROW_IO1 \
    ROW_IO2 \
    ROW_IO3 \
}

#endif /* IO_CONFIG_H */