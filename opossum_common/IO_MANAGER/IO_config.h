#ifndef IO_CONFIG_H
#define IO_CONFIG_H

#include "IO_manager.h"

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
    extern int bno_cs_state;
    extern int bno_rst_state;
    extern int bno_int_state;
    extern int bno_wake_state;

    #define IO_PIN_BNO_CS  63U
    #define IO_PIN_BNO_RST 61U
    #define IO_PIN_BNO_INT 60U

    #define ROW_BNO_RST  {61, IO_DIR_OUTPUT, &bno_rst_state, CORE_CPU1},
    #define ROW_BNO_WAKE {62, IO_DIR_OUTPUT, &bno_wake_state, CORE_CPU1},
    #define ROW_BNO_CS   {63, IO_DIR_OUTPUT, &bno_cs_state, CORE_CPU1},
    #define ROW_BNO_INT  {60, IO_DIR_INPUT,  &bno_int_state, CORE_CPU1},
#else
    #define ROW_BNO_RST  // Vide pour le CPU0
    #define ROW_BNO_WAKE // Vide pour le CPU0
    #define ROW_BNO_CS   // Vide pour le CPU0
    #define ROW_BNO_INT  // Vide pour le CPU0

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
    ROW_BNO_RST \
    ROW_BNO_WAKE \
    ROW_BNO_CS \
    ROW_BNO_INT \
}

#endif /* IO_CONFIG_H */