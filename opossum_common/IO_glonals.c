#include "IO_config.h"

// --- Définition des variables d'état déclarées extern dans IO_config.h ---
volatile int AU_state    = 0;
volatile int leash_state = 0;
volatile int team_state  = 0;
volatile int IO_1_state  = 0;
volatile int IO_2_state  = 0;
volatile int IO_3_state  = 0;
volatile int bno_cs_state   = 0;
volatile int bno_rst_state  = 0;
volatile int bno_int_state  = 0;
volatile int bno_wake_state = 0;

// --- Callback appelé sur interruption de la laisse ---
void leash_Callback(void *callback_ref) {
    (void)callback_ref;
    // rien pour l'instant, juste pour que le link passe
}