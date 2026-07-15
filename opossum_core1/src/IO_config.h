#ifndef IO_CONFIG_H
#define IO_CONFIG_H

#include "IO_MANAGER/IO_manager.h"

/* * 1. DECLARATION DES VARIABLES METIER */
extern int leash_state;
extern int AU_state;
extern int team_state;
extern int IO_1_state;
extern int IO_2_state;
extern int IO_3_state;

/*
 * 2. TABLE DE CONFIGURATION DES IO
 * Format : { PIN_NUMBER, DIRECTION, &VARIABLE }
 */
#define IO_CONFIG_TABLE { \
    {54, IO_DIR_INPUT,  &leash_state}, \
    {55, IO_DIR_INPUT,  &AU_state}, \
    {56, IO_DIR_OUTPUT, &team_state}, \
    {57, IO_DIR_INPUT,  &IO_1_state}, \
    {58, IO_DIR_INPUT,  &IO_2_state}, \
    {59, IO_DIR_OUTPUT, &IO_3_state} \
}

// 3. NOMBRE D'IO 
#define IO_CONFIG_COUNT 6

#endif /* IO_CONFIG_H */