#ifndef INTERPRETEUR_H
#define INTERPRETEUR_H

#include "xil_types.h"

#define MAX_CMD_LENGTH 100
#define SEUIL_ERR_SLASH_N_REBOOT_COM 30

//#define WORLD_OF_SILENCE

#define PARAM_ERROR_CODE 1
#define PARAM_OUT_OF_RANGE_ERROR_CODE 2
#define IMPOSSIBLE_STATE_ERROR_CODE 3

/*
 * Contexte de parsing : une instance par source de commandes (UART,
 * Ethernet, ...). Evite qu'une commande partielle d'une source ne se
 * melange avec celle d'une autre entre deux tours de boucle principale.
 */
typedef struct {
    char     Current_Cmd[MAX_CMD_LENGTH + 1];
    uint16_t i_Current_Cmd;
    uint16_t i_Lecture_Current_Cmd;
} interp_ctx_t;

void Interp(interp_ctx_t *ctx, char c);

char To_UpperCase(char c);

uint8_t Get_Param_Float(float *retour);
uint8_t Get_Param_u32(u32 *retour);
uint8_t Get_Param_x32(u32 *retour);
uint8_t Get_Param_String(char Dest_Str[], uint8_t Max_Len);

uint8_t Print_All_CMD_Cmd(void);
uint8_t Test_Interpreteur(void);

#endif // INTERPRETEUR_H