#ifndef INTERPRETEUR_H
#define INTERPRETEUR_H

#define MAX_CMD_LENGTH 100
#define SEUIL_ERR_SLASH_N_REBOOT_COM 30

//pour ne pas avoir de message d'erreur
//#define WORLD_OF_SILENCE

#define PARAM_ERROR_CODE 1
#define PARAM_OUT_OF_RANGE_ERROR_CODE 2
#define IMPOSSIBLE_STATE_ERROR_CODE 3

typedef struct {
    char* Name;
    uint8_t (*Func)(void);
} Command;


void Interp(char c);

char To_UpperCase (char c);

uint8_t Get_Param_Float (float *retour);
uint8_t Get_Param_u32(u32 *retour);
uint8_t Get_Param_x32(u32 *retour);

uint8_t Get_Param_String(char Dest_Str[], uint8_t Max_Len);

uint8_t Print_All_CMD_Cmd (void);


uint8_t Test_Interpreteur(void);

#endif // INTERPRETEUR_H