#include "interpreteur.h"
#include "command_list.h"
#include "../TIMER_MANAGER/timer_manager.h"
#include "xil_printf.h"
#include <string.h>

/*
 * Contexte actif le temps de l'execution d'un Command_List[].Func().
 * Valide UNIQUEMENT entre le moment ou Interp() trouve la commande et la
 * fin de l'appel a Func() -- execution toujours synchrone, jamais en ISR,
 * jamais reentrante : aucun risque de melange entre deux contextes.
 * C'est ce qui permet de garder la signature Func(void) inchangee pour
 * toutes les commandes, presentes et futures.
 */
static interp_ctx_t *g_active_ctx = NULL;

static uint16_t Min_Ternaire(uint16_t a, uint16_t b) {
    return (a < b) ? a : b;
}

void Interp(interp_ctx_t *ctx, char c) {
    if (c != '\n' && c != '\r') {
        if (c == 0x08) {
            if (ctx->i_Current_Cmd) {
                ctx->i_Current_Cmd--;
                ctx->Current_Cmd[ctx->i_Current_Cmd] = '\0';
            }
        } else if (ctx->i_Current_Cmd < MAX_CMD_LENGTH) {
            ctx->Current_Cmd[ctx->i_Current_Cmd] = c;
            ctx->i_Current_Cmd++;
        } else if (ctx->i_Current_Cmd < (MAX_CMD_LENGTH + 1)) {
            ctx->i_Current_Cmd++;
        }
    } else {
        ctx->Current_Cmd[ctx->i_Current_Cmd] = '\0';
        if (ctx->i_Current_Cmd) {
            if (ctx->i_Current_Cmd == (MAX_CMD_LENGTH + 1)) {
                #ifndef WORLD_OF_SILENCE
                    xil_printf("ERROR_Cmd_Length_Overshoot\r\n");
                #endif
            } else {
                uint16_t Len = 0;
                uint16_t Ind = 0;
                uint8_t Found = 0;
                while ( (ctx->Current_Cmd[Len] >= 'a' && ctx->Current_Cmd[Len] <= 'z') ||
                        (ctx->Current_Cmd[Len] >= 'A' && ctx->Current_Cmd[Len] <= 'Z') ||
                        (ctx->Current_Cmd[Len] >= '0' && ctx->Current_Cmd[Len] <= '9')    ) {
                    ctx->Current_Cmd[Len] = To_UpperCase(ctx->Current_Cmd[Len]);
                    Len++;
                }
                if (Len) {
                    ctx->Current_Cmd[Len] = '\0';
                    ctx->i_Lecture_Current_Cmd = Min_Ternaire(ctx->i_Current_Cmd, (uint16_t)(Len + 1));
                    while ((!Found) && (Ind < Command_List_Length)) {
                        if (strcmp(&ctx->Current_Cmd[0], Command_List[Ind].Name) == 0) {
                            Found = 1;
                        } else {
                            Ind++;
                        }
                    }
                }
                if (!Found) {
                    #ifndef WORLD_OF_SILENCE
                        xil_printf("Cmd_Not_Found,%s\n", &ctx->Current_Cmd[0]);
                    #endif
                } else {
                    g_active_ctx = ctx;
                    uint8_t val8 = (*Command_List[Ind].Func)();
                    g_active_ctx = NULL;
                    #ifndef WORLD_OF_SILENCE
                        if (val8)
                            xil_printf("Cmd_Error %d\r\n", val8);
                    #endif
                }
            }
        }
        ctx->i_Current_Cmd = 0;
    }
}

char To_UpperCase(char c) {
    if (c >= 'a' && c <= 'z')
        return c - 32;
    else
        return c;
}

uint8_t Get_Param_Float(float *retour) {
    if (g_active_ctx == NULL) return 1;
    interp_ctx_t *ctx = g_active_ctx;
    float valf = 0;
    float div = 0;
    uint8_t Is_Neg = 0;
    uint8_t Result_Is_Error = 1;

    while ((ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] != '\0') && (ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] != '-') &&
            !((ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] >= '0') && (ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] <= '9')))  {
        ctx->i_Lecture_Current_Cmd++;
    }

    if ((ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] == '-')) {
        Is_Neg = 1;
        ctx->i_Lecture_Current_Cmd++;
    }

    while ( ((ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] >= '0') && (ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] <= '9')) || (ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] == '.')) {
        if (ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] != '.') {
            valf *= 10;
            valf += ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] - '0';
            div *= 10;
            Result_Is_Error = 0;
        } else {
            div = 1;
        }
        ctx->i_Lecture_Current_Cmd++;
    }

    if (!Result_Is_Error) {
        if (div == 0)
            div = 1;
        valf = valf / div;
        *retour = Is_Neg ? -valf : valf;
    }
    return Result_Is_Error;
}

uint8_t Get_Param_u32(u32 *retour) {
    if (g_active_ctx == NULL) return 1;
    interp_ctx_t *ctx = g_active_ctx;
    u32 val = 0;
    uint8_t Result_Is_Error = 1;

    while ((ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] != '\0') &&
            !((ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] >= '0') && (ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] <= '9')))  {
        ctx->i_Lecture_Current_Cmd++;
    }

    while ((ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] >= '0') && (ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] <= '9')) {
        val *= 10;
        val += ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] - '0';
        Result_Is_Error = 0;
        ctx->i_Lecture_Current_Cmd++;
    }

    if (!Result_Is_Error) {
        *retour = val;
    }
    return Result_Is_Error;
}

uint8_t Get_Param_x32(u32 *retour) {
    if (g_active_ctx == NULL) return 1;
    interp_ctx_t *ctx = g_active_ctx;
    u32 val = 0;
    uint8_t Result_Is_Error = 1;

    while ((ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] != '\0') &&
            !((ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] >= '0') && (ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] <= '9')) &&
            !((To_UpperCase(ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd]) >= 'A') && (To_UpperCase(ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd]) <= 'F'))      )
    {
        ctx->i_Lecture_Current_Cmd++;
    }

    while ( ((ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] >= '0') && (ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] <= '9')) ||
            ((To_UpperCase(ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd]) >= 'A') && (To_UpperCase(ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd]) <= 'F'))      )
    {
        val *= 16;
        if ((ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] >= '0') && (ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] <= '9'))
            val += ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] - '0';
        else if ((To_UpperCase(ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd]) >= 'A') && (To_UpperCase(ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd]) <= 'F'))
            val += 10 + To_UpperCase(ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd]) - 'A';

        Result_Is_Error = 0;
        ctx->i_Lecture_Current_Cmd++;
    }

    if (!Result_Is_Error) {
        *retour = val;
    }
    return Result_Is_Error;
}

uint8_t Get_Param_String(char Dest_Str[], uint8_t Max_Len)
{
    if (g_active_ctx == NULL) return 0;
    interp_ctx_t *ctx = g_active_ctx;
    uint8_t Len = 0;
    uint16_t i_Start = 0, i_End = 0;

    while ((ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] != '"') && (ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] != '\0')) {
        ctx->i_Lecture_Current_Cmd++;
    }
    if (ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] == '"') {
        ctx->i_Lecture_Current_Cmd++;
        i_Start = ctx->i_Lecture_Current_Cmd;
    }
    while ((ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] != '"') && (ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] != '\0')) {
        ctx->i_Lecture_Current_Cmd++;
    }
    if (ctx->Current_Cmd[ctx->i_Lecture_Current_Cmd] == '"') {
        i_End = ctx->i_Lecture_Current_Cmd;
        ctx->i_Lecture_Current_Cmd++;
    }
    if (i_End > i_Start) {
        if ((i_End - i_Start) < (Max_Len - 1)) {
            uint16_t i = i_Start;
            while (i != i_End) {
                Dest_Str[Len] = ctx->Current_Cmd[i];
                Len++;
                i++;
            }
            Dest_Str[Len] = '\0';
            Len++;
        }
    }
    return Len;
}

uint8_t Print_All_CMD_Cmd(void) {
    uint16_t i;
    for (i = 0; i < Command_List_Length; i++) {
        xil_printf("%s\n\r", Command_List[i].Name);
        uint8_t j;
        for (j = 0; j < 10; j++) {
            Delay_ms(1);
        }
    }
    return 0;
}

uint8_t Test_Interpreteur(void) {
    u32 val32;
    if (Get_Param_u32(&val32)) {
        return PARAM_ERROR_CODE;
    }
    xil_printf("test_interpreteur\n\r");
    xil_printf("val32: %d\n\r", val32);
    return 0;
}