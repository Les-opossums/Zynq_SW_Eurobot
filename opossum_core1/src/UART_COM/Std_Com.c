#include "../main.h"

uint8_t Std_Com_Out_Buff[STD_COM_SIZE_BUFF];
uint16_t i_Std_Com_Out_Buff_TODO = 0;
uint16_t i_Std_Com_Out_Buff_DONE = 0;

uint8_t Std_Com_In_Buff[STD_COM_SIZE_BUFF];
uint16_t i_Std_Com_In_Buff_TODO = 0;
uint16_t i_Std_Com_In_Buff_DONE = 0;

void Std_Com_Init (void)
{
	i_Std_Com_Out_Buff_TODO = 0;
	i_Std_Com_Out_Buff_DONE = 0;
	i_Std_Com_In_Buff_TODO = 0;
	i_Std_Com_In_Buff_DONE = 0;
}

void Send_Std_Out (uint8_t symbol)
{
    uint16_t i = i_Std_Com_Out_Buff_TODO;
    Std_Com_Out_Buff[i] = symbol;
    i++;
    if (i >= STD_COM_SIZE_BUFF) {
        i = 0;
    }
    i_Std_Com_Out_Buff_TODO = i;
}

int write (int handle, void *buffer, unsigned int len)	// printf entree
{
    unsigned int i;
    uint8_t *buff = buffer;
    for (i = 0; i < len; i++) {
        Send_Std_Out(*buff);
        buff++;
    }
    return len;
}


uint8_t Is_Std_Out_Empty(void) {
    return (i_Std_Com_Out_Buff_TODO == i_Std_Com_Out_Buff_DONE);
}


uint8_t Std_Out_To_Buff (uint8_t Buff[], uint8_t Max_Len)
{
	uint8_t Len = 0;
	while (	(i_Std_Com_Out_Buff_DONE != i_Std_Com_Out_Buff_TODO) &&
			(Len < Max_Len)		) {
        Buff[Len] = Std_Com_Out_Buff[i_Std_Com_Out_Buff_DONE];
		Len++;
        i_Std_Com_Out_Buff_DONE++;
        if (i_Std_Com_Out_Buff_DONE >= STD_COM_SIZE_BUFF)
            i_Std_Com_Out_Buff_DONE = 0;
	}
    return Len;
}

void Buff_To_Std_In (uint8_t Buff[], uint8_t Len)
{
	uint8_t i;
	for (i = 0; i < Len; i++) {
		Std_Com_In_Buff[i_Std_Com_In_Buff_TODO] = Buff[i];
        i_Std_Com_In_Buff_TODO++;
        if (i_Std_Com_In_Buff_TODO >= STD_COM_SIZE_BUFF)
            i_Std_Com_In_Buff_TODO = 0;
	}
}

uint8_t Get_Std_In (uint8_t *c)
{
    if (i_Std_Com_In_Buff_DONE != i_Std_Com_In_Buff_TODO) { // si il y a qq chose dans le buffer
        *c = Std_Com_In_Buff[i_Std_Com_In_Buff_DONE];
        i_Std_Com_In_Buff_DONE++;
        if (i_Std_Com_In_Buff_DONE >= STD_COM_SIZE_BUFF)
            i_Std_Com_In_Buff_DONE = 0;
        return 1;
    } else {
        return 0;
    }
}

void Std_Com_Loop(void)
{
    uint8_t c;
    if (Place_In_Uart_Cmd() > 10) {
        if(Std_Out_To_Buff(&c,1)) {
            Send_Uart_Cmd(c);
        }
    }
    if (Get_Uart_Cmd(&c)) {
        Buff_To_Std_In(&c, 1);
    }
}

