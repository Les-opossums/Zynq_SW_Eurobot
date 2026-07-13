#ifndef __STD_COM_H
#define __STD_COM_H


#define STD_COM_SIZE_BUFF 300

void Std_Com_Init(void);

void Send_Std_Out(uint8_t symbol);
int write(int handle, void *buffer, unsigned int len);

uint8_t Is_Std_Out_Empty(void);
uint8_t Std_Out_To_Buff (uint8_t Buff[], uint8_t Max_Len);

void Buff_To_Std_In (uint8_t Buff[], uint8_t Len);
uint8_t Get_Std_In(uint8_t *c);

void Std_Com_Loop(void);

#endif
