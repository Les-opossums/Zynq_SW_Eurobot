
#ifndef __CMD_FOR_MOVE_H
#define __CMD_FOR_MOVE_H


uint8_t Move_Cmd(void);

uint8_t SPEED_Cmd(void);
uint8_t Absolute_SPEED_Cmd(void);

uint8_t FREE_Cmd(void);
uint8_t BLOCK_Cmd(void);
uint8_t Asserv_Done_Cmd(void);
uint8_t Get_Pos_Cmd(void);
uint8_t Get_Odo_Cmd(void);
uint8_t Get_Speed_Wheel_Cmd(void);
uint8_t SETX_Cmd(void);
uint8_t SETY_Cmd(void);
uint8_t SETT_Cmd(void);
uint8_t SET0_Cmd(void);

uint8_t VMAX_Cmd(void);
uint8_t VTMAX_Cmd(void);
uint8_t AMAX_Cmd(void);
uint8_t PWM_Func(void);
uint8_t PWM1_Func(void);
uint8_t PWM2_Func(void);
uint8_t PWM3_Func(void);

uint8_t Asserv_Mode_Cmd(void);
uint8_t Param_Asserv_Cmd(void);

uint8_t MaP_Asserv_Cmd(void);

void MaP_Asserv_Loop(void);


#endif