#ifndef ASSERV_COMMANDS_H
#define ASSERV_COMMANDS_H

#include "xil_types.h"

/* --- Commandes interpreteur (UART / Ethernet) --- */
uint8_t Move_Cmd(void);
uint8_t Move_Seq_Cmd(void);
uint8_t Speed_Cmd(void);
uint8_t Absolute_Speed_Cmd(void);
uint8_t FREE_Cmd(void);
uint8_t BLOCK_Cmd(void);
uint8_t Start_Wheel_FF_Calibration_Cmd(void);
uint8_t Asserv_Done_Cmd(void);
uint8_t Get_Pos_Cmd(void);
uint8_t Get_Odo_Cmd(void);
uint8_t SET_Cmd(void);
uint8_t SET0_Cmd(void);
uint8_t Set_Lidar_Cmd(void);
uint8_t Set_Lidar_Noise_Cmd(void);
uint8_t Set_Camera_1_Cmd(void);
uint8_t Set_Camera_2_Cmd(void);
uint8_t Set_Camera_3_Cmd(void);
uint8_t VMAX_Cmd(void);
uint8_t VTMAX_Cmd(void);
uint8_t AMAX_Cmd(void);
uint8_t PWM_Func(void);
uint8_t Enable_Kalman_Cmd(void);
uint8_t Set_Odo_Spacing_Cmd(void);
uint8_t Activate_Position_Sending_Func(void);
uint8_t Speed_Timed_Cmd(void);

/* --- Boucles a appeler depuis la boucle principale de CORE0 --- */
void Print_Position_loop(void);
void Speed_Timed_Loop(void);
void Move_Seq_Loop(void);

#endif /* ASSERV_COMMANDS_H */