
#ifndef __CMD_FOR_MOVE_H
#define __CMD_FOR_MOVE_H

extern int auto_printpos_en; // si on active l'envoi de la position
extern uint32_t auto_printpos_delay; // en ms
extern uint32_t Last_Timer_print_pos; // dernier envoi de la position

/**
 * @brief Interpreter fonction for the Move command.
 * 
 * @note This function takes 3 parameters: x, y, t.
 * 
 * @return uint8_t 
 */
uint8_t Move_Cmd(void);

/**
 * @brief Interpreter function for the SPEED command.
 * 
 * @note This function takes 3 parameters: vx, vy, vt.
 * 
 * @return uint8_t 
 */
uint8_t Speed_Cmd(void);

/**
 * @brief Interpreter function for the Absolute SPEED command.
 * 
 * @note This function takes 3 parameters: vx, vy, vt.
 * 
 * @return uint8_t 
 */
uint8_t Absolute_Speed_Cmd(void);

/**
 * @brief Interpreter function for the FREE command.
 * 
 * @return uint8_t 
 */
uint8_t FREE_Cmd(void);

/**
 * @brief Interpreter function for the BLOCK command.
 * 
 * @return uint8_t 
 */
uint8_t BLOCK_Cmd(void);

/**
 * @brief Interpreter function for starting wheel feedforward calibration.
 * 
 * @return uint8_t 
 */
uint8_t Start_Wheel_FF_Calibration_Cmd(void);

/**
 * @brief Interpreter function for the Asserv Done command.
 * 
 * @return uint8_t 
 */
uint8_t Asserv_Done_Cmd(void);

/**
 * @brief Interpreter function for the Get Position command.
 * 
 * @return uint8_t 
 */
uint8_t Get_Pos_Cmd(void);

/**
 * @brief Interpreter function for the Get Odometrie command.
 * 
 * @return uint8_t 
 */
uint8_t Get_Odo_Cmd(void);

/**
 * @brief Interpreter function for the Get Speed Wheel command.
 * 
 * @return uint8_t 
 */
uint8_t Get_Speed_Wheel_Cmd(void);

/**
 * @brief Interpreter function for the Set Position command.
 * 
 * @note This function takes 3 parameters: x, y, t.
 * 
 * @return uint8_t 
 */
uint8_t SET_Cmd(void);

/**
 * @brief Interpreter function for the Set Position 0 command.
 * 
 * @return uint8_t 
 */
uint8_t SET0_Cmd(void);

/**
 * @brief Interpreter function for the Set Lidar command.
 * 
 * @note This function takes 3 parameters: lidar_position_x, lidar_position_y, lidar_position_t, and a delay.
 * 
 * @return uint8_t 
 */
uint8_t Set_Lidar_Cmd(void);


/**
 * @brief Interpreter function for the Set Lidar Noise command.
 * 
 * @note This function takes 3 parameters: process_noise_lidar_x, process_noise_lidar_y, process_noise_lidar_t.
 * 
 * @return uint8_t 
 */
uint8_t Set_Lidar_Noise_Cmd(void);

/**
 * @brief Interpreter function for the Set Camera command.
 * 
 * @note This function takes 3 parameters: camera_position_x, camera_position_y, camera_position_t, and a delay.
 * 
 * @return uint8_t 
 */
uint8_t Set_Camera_1_Cmd(void);
uint8_t Set_Camera_2_Cmd(void);
uint8_t Set_Camera_3_Cmd(void);

/**
 * @brief Interpreter function for the Set Position X command.
 * 
 * @return uint8_t 
 */
uint8_t SETX_Cmd(void);

/**
 * @brief Interpreter function for the Set Position Y command.
 * 
 * @return uint8_t 
 */
uint8_t SETY_Cmd(void);

/**
 * @brief Interpreter function for the Set Position T command.
 * 
 * @return uint8_t 
 */
uint8_t SETT_Cmd(void);

/**
 * @brief Interpreter function for the Set Position 0 command.
 * 
 * @return uint8_t 
 */
uint8_t SET0_Cmd(void);

/**
 * @brief Interpreter function for the Set Position VMAX command.
 * 
 * @return uint8_t 
 */
uint8_t VMAX_Cmd(void);

/**
 * @brief Interpreter function for the Set Position VTMAX command.
 * 
 * @return uint8_t 
 */
uint8_t VTMAX_Cmd(void);

/**
 * @brief Interpreter function for the Set Position AMAX command.
 * 
 * @return uint8_t 
 */
uint8_t AMAX_Cmd(void);

/**
 * @brief Interpreter function for the PWM command.
 * 
 * @note This function takes 4 parameters: command1, command2, command3, command4.
 * 
 * @return uint8_t 
 */
uint8_t PWM_Func(void);

/**
 * @brief Interpreter function for the Enable Kalman command.
 * 
 * @return uint8_t 
 */
uint8_t Enable_Kalman_Cmd(void);

/**
 * @brief Interpreter function for the Set Odo Spacing command.
 * 
 * @return uint8_t 
 */
uint8_t Set_Odo_Spacing_Cmd(void);


/**
 * @brief Interpreter function for the Activate Position Sending command.
 * 
 * @note This function takes 1 parameter: state (0 or 1).
 * 
 * @return uint8_t 
 */
uint8_t Activate_Position_Sending_Func(void);


/**
 * @brief Function to print the current position in a loop.
 * 
 * This function checks if automatic position printing is enabled and prints the current position
 * at regular intervals defined by `auto_printpos_delay`.
 */
void Print_Position_loop(void);

uint8_t Speed_Timed_Cmd(void);
void    Speed_Timed_Loop(void);
#endif
