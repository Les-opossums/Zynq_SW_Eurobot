#ifndef IPC_STRUCTURE_H
#define IPC_STRUCTURE_H

#include <stdint.h>
#include "common_type.h"

/**
 * @brief This strcucture describes the organization of the shared memory
 */
typedef struct {
    // ===========================================================================
    // 1 - VARIABLES TRANSPARENTES (Accès direct, pas de flag de validité)
    // ===========================================================================
    volatile uint32_t core0_ready; // 1 if core0 is ready, 0 otherwise
    volatile uint32_t core1_ready; // 1 if core1 is ready, 0 otherwise

    // etats des gpio gérés par CPU0 lus par CPU1
    volatile uint32_t AU_state;        // Etat de l'AU (0 = relâché, 1 = appuyé)
    volatile uint32_t leash_state;     // Etat de la laisse (0 = relâchée, 1 = attachée)

    // commandes GPIO envoyées par CPU1, exécutées par CPU0
    volatile uint32_t bno_cs_state;
    volatile uint32_t bno_rst_state;
    volatile uint32_t bno_int_state;
    volatile uint32_t bno_wake_state;    

    // ===========================================================================
    // 2 - BOITE AUX LETTRE (Messages avec Valid/Ack et Interrupt)
    // ===========================================================================

    // ********************************* CORE0 -> CORE1 *********************************
    volatile uint32_t flag_cmd_position_valid; // CORE0 -> CORE1: 1 if position is valid, 0 otherwise
    volatile uint32_t flag_cmd_position_ack;   // CORE1 -> CORE0: 1 new position taken into account, 0 otherwise
    Position cmd_position;

    volatile uint32_t flag_cmd_speed_valid; // CORE0 -> CORE1: 1 if speed is valid, 0 otherwise
    volatile uint32_t flag_cmd_speed_ack;   // CORE1 -> CORE0: 1 new speed taken into account, 0 otherwise
    Speed cmd_speed;

    volatile uint32_t flag_cmd_abs_speed_valid; // CORE0 -> CORE1: 1 if absolute speed is valid, 0 otherwise
    volatile uint32_t flag_cmd_abs_speed_ack;   // CORE1 -> CORE0: 1 new absolute speed taken into account, 0 otherwise
    Speed cmd_abs_speed;

    volatile uint32_t flag_set_lidar_valid; // CORE0 -> CORE1: 1 if lidar data is valid, 0 otherwise
    volatile uint32_t flag_set_lidar_ack;   // CORE1 -> CORE0: 1 new lidar data taken into account, 0 otherwise
    Set_lidar set_lidar;

    volatile uint32_t flag_set_camera_1_valid; // CORE0 -> CORE1: 1 if camera data is valid, 0 otherwise
    volatile uint32_t flag_set_camera_1_ack;   // CORE1 -> CORE0: 1 new camera data taken into account, 0 otherwise
    Set_camera set_camera_1; // camera 1
    
    volatile uint32_t flag_set_camera_2_valid; // CORE0 -> CORE1: 1 if camera data is valid, 0 otherwise
    volatile uint32_t flag_set_camera_2_ack;   // CORE1 -> CORE0: 1 new camera data taken into account, 0 otherwise
    Set_camera set_camera_2; // camera 2

    volatile uint32_t flag_set_camera_3_valid; // CORE0 -> CORE1: 1 if camera data is valid, 0 otherwise
    volatile uint32_t flag_set_camera_3_ack;   // CORE1 -> CORE0: 1 new camera data taken into account, 0 otherwise
    Set_camera set_camera_3; // camera 3

    volatile uint32_t flag_asserv_mode_valid; // CORE0 -> CORE1: 1 if asserv mode is valid, 0 otherwise
    volatile uint32_t flag_asserv_mode_ack;   // CORE1 -> CORE0: 1 new asserv mode taken into account, 0 otherwise
    int asserv_mode; // asserv mode (0: free, 1: position, 2: speed, 3: absolute speed, 4: break)

    volatile uint32_t flag_set_pos_valid; // CORE0 -> CORE1: 1 if asserv done is valid, 0 otherwise
    volatile uint32_t flag_set_pos_ack;   // CORE1 -> CORE0: 1 new asserv done taken into account, 0 otherwise
    Position set_pos; // position to set in the world frame

    volatile uint32_t flag_vmax_valid; // CORE0 -> CORE1: 1 if speed to set is valid, 0 otherwise
    volatile uint32_t flag_vmax_ack;   // CORE1 -> CORE0: 1 new speed to set taken into account, 0 otherwise
    float vmax; // maximum speed in the world frame

    volatile uint32_t flag_vtmax_valid; // CORE0 -> CORE1: 1 if angular speed to set is valid, 0 otherwise
    volatile uint32_t flag_vtmax_ack;   // CORE1 -> CORE0: 1 new angular speed to set taken into account, 0 otherwise
    float vtmax; // maximum angular speed in the world frame

    volatile uint32_t flag_amax_valid; // CORE0 -> CORE1: 1 if acceleration to set is valid, 0 otherwise
    volatile uint32_t flag_amax_ack;   // CORE1 -> CORE0: 1 new acceleration to set taken into account, 0 otherwise
    float amax; // maximum acceleration in the world frame

    volatile uint32_t flag_cmd_esc_valid; 
    volatile uint32_t flag_cmd_esc_ack; 
    ESC_Command cmd_esc; // command to send to the ESCs  

    volatile uint32_t flag_enable_kalman_valid; // CORE0 -> CORE1: 1 if kalman is enabled, 0 otherwise
    volatile uint32_t flag_enable_kalman_ack;   // CORE1 -> CORE0: 1 new kalman enable taken into account, 0 otherwise
    Enable_Kalman enable_kalman; // kalman enable command

    volatile uint32_t flag_odo_spacing_valid; // CORE0 -> CORE1: 1 if odo spacing is valid, 0 otherwise
    volatile uint32_t flag_odo_spacing_ack;   // CORE1 -> CORE0: 1 new odo spacing taken into account, 0 otherwise
    float odo_spacing; // spacing between the wheels in meters
    
    
    
    
    // ********************************* CORE1 -> CORE0 *********************************

    volatile uint32_t flag_kalman_out_valid; // CORE1 -> CORE0: 1 if kalman output is valid, 0 otherwise
    volatile uint32_t flag_kalman_out_ack;   // CORE0 -> CORE1: 1 new kalman output taken into account, 0 otherwise
    Position kalman_out;

    volatile uint32_t flag_speed_robot_valid; // CORE1 -> CORE0: 1 if speed robot is valid, 0 otherwise
    volatile uint32_t flag_speed_robot_ack;   // CORE0 -> CORE1: 1 new speed robot taken into account, 0 otherwise  
    Speed speed_robot; // speed of the robot in the robot frame (measured by odometry)

    // Consigne de vitesse contrainte (après limiteurs de vitesse et d'accélération)
    // Envoyée par Core1 à chaque cycle d'asservissement pour permettre à Core0
    // de logger la vraie consigne vue par le PID (utile pour le réglage des gains).
    volatile uint32_t flag_cmd_speed_constrained_valid; // CORE1 -> CORE0
    volatile uint32_t flag_cmd_speed_constrained_ack;   // CORE0 -> CORE1
    Speed cmd_speed_constrained; // consigne vitesse après rampe d'accélération (repère robot)

    // ********************************* Timer variables *********************************
    volatile uint32_t flag_Timer_ms_valid; // CORE1 -> CORE0: 1 if timer is valid, 0 otherwise
    volatile uint32_t flag_Timer_ms_ack;   // CORE0 -> CORE1: 1 new timer taken into account, 0 otherwise
    int Timer_ms; // Timer value in ms

    volatile uint32_t flag_asserv_step_timing_valid; // CORE1 -> CORE0: 1 if timing is valid, 0 otherwise
    volatile uint32_t flag_asserv_step_timing_ack;   // CORE0 -> CORE1: 1 new timing taken into account, 0 otherwise
    Asserv_Step_Timing asserv_step_timing; // timing of the asserv steps 

    volatile uint32_t flag_kalman_noise_lidar_valid; // CORE0 -> CORE1: 1 if kalman noise lidar is valid, 0 otherwise
    volatile uint32_t flag_kalman_noise_lidar_ack;   // CORE1 -> CORE0: 1 new kalman noise lidar taken into account, 0 otherwise
    Set_lidar_noise kalman_noise_lidar; // estimation of the process noise on the lidar measurement for the kalman filter (standard deviation in m for x and y, in rad for theta)

    volatile uint32_t flag_motion_done_valid; // CORE1 -> CORE0: 1 if motion done is valid, 0 otherwise
    volatile uint32_t flag_motion_done_ack;   // CORE0 -> CORE1: 1 new motion done taken into account, 0 otherwise
    uint32_t motion_done; // 0 if the robot is moving, 1 if the robot

} ipc_shared_data_t;

#endif // IPC_STRUCTURE_H