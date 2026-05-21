#ifndef __ASSERV_MASTER_H
#define __ASSERV_MASTER_H

// #define DEBUG_TIMING 1

// calcul d'un step d'odometrie toutes les 2 ms
#define ODO_EVERY_MS 1
// asserv tous les 5 steps d'odo
#define ASSERV_EVERY 10

#define PWM_BLOCK_LIMIT 90      // % de PWM qui declenche la limite
#define BLOCK_MAX_TIME  2000    // temps en ms au bout duquel il se verouille
extern uint16_t Asserv_Full_Count;  // variable a remettre a 0 pour recuperer le droit de bouger en cas de pb..

extern float Consigne1, Consigne2, Consigne3, Consigne4;
extern int16_t Rotor_RPM1;
extern int16_t Rotor_RPM2;
extern int16_t Rotor_RPM3;
extern int16_t Rotor_RPM4;

extern int kalman_initialized;

extern float R_lidar[3];
extern float R_camera[3];


void Init_Asserv(void);

void Asserv_Loop(void);

void Set_Lidar_Cmd(Set_lidar set_lidar);
void Set_Camera_Cmd(Set_camera set_camera, uint8_t camera_id);
void Set_Lidar_Noise_Cmd(Set_lidar_noise kalman_noise_lidar);
void Set_Kalman_Enable_Cmd(Enable_Kalman enable_kalman);


void Apply_Deadzone_Compensation(ESC_Command* cmd);

int Process_Shared_Memory_Commands(void);

int kalman_fifo_insert_lidar(KalmanFIFO* fifo, Set_lidar* set_lidar, float R_lidar[3]);
int kalman_fifo_insert_camera(KalmanFIFO* fifo, Set_camera* set_camera, uint8_t cam_id);

#endif // __ASSERV_MASTER_H