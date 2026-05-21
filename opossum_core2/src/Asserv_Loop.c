#include "main.h"
#include "lib_asserv/Lib_Asserv.h"

uint16_t auto_printpos_delay = 100;

uint8_t Debug_Timing = 0;

uint16_t Asserv_Full_Count = 0;

CAN_Message CAN_Motor1;
CAN_Message CAN_Motor2;
CAN_Message CAN_Motor3;
CAN_Message CAN_Motor4;

uint8_t Channel_Motor1 = 0;
uint8_t Channel_Motor2 = 1;
uint8_t Channel_Motor3 = 2;
uint8_t Channel_Motor4 = 3;

int16_t Rotor_RPM1 = 0;
int16_t Rotor_RPM2 = 0;
int16_t Rotor_RPM3 = 0;
int16_t Rotor_RPM4 = 0;

float wheel_speed1 = 0;
float wheel_speed2 = 0;
float wheel_speed3 = 0;
float wheel_speed4 = 0;

uint8_t stop = 0;

Position position_lidar;

int Last_Timer_Asserv = 0;
int Asserv_State = 0;
int Asserv_Odo_Count = 0;

ESC_Command Consigne;
ESC_Command Wanted_Forced_Consigne;
ESC_Command old_Consigne;

Enable_Kalman en_kalman;

int Lidar_inconsistency_count = 0;

int kalman_initialized = 0;

float dx, dy, dt = 0;

int lidar_delay = 0; // délai de la dernière mesure lidar

int tampon;
int tampon3;
int tampon4 = 0;

// Définition des profils de bruit
float R_lidar[3];

float R_camera[3] = {R_CAMERA_MIN_XY * R_CAMERA_MIN_XY,
                     R_CAMERA_MIN_XY * R_CAMERA_MIN_XY, 
                     R_CAMERA_MIN_T * R_CAMERA_MIN_T};


extern volatile uint32_t new_cmd_from_core0;

void Init_Asserv(void) {
    Consigne.command1 = 0;
    Consigne.command2 = 0;
    Consigne.command3 = 0;
    Consigne.command4 = 0;

    Wanted_Forced_Consigne.command1 = 0;
    Wanted_Forced_Consigne.command2 = 0;
    Wanted_Forced_Consigne.command3 = 0;
    Wanted_Forced_Consigne.command4 = 0;

    old_Consigne.command1 = 0;
    old_Consigne.command2 = 0;
    old_Consigne.command3 = 0;
    old_Consigne.command4 = 0;

    R_lidar[0]  = PROCESS_NOISE_LIDAR_X * PROCESS_NOISE_LIDAR_X;
    R_lidar[1]  = PROCESS_NOISE_LIDAR_Y * PROCESS_NOISE_LIDAR_Y;
    R_lidar[2]  = PROCESS_NOISE_LIDAR_THETA * PROCESS_NOISE_LIDAR_THETA;

    en_kalman.enable_lidar_kalman = 1;
    en_kalman.enable_camera_kalman = 0;

    asserv_init();

    Last_Timer_Asserv = Timer_ms1;
}

void Asserv_Loop(void)
{
	if (Asserv_State == 0) {
        //----------------------------------
        // ODO step 1:
        // - calcul de la vitesse du robot 
        //-----------------------------------
        if ((Timer_ms1 - Last_Timer_Asserv) > ODO_EVERY_MS) {
            
                        #ifdef DEBUG_TIMING
                        tampon3 = Timer_us1;
                        #endif
            Last_Timer_Asserv += ODO_EVERY_MS;

            if (new_cmd_from_core0) {
                new_cmd_from_core0 = 0;
                Process_Shared_Memory_Commands();
            }

            odo_speed_step(speed_motor_1, speed_motor_2, speed_motor_3, speed_motor_4);       
            Asserv_State = 2;
                        #ifdef DEBUG_TIMING
                        tampon4 += Timer_us1 - tampon3;
                        #endif
            
        }

    } else if (Asserv_State == 2) {
        // -----------------------------------
        // ODO step 2:
        // - calcul du kalman + history
        // -----------------------------------
                        #ifdef DEBUG_TIMING
                        tampon3 = Timer_us1;
                        #endif
        kalman_predict(&kalman_current_state, &speed_robot_odom, ODO_EVERY_MS*0.001f);
        kalman_fifo_push(&kalman_fifo, &kalman_current_state, &speed_robot_odom);
        Asserv_Odo_Count ++;
                        #ifdef DEBUG_TIMING
                        tampon4 += Timer_us1 - tampon3;
                        #endif

        if (Asserv_Odo_Count >= ASSERV_EVERY){
            Asserv_Odo_Count = 0;
            Asserv_State ++;
        } else {
            Asserv_State = 0;
        }

    } else if (Asserv_State == 3) {
        // -----------------------------------
        // ODO step 3:
        // - calcul de la vitesse du robot
        // -----------------------------------
                        #ifdef DEBUG_TIMING
                        tampon3 = Timer_us1;
                        #endif
        odo_speed_cumulate_step(ASSERV_EVERY);
                        #ifdef DEBUG_TIMING
                        tampon4 += Timer_us1 - tampon3;
                        #endif
        Asserv_State = 4;

    } else if (Asserv_State == 4) {
        // -----------------------------------
        // ODO step 4:
        // - mise a jour de la position du robot dans la mémoire partagée
        // - mse a jour de la vitesse du robot dans la mémoire partagée
        // -----------------------------------
        local_data.kalman_out.x = kalman_current_state.x[0];
        local_data.kalman_out.y = kalman_current_state.x[1];
        local_data.kalman_out.t = kalman_current_state.x[2];

        local_data.speed_robot = speed_robot_asserv;
        local_data.cmd_speed_constrained = speed_order_constrained;
 
        SEND_FIELD(&local_data, kalman_out);
        SEND_FIELD(&local_data, speed_robot);
        SEND_FIELD(&local_data, cmd_speed_constrained);

        Asserv_State = 10;
    
    } else if (Asserv_State == 10) {
        //-----------------------------------
        // motion step
        //-----------------------------------
                        #ifdef DEBUG_TIMING
                        tampon3 = Timer_us1;
                        #endif
        motion_step();
                        #ifdef DEBUG_TIMING
                        tampon4 += Timer_us1 - tampon3;
                        #endif
        Asserv_State = 20;

    } else if (Asserv_State == 20) {
        //-----------------------------------
        // spped constrain
        //-----------------------------------
                        #ifdef DEBUG_TIMING
                        tampon3 = Timer_us1;
                        #endif
        constrain_speed_order();
                        #ifdef DEBUG_TIMING
                        tampon4 += Timer_us1 - tampon3;
                        #endif
        Asserv_State = 21;

    } else if (Asserv_State == 21) {
        //-----------------------------------
        // acceleration constrain
        //-----------------------------------
                        #ifdef DEBUG_TIMING
                        tampon3 = Timer_us1;
                        #endif
        constrain_acceleration_order(ASSERV_EVERY*ODO_EVERY_MS*0.001f);
                        #ifdef DEBUG_TIMING
                        tampon4 += Timer_us1 - tampon3;
                        #endif

        // printf("ROBOTDATA %.4f %.4f %.4f %.4f %.4f %.4f\n",
        //    (double)speed_order_constrained.vx,
        //    (double)speed_order_constrained.vy,
        //    (double)speed_order_constrained.vt,
        //    (double)speed_robot_asserv.vx,
        //    (double)speed_robot_asserv.vy,
        //    (double)speed_robot_asserv.vt);

        Asserv_State = 30;

    } else if (Asserv_State == 30) {
        //-----------------------------------
        // consigne
        //-----------------------------------
                        #ifdef DEBUG_TIMING
                        tampon3 = Timer_us1;
                        #endif
        Asserv_PWM_calculator(&Consigne);
                        #ifdef DEBUG_TIMING
                        tampon4 += Timer_us1 - tampon3;
                        #endif
        Asserv_State = 40;


    } else if (Asserv_State == 40) {
        if ((Wanted_Forced_Consigne.command1 != 0) || 
            (Wanted_Forced_Consigne.command2 != 0) || 
            (Wanted_Forced_Consigne.command3 != 0) ||
            (Wanted_Forced_Consigne.command4 != 0)) {

            Consigne = Wanted_Forced_Consigne;
        } else {
            // Apply_Deadzone_Compensation(&Consigne);
        }
        
        float Abs_Consigne1 = Abs_Ternaire(Consigne.command1);
        float Abs_Consigne2 = Abs_Ternaire(Consigne.command2);
        float Abs_Consigne3 = Abs_Ternaire(Consigne.command3);
        float Abs_Consigne4 = Abs_Ternaire(Consigne.command4);
        
        float Consigne_Max = Max_Quatre(Abs_Consigne1, Abs_Consigne2, Abs_Consigne3, Abs_Consigne4);

        if (Consigne_Max > 10000) {
            float Consigne_rapport = 10000.0/Consigne_Max;
            Consigne.command1 *= Consigne_rapport;
            Consigne.command2 *= Consigne_rapport;
            Consigne.command3 *= Consigne_rapport;
            Consigne.command4 *= Consigne_rapport;
        }

        old_Consigne = Consigne;
       
        Asserv_State = 50;

    } else if (Asserv_State == 50) {
        if(AU_state){
            asserv_off_step();
        }else{
            motor1_current_order = Consigne.command1;
            motor2_current_order = Consigne.command2;
            motor3_current_order = Consigne.command3;
            motor4_current_order = Consigne.command4;
        }
        CAN_transmit_motor(motor1_current_order, motor2_current_order, motor3_current_order, motor4_current_order);
        Asserv_State = 0;
            #ifdef DEBUG_TIMING
            printf("Long asserv time: %d us\n\r", tampon4);
            tampon4 = 0;
            #endif

        
        

    } else {
        Asserv_State = 0;
    }
}


#define PWM_MIN_ACTIF 30 // seuil pour lequel on considère que la consigne est active (en dessous, on la met à 0 pour éviter de rester dans la zone morte du moteur)
#define PWM_DEADZONE 200 // compensation à ajouter à la consigne pour compenser la zone morte du moteur (valeur à ajuster en fonction du moteur et de la batterie, à tester empiriquement)


void Apply_Deadzone_Compensation(ESC_Command* cmd) {
    if (cmd->command1 > PWM_MIN_ACTIF)      cmd->command1 += PWM_DEADZONE;
    else if (cmd->command1 < -PWM_MIN_ACTIF) cmd->command1 -= PWM_DEADZONE;
    else                                     cmd->command1 = 0;

    if (cmd->command2 > PWM_MIN_ACTIF)      cmd->command2 += PWM_DEADZONE;
    else if (cmd->command2 < -PWM_MIN_ACTIF) cmd->command2 -= PWM_DEADZONE;
    else                                     cmd->command2 = 0;

    if (cmd->command3 > PWM_MIN_ACTIF)      cmd->command3 += PWM_DEADZONE;
    else if (cmd->command3 < -PWM_MIN_ACTIF) cmd->command3 -= PWM_DEADZONE;
    else                                     cmd->command3 = 0;

    if (cmd->command4 > PWM_MIN_ACTIF)      cmd->command4 += PWM_DEADZONE;
    else if (cmd->command4 < -PWM_MIN_ACTIF) cmd->command4 -= PWM_DEADZONE;
    else                                     cmd->command4 = 0;
}

void Set_Lidar_Noise_Cmd(Set_lidar_noise kalman_noise_lidar) {
    R_lidar[0]  = kalman_noise_lidar.process_noise_lidar_x * kalman_noise_lidar.process_noise_lidar_x;
    R_lidar[1]  = kalman_noise_lidar.process_noise_lidar_y * kalman_noise_lidar.process_noise_lidar_y;
    R_lidar[2]  = kalman_noise_lidar.process_noise_lidar_t * kalman_noise_lidar.process_noise_lidar_t;
}

void Set_Kalman_Enable_Cmd(Enable_Kalman enable_kalman) {
    en_kalman.enable_lidar_kalman = enable_kalman.enable_lidar_kalman;
    en_kalman.enable_camera_kalman = enable_kalman.enable_camera_kalman;
}


int count_lidar_cycle = 0; // nombre de cycles de la boucle d'asserv avant d'initialiser le kalman avec le lidar, pour laisser le temps au filtre de se stabiliser

void Process_Shared_Memory_Commands(void) {
    if (CHECK_FIELD(&local_data, cmd_position)) { motion_pos(local_data.cmd_position); }
    if (CHECK_FIELD(&local_data, cmd_speed)) { motion_speed(local_data.cmd_speed); }
    if (CHECK_FIELD(&local_data, cmd_abs_speed)) { motion_absolute_speed(local_data.cmd_abs_speed); }
    
    if (CHECK_FIELD(&local_data, asserv_mode)) {
        if (local_data.asserv_mode == 0) { motion_free(); } 
        else if (local_data.asserv_mode == 4) { motion_block(); }
    }

    if (CHECK_FIELD(&local_data, set_pos)) { set_position(local_data.set_pos); }
    if (CHECK_FIELD(&local_data, vmax)) { set_Constraint_vitesse_xy_max(local_data.vmax); }
    if (CHECK_FIELD(&local_data, vtmax)){ set_Constraint_vt_max(local_data.vtmax); }
    if (CHECK_FIELD(&local_data, amax)) { set_Constraint_a_xy_max(local_data.amax); }
    if (CHECK_FIELD(&local_data, cmd_esc)) { Wanted_Forced_Consigne = local_data.cmd_esc; }
    if (CHECK_FIELD(&local_data, enable_kalman)){ Set_Kalman_Enable_Cmd(local_data.enable_kalman); }
    if (CHECK_FIELD(&local_data, odo_spacing)){ odo_set_spacing(local_data.odo_spacing); }
    if (CHECK_FIELD(&local_data, kalman_noise_lidar)){ Set_Lidar_Noise_Cmd(local_data.kalman_noise_lidar); }

    if (!kalman_initialized) {
        // Attendre quelques cycles lidar pour laisser le filtre se stabiliser
        if (CHECK_FIELD(&local_data, set_lidar)) {
            if (count_lidar_cycle < 10) {
                count_lidar_cycle++;
            } else {
                Position init_pos = {
                    local_data.set_lidar.lidar_position_x,
                    local_data.set_lidar.lidar_position_y,
                    local_data.set_lidar.lidar_position_t
                };
                kalman_init_with_lidar(&kalman_fifo, &init_pos);
                kalman_initialized = 1;
            }
        }
        return;
    }

    // --- Collecte de toutes les observations Kalman ---
    int earliest_index = -1;
    int earliest_delay = -1; // le plus grand delay = le plus loin dans le passé

    if (CHECK_FIELD(&local_data, set_lidar)) {
        int idx = kalman_fifo_insert_lidar(&kalman_fifo, &local_data.set_lidar, R_lidar);
        if (idx >= 0 && local_data.set_lidar.delay > earliest_delay) {
            earliest_delay = local_data.set_lidar.delay;
            earliest_index = idx;
        }
    }

    Set_camera* cameras[3] = {&local_data.set_camera_1,
                               &local_data.set_camera_2,
                               &local_data.set_camera_3};
    uint8_t cam_fields[3] = {
        CHECK_FIELD(&local_data, set_camera_1),
        CHECK_FIELD(&local_data, set_camera_2),
        CHECK_FIELD(&local_data, set_camera_3)
    };

    for (int i = 0; i < 3; i++) {
        if (cam_fields[i]) {
            int idx = kalman_fifo_insert_camera(&kalman_fifo, cameras[i], i); // 0-indexé
            if (idx >= 0 && cameras[i]->delay > earliest_delay) {
                earliest_delay = cameras[i]->delay;
                earliest_index = idx;
            }
        }
    }

    // --- Une seule repropagate depuis le point le plus ancien ---
    if (earliest_index >= 0) {
        kalman_fifo_repropagate(&kalman_fifo, earliest_index,
                                ODO_EVERY_MS * 0.001f, R_lidar);
        kalman_current_state = kalman_fifo.buffer[
            (kalman_fifo.head - 1 + KALMAN_FIFO_LEN) % KALMAN_FIFO_LEN];
    }
}
