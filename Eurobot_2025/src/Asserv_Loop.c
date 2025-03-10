#include "main.h"
#include "lib_asserv/Lib_Asserv.h"

u32 Last_Timer_print_pos = 0;

uint8_t auto_printpos_en = 1;
uint16_t auto_printpos_delay = 100;

uint8_t Debug_Timing = 0;

uint16_t Asserv_Full_Count = 0;

CAN_Message CAN_Motor1;
CAN_Message CAN_Motor2;
CAN_Message CAN_Motor3;

uint8_t Channel_Motor1 = 0;
uint8_t Channel_Motor2 = 1;
uint8_t Channel_Motor3 = 2;

int16_t Rotor_RPM1 = 0;
int16_t Rotor_RPM2 = 0;
int16_t Rotor_RPM3 = 0;

float wheel_speed1 = 0;
float wheel_speed2 = 0;
float wheel_speed3 = 0;

uint8_t stop = 0;


int Last_Timer_Asserv = 0;
int Asserv_State = 0;
int Asserv_Odo_Count = 0;

int16_t previous_angle_motor_1 = 0;
int16_t previous_angle_motor_2 = 0;
int16_t previous_angle_motor_3 = 0;

int16_t delta_angle_motor_1 = 0;
int16_t delta_angle_motor_2 = 0;
int16_t delta_angle_motor_3 = 0;

ESC_Command Consigne;
ESC_Command Wanted_Forced_Consigne;
ESC_Command old_Consigne;

void Init_Asserv(void) {
    Consigne.command1 = 0;
    Consigne.command2 = 0;
    Consigne.command3 = 0;

    Wanted_Forced_Consigne.command1 = 0;
    Wanted_Forced_Consigne.command2 = 0;
    Wanted_Forced_Consigne.command3 = 0;

    old_Consigne.command1 = 0;
    old_Consigne.command2 = 0;
    old_Consigne.command3 = 0;

    asserv_init();

    xil_printf("Asserv Init done\n");
}

void Asserv_Loop(void)
{
	if (Asserv_State == 0) {
        //-----------------------------------
        // ODO step :
        // - calcul de la position du robot a partir de l'angle moteur 
        // - calcul de la vitesse du robot a partir de la vitesse des moteurs
        //-----------------------------------
        if ((Timer_ms1 - Last_Timer_Asserv) > ODO_EVERY_MS) {
            Last_Timer_Asserv += ODO_EVERY_MS;
            Asserv_Odo_Count ++;

            delta_angle_motor_1 = angle_motor_1 - previous_angle_motor_1;
            delta_angle_motor_2 = angle_motor_2 - previous_angle_motor_2;
            delta_angle_motor_3 = angle_motor_3 - previous_angle_motor_3;

            odo_speed_step(speed_motor_1, speed_motor_2, speed_motor_3);
            odo_position_step(delta_angle_motor_1, delta_angle_motor_2, delta_angle_motor_3);

            previous_angle_motor_1 = angle_motor_1;
            previous_angle_motor_2 = angle_motor_2;
            previous_angle_motor_3 = angle_motor_3;

            if (Asserv_Odo_Count >= ASSERV_EVERY){
                Asserv_Odo_Count = 0;
                Asserv_State ++;
            } else {
                Asserv_State = 0;
            }   
        }
    } else if (Asserv_State == 1) {
        //-----------------------------------
        // motion step
        //-----------------------------------
        motion_step();
        Asserv_State ++;

    } else if (Asserv_State == 2) {
        //-----------------------------------
        // spped constrain
        //-----------------------------------
        constrain_speed_order(ASSERV_EVERY*ODO_EVERY_MS*0.001);
        Asserv_State ++;

    } else if (Asserv_State == 3) {
        //-----------------------------------
        // consigne
        //-----------------------------------
        Asserv_PWM_calculator(&Consigne);
        Asserv_State ++;


    } else if (Asserv_State == 4) {
        if ((Wanted_Forced_Consigne.command1 != 0) || 
            (Wanted_Forced_Consigne.command2 != 0) || 
            (Wanted_Forced_Consigne.command3 != 0)) {

            Consigne = Wanted_Forced_Consigne;
        }
        
        float Abs_Consigne1 = Abs_Ternaire(Consigne.command1);
        float Abs_Consigne2 = Abs_Ternaire(Consigne.command2);
        float Abs_Consigne3 = Abs_Ternaire(Consigne.command3);
        
        float Consigne_Max = Max_Trois(Abs_Consigne1, Abs_Consigne2, Abs_Consigne3);

        if (Consigne_Max > 10000) {
            float Consigne_rapport = 10000.0/Consigne_Max;
            Consigne.command1 *= Consigne_rapport;
            Consigne.command2 *= Consigne_rapport;
            Consigne.command3 *= Consigne_rapport;
        }

        old_Consigne = Consigne;
       
        Asserv_State ++;

    } else if (Asserv_State == 5) {
        motor1_current_order = Consigne.command1;
        motor2_current_order = Consigne.command2;
        motor3_current_order = Consigne.command3;
        Asserv_State ++;

    } else if (Asserv_State == 6) {
        if (auto_printpos_en && ((Timer_ms1 - Last_Timer_print_pos) > auto_printpos_delay)) {
            printf("motor,");
            printf("%d,",Timer_ms1);
            printf("%d,",motion_done);
            // print angle_motor
            printf("%.2f,", (float)(position_robot.x));
            printf("%.2f,", (float)(position_robot.y));
            printf("%.2f,", (float)(position_robot.t));
            printf("%.2f,", (float)(speed_robot.vx));
            printf("%.2f,", (float)(speed_robot.vy)); 
            printf("\n");
            Last_Timer_print_pos += auto_printpos_delay;
        }
        Asserv_State = 0;
        
    } else {
        Asserv_State = 0;
    }
}

uint8_t Activate_Position_Sending_Func (void) {
    uint32_t state;
    if (Get_Param_u32(&state)) return PARAM_ERROR_CODE;
    auto_printpos_en = (state != 0);
    Last_Timer_print_pos = Timer_ms1;
    
    uint32_t Delay;
    if (!Get_Param_u32(&Delay)) {   // s'il y a un 2eme param, on s'en sert comme delai entre les prints (en ms, of course)
        auto_printpos_delay = Delay;
    }
    return 0;
}


uint8_t Set_Odo_Spacing_Cmd(void){
    float valf;
    if (Get_Param_Float(&valf))
        return 1;
    odo_set_spacing(valf);
    return 0;
}

