#include "main.h"
#include "lib_asserv/Lib_Asserv.h"


//MOVE
uint8_t Move_Cmd(void) {
    Position Pos_Obj;
    float valf;
    if (Get_Param_Float(&valf))     return PARAM_ERROR_CODE;
    Pos_Obj.x = valf;
    if (Get_Param_Float(&valf))     return PARAM_ERROR_CODE;
    Pos_Obj.y = valf;
    if (Get_Param_Float(&valf))     return PARAM_ERROR_CODE;
    Pos_Obj.t = valf;
    motion_pos(Pos_Obj);
    return 0;
}

// SPEED
uint8_t SPEED_Cmd(void) {
    Speed Vitesse_Obj;
    float valf;
    if (Get_Param_Float(&valf)) return PARAM_ERROR_CODE;
    Vitesse_Obj.vx = valf;
    if (Get_Param_Float(&valf)) return PARAM_ERROR_CODE;
    Vitesse_Obj.vy = valf;
    if (Get_Param_Float(&valf)) return PARAM_ERROR_CODE;
    Vitesse_Obj.vt = valf;
    motion_speed(Vitesse_Obj);
    return 0;
}

// ASPEED
uint8_t Absolute_SPEED_Cmd(void) {
    Speed Vitesse_Obj;
    float valf;
    if (Get_Param_Float(&valf)) return PARAM_ERROR_CODE;
    Vitesse_Obj.vx = valf;
    if (Get_Param_Float(&valf)) return PARAM_ERROR_CODE;
    Vitesse_Obj.vy = valf;
    if (Get_Param_Float(&valf)) return PARAM_ERROR_CODE;
    Vitesse_Obj.vt = valf;
    motion_absolute_speed(Vitesse_Obj);
    return 0;
}
//

// FREE
uint8_t FREE_Cmd(void) {
    motion_free();
    Wanted_Forced_Consigne.command1 = 0;
    Wanted_Forced_Consigne.command2 = 0;
    Wanted_Forced_Consigne.command3 = 0;
    return 0;
}

// HOLD
uint8_t BLOCK_Cmd(void) {
    motion_block();
    Wanted_Forced_Consigne.command1 = 0;
    Wanted_Forced_Consigne.command2 = 0;
    Wanted_Forced_Consigne.command3 = 0;
    return 0;
}


// DONE

uint8_t Asserv_Done_Cmd(void) {
    printf("%d\n", Get_asserv_done());
    return 0;
}

uint8_t Get_Pos_Cmd(void) {
    Position pos = get_position();
    printf("X:%.4f,", (double) (pos.x));
    printf("Y:%.4f,", (double) (pos.y));
    printf("T:%.4f\n", (double) (pos.t));
    return 0;
}

uint8_t Get_Odo_Cmd(void) {
    Position pos = get_position();
    Speed speed = get_speed();
    printf("X:%.4f,", (double) (pos.x));
    printf("Y:%.4f,", (double) (pos.y));
    printf("T:%.4f,", (double) (pos.t));
    printf("Vx:%.4f,", (double) (speed.vx));
    printf("Vy:%.4f,", (double) (speed.vy));
    printf("Vt:%.4f\n", (double) (speed.vt));
    return 0;
}

uint8_t Get_Speed_Wheel_Cmd(void) {
    printf("RPM1:%d,", (Rotor_RPM1));
    printf("RPM2:%d,", (Rotor_RPM2));
    printf("RPM3:%d\n", (Rotor_RPM3));
    return 0;
}

// SETX

uint8_t SETX_Cmd(void) {
    float valf;
    if (Get_Param_Float(&valf))
        return 1;
    set_position_x(valf);
    return 0;
}

// SETY

uint8_t SETY_Cmd(void) {
    float valf;
    if (Get_Param_Float(&valf))
        return 1;
    set_position_y(valf);
    return 0;
}

// SETT

uint8_t SETT_Cmd(void) {
    float valf;
    if (Get_Param_Float(&valf))
        return 1;
    set_position_t(valf);
    return 0;
}

uint8_t SET0_Cmd(void) {
    Position Pos;
    Pos.x = 0;
    Pos.y = 0;
    Pos.t = 0;
    set_position(Pos);
    return 0;
}


// VMAX

uint8_t VMAX_Cmd(void) {
    float valf;
    if (Get_Param_Float(&valf))
        return 1;
    set_Constraint_vitesse_max(valf);
    return 0;
}

// VTMAX

uint8_t VTMAX_Cmd(void) {
    float valf;
    if (Get_Param_Float(&valf))
        return 1;
    set_Constraint_vt_max(valf);
    return 0;
}

// AMAX

uint8_t AMAX_Cmd(void) {
    float valfa;
    if (Get_Param_Float(&valfa))    return PARAM_ERROR_CODE;  //almax
    set_Constraint_acceleration_max(valfa);
    return 0;
}


uint8_t PWM_Func(void)
{
    float valfa, valfb, valfc;
    Wanted_Forced_Consigne.command1 = 0;
    Wanted_Forced_Consigne.command2 = 0;
    Wanted_Forced_Consigne.command3 = 0;
    
    if (Get_Param_Float(&valfa))    return PARAM_ERROR_CODE;    // front
    if (Get_Param_Float(&valfb))    return PARAM_ERROR_CODE;    // back left
    if (Get_Param_Float(&valfc))    return PARAM_ERROR_CODE;    // back right
    
    
    Wanted_Forced_Consigne.command1 = limit_float(valfa, -10000.0, 10000.0);
    Wanted_Forced_Consigne.command2 = limit_float(valfb, -10000.0, 10000.0);
    Wanted_Forced_Consigne.command3 = limit_float(valfc, -10000.0, 10000.0);
    
    return 0;
}

uint8_t PWM1_Func(void)
{
    float valfa;
    Wanted_Forced_Consigne.command1 = 0;
    if (Get_Param_Float(&valfa))    return PARAM_ERROR_CODE;    // front
    Wanted_Forced_Consigne.command1 = limit_float(valfa, -10000.0, 10000.0);
    return 0;
}

uint8_t PWM2_Func(void)
{
    float valfa;
    Wanted_Forced_Consigne.command2 = 0;
    if (Get_Param_Float(&valfa))    return PARAM_ERROR_CODE;    // front
    Wanted_Forced_Consigne.command2 = limit_float(valfa, -10000.0, 10000.0);
    return 0;
}

uint8_t PWM3_Func(void)
{
    float valfa;
    Wanted_Forced_Consigne.command3 = 0;
    if (Get_Param_Float(&valfa))    return PARAM_ERROR_CODE;    // front
    Wanted_Forced_Consigne.command3 = limit_float(valfa, -10000.0, 10000.0);
    return 0;
}



uint8_t Asserv_Mode_Cmd(void) {
    printf("%d\n", asserv_mode);
    return 0;
}


uint8_t MaP_Asserv_State = 0;
uint16_t MaP_Asserv_Count = 0;
uint16_t MaP_Asserv_Count_Max = 0;
uint32_t MaP_Asserv_Timer = 0;
float Param_Map_Asserv = 0.3;


uint8_t Param_Asserv_Cmd(void) {
    float valf1, valf2;
    if (Get_Param_Float(&valf1))
        return 1;
    if (Get_Param_Float(&valf2))
        return 1;

    int Param = valf1;
    if (Param == 10) {
        printf("V_Lin\n");
        printf("KP %f\n", (double)(pid_speed.coef.kp));
        printf("KI %f\n", (double)(pid_speed.coef.ki));
        printf("KD %f\n", (double)(pid_speed.coef.kd));
    } else if (Param == 11) {
        pid_speed.coef.kp = valf2;
        printf("Set V_Lin KP to %f\n", (double)(pid_speed.coef.kp));
    } else if (Param == 12) {
        pid_speed.coef.ki = valf2;
        printf("Set V_Lin KI to %f\n", (double)(pid_speed.coef.ki));
    } else if (Param == 13) {
        pid_speed.coef.kd = valf2;
        printf("Set V_Lin KD to %f\n", (double)(pid_speed.coef.kd));
    }

    else if(Param == 20){
        printf("PID_Dist_X\n");
        printf("KP %f\n", (double)(pid_dist.coef.kp));
        printf("KI %f\n", (double)(pid_dist.coef.ki));
        printf("KD %f\n", (double)(pid_dist.coef.kd));
    } else if(Param == 21){
        pid_dist.coef.kp = valf2;
        printf("Set PID_Dist_X KP to %f\n", (double)(pid_dist.coef.kp));
    } else if(Param == 22){
        pid_dist.coef.ki = valf2;
        printf("Set PID_Dist_X KI to %f\n", (double)(pid_dist.coef.ki));
    } else if(Param == 23){
        pid_dist.coef.kd = valf2;
        printf("Set PID_Dist_X KD to %f\n", (double)(pid_dist.coef.kd));
    } 

    else if(Param == 40){
        printf("PID_ANGLE\n");
        printf("KP %f\n", (double)(pid_angle.coef.kp));
        printf("KI %f\n", (double)(pid_angle.coef.ki));
        printf("KD %f\n", (double)(pid_angle.coef.kd));
    } else if(Param == 41){
        pid_angle.coef.kp = valf2;
        printf("Set PID_ANGLE KP to %f\n", (double)(pid_angle.coef.kp));
    } else if(Param == 42){
        pid_angle.coef.ki = valf2;
        printf("Set PID_ANGLE KI to %f\n", (double)(pid_angle.coef.ki));
    } else if(Param == 43){
        pid_angle.coef.kd = valf2;
        printf("Set PID_ANGLE KD to %f\n", (double)(pid_angle.coef.kd));
    }

    return 0;
}


uint8_t MaP_Asserv_Cmd(void) {
    float valf;
    if (Get_Param_Float(&valf))
        return 1;
    MaP_Asserv_State = valf;


    if (Get_Param_Float(&valf))
        MaP_Asserv_Count_Max = 0;
    else
        MaP_Asserv_Count_Max = valf;

    MaP_Asserv_Count = 0;
    MaP_Asserv_Timer = Timer_ms1;

    if (MaP_Asserv_State == 10) {
        if (MaP_Asserv_Count_Max) {
            MaP_Asserv_State = 10;
            Speed Vitesse_Obj;
            Vitesse_Obj.vx = Param_Map_Asserv;
            Vitesse_Obj.vy = 0;
            Vitesse_Obj.vt = 0;
            motion_speed(Vitesse_Obj);
        } else {
           MaP_Asserv_State = 0;
           return 2;
        }
    }
    if(MaP_Asserv_State == 11){
        if (MaP_Asserv_Count_Max) {
            MaP_Asserv_State = 10;
            Speed Vitesse_Obj;
            Vitesse_Obj.vx = 0;
            Vitesse_Obj.vy = Param_Map_Asserv;
            Vitesse_Obj.vt = 0;
            motion_speed(Vitesse_Obj);
        } else {
           MaP_Asserv_State = 0;
           return 2;
        }
    }
    if(MaP_Asserv_State == 12){
        if (MaP_Asserv_Count_Max) {
            MaP_Asserv_State = 10;
            Speed Vitesse_Obj;
            Vitesse_Obj.vx = 0;
            Vitesse_Obj.vy = 0;
            Vitesse_Obj.vt = Param_Map_Asserv;
            motion_speed(Vitesse_Obj);
        } else {
           MaP_Asserv_State = 0;
           return 2;
        }
    }

    if (MaP_Asserv_State == 50) {
        if (MaP_Asserv_Count_Max) {
            MaP_Asserv_State = 50;
            Position Pos_Obj;
            Pos_Obj.x = Param_Map_Asserv;
            Pos_Obj.y = 0;
            Pos_Obj.t = 0;
            motion_pos(Pos_Obj);
        } else {
           MaP_Asserv_State = 0;
           return 2;
        }
    }

    if (MaP_Asserv_State == 51) {
        if (MaP_Asserv_Count_Max) {
            MaP_Asserv_State = 50;
            Position Pos_Obj;
            Pos_Obj.x = 0;
            Pos_Obj.y = Param_Map_Asserv;
            Pos_Obj.t = 0;
            motion_pos(Pos_Obj);
        } else {
           MaP_Asserv_State = 0;
           return 2;
        }
    }

    if (MaP_Asserv_State == 52) {
        if (MaP_Asserv_Count_Max) {
            MaP_Asserv_State = 50;
            Position Pos_Obj;
            Pos_Obj.x = 0;
            Pos_Obj.y = 0;
            Pos_Obj.t = Param_Map_Asserv;
            motion_pos(Pos_Obj);
        } else {
           MaP_Asserv_State = 0;
           return 2;
        }
    }

    return 0;
}

void MaP_Asserv_Loop(void)
{
    if (MaP_Asserv_State) {
        if (MaP_Asserv_State == 1) {    // Odometrie
            if ((Timer_ms1 - MaP_Asserv_Timer) > 10) {
                // printf("%d,%u,%u,%u\n", MaP_Asserv_Count, QEI1, QEI2, QEI3);
                MaP_Asserv_Timer += 10;
                MaP_Asserv_Count++;
                if (MaP_Asserv_Count_Max) {
                    if (MaP_Asserv_Count > MaP_Asserv_Count_Max) {
                        MaP_Asserv_State = 0;
                    }
                }
            }
        } else if (MaP_Asserv_State == 2) {
            //POS1CNT = 0;
            //POS2CNT = 0;
            MaP_Asserv_State = 1;
            MaP_Asserv_Count = 0;

        } else if (MaP_Asserv_State == 10)  {
            if ((Timer_ms1 - MaP_Asserv_Timer) > 10) {
                MaP_Asserv_Timer += 10;
                
                printf("%.2f,",  (double)(speed_order_constrained.vx));
                printf("%.2f,",  (double)(speed_order_constrained.vy));
                printf("%.2f,",  (double)(speed_order_constrained.vt));

                printf("%.2f,", (double)(speed_robot.vx));
                printf("%.2f,", (double)(speed_robot.vy));
                printf("%.2f",  (double)(speed_robot.vt));

                printf("\n");
                MaP_Asserv_Count++;
                if (MaP_Asserv_Count_Max) {
                    if (MaP_Asserv_Count > MaP_Asserv_Count_Max) {
                        MaP_Asserv_State = 0;
                        MaP_Asserv_Count = 0;
                        motion_free();
                    }
                }
            }
        } else if (MaP_Asserv_State == 50)  {
            if ((Timer_ms1 - MaP_Asserv_Timer) > 10) {
                MaP_Asserv_Timer += 10;
                
                printf("%.2f,",  (double)(speed_order_constrained.vx));
                printf("%.2f,",  (double)(speed_order.vx));
                printf("%.2f,", (double)(speed_robot.vx));

                printf("\n");
                MaP_Asserv_Count++;
                if (MaP_Asserv_Count_Max) {
                    if (MaP_Asserv_Count > MaP_Asserv_Count_Max) {
                        MaP_Asserv_State = 0;
                        MaP_Asserv_Count = 0;
                        motion_free();
                    }
                }
            }
        }
    }
    
}
