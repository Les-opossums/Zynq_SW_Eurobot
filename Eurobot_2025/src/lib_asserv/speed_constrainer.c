
#include "lib_asserv.h"


Acceleration Accel_Max;
Speed Speed_Max;

Speed speed_order;
Speed speed_order_constrained;

float Accel_Max_Roue;
float Speed_Max_Roue;

float Speed_Order_1, Speed_Order_2, Speed_Order_3;

float tau_x, tau_y, tau_t = 0;

float old_vx1, old_vx2, old_vx3 = 0;
float old_vy1, old_vy2, old_vy3 = 0;
float old_vt1, old_vt2, old_vt3 = 0;

float vx_o, vy_o, vt_o = 0;

float old_vx_o, old_vy_o, old_vt_o = 0;

float v_o, old_v_o = 0;
float tau = 0;
float coef;


void speed_constrainer_init(void)
{
	Accel_Max.ax    = DEFAULT_CONSTRAINT_A_MAX;
	Accel_Max.ay    = DEFAULT_CONSTRAINT_A_MAX;
	Accel_Max.at    = DEFAULT_CONSTRAINT_AT_MAX;
    Accel_Max_Roue  = DEFAULT_CONSTRAINT_A_ROUE;

	Speed_Max.vx   = DEFAULT_AUTHORIZED_V_MAX;
	Speed_Max.vy   = DEFAULT_AUTHORIZED_V_MAX;
	Speed_Max.vt   = DEFAULT_AUTHORIZED_VT_MAX;
    Speed_Max_Roue = DEFAULT_CONSTRAINT_V_ROUE_MAX;
}


void constrain_speed_order(float period) {
    float vx_o = speed_order.vx;
    float vy_o = speed_order.vy;
    float vt_o = speed_order.vt;
    
    float delta_vx_max = Accel_Max.ax * period;
    float delta_vy_max = Accel_Max.ay * period;
	float delta_vt_max = Accel_Max.at * period;
    //    float delta_vr_max = Accel_Max_Roue * period;
     
	// limites absolues sur les vitesses
	vx_o  = limit_float(vx_o, -Speed_Max.vx, Speed_Max.vx);
	vy_o  = limit_float(vy_o, -Speed_Max.vy, Speed_Max.vy);
	vt_o  = limit_float(vt_o,  -Speed_Max.vt, Speed_Max.vt);

	// limites sur la variation par rapport a la fois d'avant
	vx_o = limit_float(vx_o, speed_order_constrained.vx - delta_vx_max, speed_order_constrained.vx + delta_vx_max);
	vy_o = limit_float(vy_o, speed_order_constrained.vy - delta_vy_max, speed_order_constrained.vy + delta_vy_max);
	vt_o = limit_float(vt_o, speed_order_constrained.vt - delta_vt_max, speed_order_constrained.vt + delta_vt_max);
    
    speed_order_constrained.vx = vx_o;
    speed_order_constrained.vy = vy_o;
    speed_order_constrained.vt = vt_o;
    
    // pour l'instant un peu con, ne prend pas de limite par roue, juste des limites globales
    Speed_Order_1  = -(vt_o * robot_wheel_distance) + vy_o;
    Speed_Order_2 = -(vt_o * robot_wheel_distance) - (vy_o * 0.5) - (vx_o * sin(PI/3));   // *0.5 = * cos(PI/3))
    Speed_Order_3 = -(vt_o * robot_wheel_distance) - (vy_o * 0.5) + (vx_o * sin(PI/3));

}

void set_Constraint_vitesse_xy_max(float v_max) {
    if (v_max != 0) {
        if (v_max <= DEFAULT_CONSTRAINT_V_MAX) {
            Speed_Max.vx = v_max;
            Speed_Max.vy = v_max;
        } else {
            Speed_Max.vx = DEFAULT_CONSTRAINT_V_MAX;
            Speed_Max.vy = DEFAULT_CONSTRAINT_V_MAX;
        }
    } else {
        Speed_Max.vx = DEFAULT_CONSTRAINT_V_MAX;
        Speed_Max.vy = DEFAULT_CONSTRAINT_V_MAX;
    }
}

void set_Constraint_vt_max(float vt_max) {
    if (vt_max != 0) {
        if (vt_max <= DEFAULT_CONSTRAINT_VT_MAX) {
            Speed_Max.vt = vt_max;
        } else {
            Speed_Max.vt = DEFAULT_CONSTRAINT_VT_MAX;
        }         
    } else {
        Speed_Max.vt = DEFAULT_CONSTRAINT_VT_MAX;
    }
}

void set_Constraint_acceleration_xy_max(float a_max, float at_max) {
    if (a_max != 0) {
        Accel_Max.ax = a_max;
        Accel_Max.ay = a_max;
    } else {
        Accel_Max.ax = DEFAULT_CONSTRAINT_A_MAX;
        Accel_Max.ay = DEFAULT_CONSTRAINT_A_MAX;
    }
    if (at_max != 0) {
        Accel_Max.at = at_max;
    } else {
        Accel_Max.at = DEFAULT_CONSTRAINT_AT_MAX;
    }
}


