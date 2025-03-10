#ifndef __SPEED_CONSTRAINER_H_
#define __SPEED_CONSTRAINER_H_

extern Acceleration Accel_Max;
extern Speed Speed_Max;
extern Speed Old_Speed_Max;

extern Speed speed_order;
extern Speed speed_order_constrained;


extern float Speed_Order_1, Speed_Order_2, Speed_Order_3;
extern float vx_o, vy_o, vt_o;
extern float tau_x;

void speed_constrainer_init(void);

// contraint la consigne de vitesse selon les caracteristiques du robot
void constrain_speed_order(float period);

void set_Constraint_vitesse_xy_max(float v_max);
void set_Constraint_vt_max(float vt_max);

void set_Constraint_acceleration_xy_max(float a_max, float at_max);

#endif // _ASSERV_H_
