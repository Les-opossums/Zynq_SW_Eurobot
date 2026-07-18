#ifndef __SPEED_CONSTRAINER_H_
#define __SPEED_CONSTRAINER_H_

#include "../../../opossum_common/common_type.h"

extern Acceleration Accel_Max;

extern Speed speed_order;
extern Speed speed_order_constrained;

extern float Speed_Order_1, Speed_Order_2, Speed_Order_3, Speed_Order_4;

extern float robot_v_max;
extern float robot_vt_max;

extern float robot_a_max;
extern float robot_at_max;


/**
 * @brief Initializes the speed constrainer with default values.
 * 
 */
void speed_constrainer_init(void);

/**
 * @brief Initializes the acceleration constrainer with default values.
 * 
 */
void acceleration_constrainer_init(void);

/**
 * @brief Constraints the speed order based on the maximum speed limits.
 * 
 */
void constrain_speed_order(void);

/**
 * @brief Constraints the acceleration order based on the maximum acceleration limits.
 * 
 * @param period The time period over which the acceleration is calculated.
 */
void constrain_acceleration_order(float period);

/**
 * @brief Sets the maximum linear speed constraint.
 * 
 * @param v_max The maximum linear speed in m/s.
 */
void set_Constraint_vitesse_xy_max(float v_max);

/**
 * @brief Sets the maximum angular speed constraint.
 * 
 * @param vt_max The maximum angular speed in rad/s.
 */
void set_Constraint_vt_max(float vt_max);

/**
 * @brief Sets the maximum linear acceleration constraint.
 * 
 * @param a_max The maximum linear acceleration in m/s².
 */
void set_Constraint_a_xy_max(float a_max);

/**
 * @brief Sets the maximum angular acceleration constraint.
 * 
 * @param at_max The maximum angular acceleration in rad/s².
 */
void set_Constraint_at_max(float at_max);

#endif // _ASSERV_H_