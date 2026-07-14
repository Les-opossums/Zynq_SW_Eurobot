#ifndef __ASSERV_H_
#define __ASSERV_H_


// mode de l'asservissement
#define ASSERV_MODE_OFF 0
#define ASSERV_MODE_FREE 1
#define ASSERV_MODE_BREAK 2
#define ASSERV_MODE_POS 10
#define ASSERV_MODE_SPEED 30
#define ASSERV_MODE_ABSOLUTE_SPEED 31

// États de fin de mouvement (motion_done)
#define MOTION_MOVING   0  // Le robot est en train d'exécuter une consigne
#define MOTION_SUCCESS  1  // Le robot a atteint sa cible avec succès
#define MOTION_BRAKED   2  // Le robot s'est arrêté (Arrêt d'urgence, blocage, freinage manuel)

#define DIST_TOL 0.015 // 5mm
#define ANGLE_TOL 0.0349066 // 1 deg


extern int motion_done;

extern int asserv_mode;

extern float blocked_time;

extern float current_stop_distance;
extern float default_stop_distance;

extern int emergency_break_requested;

/******************************    Fonctions    *******************************/

/**
 * @brief Update the shared motion done status and notify Core 0.
 * 
 * @param new_state The new motion done status.
 */
void update_shared_motion_done(int new_state);

/**
 * @brief Initialize the asservissement system.
 * 
 */
void asserv_init(void);


/**
 * @brief Block the motion of the robot.
 * 
 * This function sets the asservissement mode to break, stopping the robot's movement.
 */
void motion_block(void);

/**
 * @brief Free the motion of the robot.
 * 
 * This function sets the asservissement mode to free, allowing the robot to move without constraints.
 */
void motion_free(void);

/**
 * @brief Motion to a specific position.
 * 
 * This function sets the robot's position to the specified coordinates and activates the position asservissement mode.
 * 
 * @param pos The target position to move to, specified as a Position structure containing x, y, and t (angle).
 */
void motion_pos(Position pos);

/**
 * @brief Disables the asserv of the robot.
 * 
 */
void motion_off(void);

/**
 * @brief Sets the speed of the robot.
 * 
 * This function sets the desired speed of the robot and activates the speed asservissement mode.
 * 
 * @param speed The desired speed to set, specified as a Speed structure containing vx, vy, and vt (angular velocity).
 */
void motion_speed(Speed speed);

/**
 * @brief Sets the absolute speed of the robot.
 * 
 * This function sets the desired absolute speed of the robot and activates the absolute speed asservissement mode.
 * 
 * @param speed The desired absolute speed to set, specified as a Speed structure containing vx, vy, and vt (angular velocity).
 */
void motion_absolute_speed(Speed speed);

/**
 * @brief Step function for the motion control.
 * 
 * This function is called periodically to update the robot's motion based on the current asservissement mode.
 */
void motion_step(void);

/**
 * @brief Mode off step function.
 * 
 */
void asserv_off_step(void);

/**
 * @brief Free step function for the asservissement.
 * 
 * This function is called when the robot is in free mode, allowing it to move without constraints.
 */
void asserv_free_step(void);

/**
 * @brief Step function for position asservissement.
 * 
 * This function calculates the speed orders based on the desired position and current state of the robot.
 */
void pos_asserv_step(void);

/**
 * @brief Step function for speed asservissement.
 * 
 * This function sets the speed orders based on the desired speed and activates the speed PID.
 */
void speed_asserv_step(void);

/**
 * @brief Step function for emergency break in speed asservissement.
 * 
 * This function checks if the robot is moving and applies an emergency break if necessary.
 */
void speed_asserv_break_step(void);

/**
 * @brief Step function for absolute speed asservissement.
 * 
 * This function calculates the speed orders based on the desired absolute speed and current robot orientation.
 */
void absolute_speed_asserv_step(void);

/**
 * @brief Checks if the robot is blocked.
 * 
 * This function checks if the robot's speed deviates significantly from the desired speed and handles the blocked state.
 * 
 * @param period The time period over which to check for blockage.
 */
void asserv_check_blocked(float period);

/**
 * @brief Get the status of the asservissement.
 * 
 * This function returns 1 if the asservissement is done (in OFF mode), otherwise returns 0.
 * 
 * @return int Returns 1 if asservissement is done, otherwise 0.
 */
int Get_asserv_done();

/**
 * @brief Calculates the radial speed based on the distance to the target.
 * 
 * This function computes the radial speed required to reach a target distance using a predefined maximum acceleration.
 * 
 * @param distance The distance to the target in meters.
 * @return float Returns the calculated radial speed in m/s.
 */
float radial_speed_calculation(float distance);

/**
 * @brief Calculates the angular speed based on the angle to the target.
 * 
 * This function computes the angular speed required to reach a target angle using a predefined maximum angular acceleration.
 * 
 * @param angle The angle to the target in radians.
 * @return float Returns the calculated angular speed in rad/s.
 */
float angular_speed_calculation(float angle);

#endif // _ASSERV_H_
