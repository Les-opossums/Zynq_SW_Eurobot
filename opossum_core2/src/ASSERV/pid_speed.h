#ifndef __PID_SPEED_H_
#define __PID_SPEED_H_

#include "../../../opossum_common/common_type.h"

// coefficient du PID (proportionnel, integrale, derivee) et leurs coefs de moyennage pour l'asserv en vitesse
// lineaire
extern PID_speed pid_speed;

extern uint8_t Pid_Speed_En;

/******************************    Fonctions    *******************************/
/**
 * @brief Initialize the PID for speed control.
 */
void pid_vitesse_init(void);

/**
 * @brief Reset the integral errors of the PID.
 *
 * This function resets the integral errors for all three PID error components.
 */
void pid_vitesse_reset(void); // reset des erreurs integrales des pid

/******************************    Fonctions Utilitaires   *******************************/

/**
 * @brief Process the PID speed control.
 *
 * This function calculates the PID control commands based on the current errors.
 *
 * @param pid Pointer to the PID_speed structure containing PID coefficients and errors.
 * @param err1 Error for the first PID component.
 * @param err2 Error for the second PID component.
 * @param err3 Error for the third PID component.
 * @param err4 Error for the fourth PID component.
 * @return ESC_Command The calculated command for each motor.
 */
ESC_Command pid_speed_processing(PID_speed *pid, float err1, float err2, float err3, float err4);

#endif