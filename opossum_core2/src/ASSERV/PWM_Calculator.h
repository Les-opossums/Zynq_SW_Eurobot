#ifndef __ASSERV_PWM_CALCULATOR_H_
#define __ASSERV_PWM_CALCULATOR_H_

#include "../../../opossum_common/common_type.h"

/**
 * @brief Calculates the PWM commands for the ESCs based on the current speed measurements and desired speed
 * orders.
 *
 * @param commande
 */
void Asserv_PWM_calculator(ESC_Command *commande);

#endif