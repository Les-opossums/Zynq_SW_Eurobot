#ifndef CONTROL_HAL_PC_H
#define CONTROL_HAL_PC_H

/* Implementation PC de control_hal.h (tests, simulation SIL). */

#include <stdint.h>

/* Redirige les messages de suivi (NULL : silencieux, defaut : stderr). */
void ctrl_hal_pc_set_log(void (*log_fn)(const char *msg));

/* Dernier motion_done remonte par l'asservissement (0 succes, 1 en
 * mouvement, 2 freine) et nombre de transitions depuis le demarrage. */
uint32_t ctrl_hal_pc_motion_done(void);
uint32_t ctrl_hal_pc_motion_done_count(void);

#endif /* CONTROL_HAL_PC_H */
