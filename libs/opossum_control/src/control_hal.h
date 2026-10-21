#ifndef CONTROL_HAL_H
#define CONTROL_HAL_H

/* ------------------------------------------------------------------------
 * Services plateforme requis par opossum_control.
 *
 * La bibliotheque ne connait ni l'IPC, ni le CAN, ni les timers du Zynq :
 * capteurs et consignes entrent par control_loop.h, les commandes moteur
 * en sortent. Les seuls besoins restants sont declares ici et implementes :
 *   - sur le Zynq : opossum_core2/src/ASSERV/control_hal_zynq.c
 *   - sur PC      : libs/opossum_control/pc/control_hal_pc.c (tests, SIL)
 * ------------------------------------------------------------------------ */

#include <stdint.h>

/* Message texte de suivi (xil_printf sur le Zynq). */
void ctrl_hal_log(const char *msg);

/* Changement d'etat de motion_done (motion_status_t : 0 succes, 1 en
 * mouvement, 2 freine). Appele a chaque transition, y compris a la reception
 * d'une commande : le Zynq le remonte immediatement a CORE0 par IPC. */
void ctrl_hal_motion_done(uint32_t status);

/* Chronometrage des etapes des boucles (TIMING_MEASURE sur le Zynq).
 * Toujours appele ; une implementation vide suffit si non utilise. */
typedef enum {
    CTRL_T_FAST_ODO = 0,
    CTRL_T_FAST_KALMAN,
    CTRL_T_FAST_IMU,
    CTRL_T_FAST_FIFO,
    CTRL_T_FAST_TOTAL,
    CTRL_T_SLOW_MOTION,
    CTRL_T_SLOW_CONSTRAIN,
    CTRL_T_SLOW_PID,       /* PID + saturation + coupure AU (sans l'envoi CAN) */
    CTRL_T_COUNT
} ctrl_timing_id_t;

void ctrl_hal_timing_start(ctrl_timing_id_t id);
void ctrl_hal_timing_stop(ctrl_timing_id_t id);

#endif /* CONTROL_HAL_H */
