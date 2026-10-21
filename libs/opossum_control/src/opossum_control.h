#ifndef OPOSSUM_CONTROL_H
#define OPOSSUM_CONTROL_H

/* ------------------------------------------------------------------------
 * opossum_control : odometrie, generation de trajectoire, contraintes,
 * PID de vitesse et filtre de Kalman du robot. C pur, sans dependance a la
 * plateforme (cf control_hal.h) : le meme code tourne sur le CPU1 du Zynq
 * et sur PC (tests, simulation SIL).
 *
 * Point d'entree : control_loop.h. Les autres en-tetes exposent les modules
 * internes (et leurs variables globales) pour le debug et les tests.
 * ------------------------------------------------------------------------ */

#include "control_types.h"
#include "control_hal.h"

/* Filtre d'etat */
#include "kalman.h"
#include "kalman_FIFO.h"

/* Chaine de commande */
#include "asserv_default.h"
#include "PWM_Calculator.h"
#include "motion.h"
#include "odometry.h"
#include "pid_speed.h"
#include "speed_constrainer.h"

/* Sequencement (boucles rapide / lente) */
#include "control_loop.h"

#endif /* OPOSSUM_CONTROL_H */
