#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <stdint.h>

/* =========================================================================
 * Watchdog logiciel + heartbeat inter-coeur
 * =========================================================================
 * Surveille que la boucle principale de CHAQUE coeur continue de tourner. La
 * verification s'effectue dans l'ISR du timer 1 ms (contexte independant de la
 * boucle) : si la boucle de ce coeur n'a plus "kicke" depuis WATCHDOG_TIMEOUT_MS,
 * ou si le heartbeat de l'AUTRE coeur ne progresse plus, on redemarre le PS
 * (System_Reboot). Ainsi un blocage d'un coeur (attente active, spin driver...)
 * fait repartir le robot au lieu de le laisser fige.
 *
 * Portee : detecte les blocages tant que l'ISR du timer 1 ms de l'un des deux
 * coeurs tourne encore (cas realiste : boucle bloquee, interruptions actives).
 * Un blocage des DEUX coeurs interruptions coupees simultanement echapperait a
 * ce mecanisme -> un vrai watchdog materiel (SWDT) serait le complement ultime.
 * ========================================================================= */

#define WATCHDOG_DEFAULT_TIMEOUT_MS  1000U

/* A appeler une fois, tot (apres Timer_Manager_Init). N'arme PAS encore. */
void Watchdog_Init(uint32_t timeout_ms);

/* Arme le watchdog. A appeler une fois les 2 coeurs synchronises
 * (apres IPC_SyncCores), pour ne pas se declencher pendant le boot. */
void Watchdog_Enable(void);

/* A appeler a CHAQUE tour de la boucle principale : signale que ce coeur est
 * vivant et fait progresser son heartbeat vu par l'autre coeur. */
void Watchdog_Kick(void);

/* A appeler depuis l'ISR du timer 1 ms. Declenche System_Reboot() si la boucle
 * de CE coeur est bloquee, ou si le heartbeat de l'AUTRE coeur ne progresse
 * plus. No-op tant que Watchdog_Enable() n'a pas ete appele. */
void Watchdog_Check(uint32_t now_ms);

/* Suspend / reprend la surveillance (etat partage entre les 2 coeurs via IPC).
 * A utiliser autour d'une operation qui prend legitimement la main sur la
 * boucle plus de WATCHDOG_TIMEOUT_MS (ex: reflash OTA). */
void Watchdog_Suspend(void);
void Watchdog_Resume(void);

#endif /* WATCHDOG_H */
