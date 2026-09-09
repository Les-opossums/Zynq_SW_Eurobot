#include "watchdog.h"
#include "../CORE_ID/core_id.h"
#include "../IPC_MANAGER/IPC_manager.h"       /* IPC_DATA : heartbeats + wd_suspended */
#include "../SYSTEM_MANAGER/system_reset.h"   /* System_Reboot()                     */
#include "../TIMER_MANAGER/timer_manager.h"   /* Timer_ms1                           */

static volatile uint32_t wd_timeout_ms;
static volatile uint8_t  wd_enabled;
static volatile uint8_t  wd_monitor_peer;        /* surveiller le heartbeat du pair ? (0 en mode degrade) */
static volatile uint32_t wd_self_last_kick_ms;   /* dernier "kick" de la boucle de ce coeur */
static volatile uint32_t wd_peer_last_hb;        /* derniere valeur vue du heartbeat pair    */
static volatile uint32_t wd_peer_last_change_ms; /* date du dernier changement du hb pair     */

/* Heartbeat de l'AUTRE coeur, lu dans la memoire partagee. */
static inline uint32_t wd_peer_hb(void) {
#if THIS_CORE_ID == CORE_ID_CPU0
    return IPC_DATA->hb_cpu1;
#else
    return IPC_DATA->hb_cpu0;
#endif
}

void Watchdog_Init(uint32_t timeout_ms) {
    wd_timeout_ms          = (timeout_ms > 0U) ? timeout_ms : WATCHDOG_DEFAULT_TIMEOUT_MS;
    wd_enabled             = 0U;
    wd_self_last_kick_ms   = (uint32_t)Timer_ms1;
    wd_peer_last_hb        = wd_peer_hb();
    wd_peer_last_change_ms = (uint32_t)Timer_ms1;
}

void Watchdog_Enable(uint8_t monitor_peer) {
    const uint32_t now     = (uint32_t)Timer_ms1;
    wd_self_last_kick_ms   = now;
    wd_peer_last_hb        = wd_peer_hb();
    wd_peer_last_change_ms = now;
    wd_monitor_peer        = monitor_peer ? 1U : 0U;
    wd_enabled             = 1U;
}

void Watchdog_Kick(void) {
    wd_self_last_kick_ms = (uint32_t)Timer_ms1;
#if THIS_CORE_ID == CORE_ID_CPU0
    IPC_DATA->hb_cpu0++;
#else
    IPC_DATA->hb_cpu1++;
#endif
}

void Watchdog_Suspend(void) {
    IPC_DATA->wd_suspended = 1U;
}

void Watchdog_Resume(void) {
    const uint32_t now     = (uint32_t)Timer_ms1;
    /* Re-references fraiches avant de reprendre, pour ne pas declencher sur le
     * temps ecoule pendant la suspension. */
    wd_self_last_kick_ms   = now;
    wd_peer_last_hb        = wd_peer_hb();
    wd_peer_last_change_ms = now;
    IPC_DATA->wd_suspended = 0U;
}

void Watchdog_Check(uint32_t now_ms) {
    if (!wd_enabled) return;

    /* Suspendu (ex: OTA) : on garde les references fraiches pour eviter tout
     * faux declenchement a la reprise, et on ne surveille pas. */
    if (IPC_DATA->wd_suspended) {
        wd_self_last_kick_ms   = now_ms;
        wd_peer_last_hb        = wd_peer_hb();
        wd_peer_last_change_ms = now_ms;
        return;
    }

    /* 1) Boucle principale de CE coeur bloquee ? */
    if ((uint32_t)(now_ms - wd_self_last_kick_ms) > wd_timeout_ms) {
        System_Reboot(); /* ne revient jamais */
    }

    /* 2) Heartbeat de l'AUTRE coeur fige ? (couvre un blocage de l'autre coeur
     *    meme si ses propres interruptions ne tournent plus). Uniquement si le
     *    pair est cense etre present (desactive en mode degrade). */
    if (wd_monitor_peer) {
        const uint32_t peer = wd_peer_hb();
        if (peer != wd_peer_last_hb) {
            wd_peer_last_hb        = peer;
            wd_peer_last_change_ms = now_ms;
        } else if ((uint32_t)(now_ms - wd_peer_last_change_ms) > wd_timeout_ms) {
            System_Reboot(); /* ne revient jamais */
        }
    }
}
