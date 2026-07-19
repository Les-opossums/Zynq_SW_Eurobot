#ifndef TIMING_STATS_H
#define TIMING_STATS_H

/**
 * @file timing_stats.h
 * @brief Infrastructure commune de monitoring des temps d'execution (marge
 *        temps-reel), utilisable sur CPU0 ET CPU1.
 *
 * Un seul interrupteur (TIMING_MEASURE ci-dessous) active/desactive les
 * mesures sur les DEUX cœurs a la fois (chaque binaire est recompile
 * separement mais partage ce header).
 *
 * Usage typique dans un .c :
 *
 *   #include "TIMER_MANAGER/timing_stats.h"
 *   #if defined(TIMING_MEASURE)
 *   static TimingStats ts_ma_step;
 *   #endif
 *   ...
 *   #if defined(TIMING_MEASURE)
 *       T_START(ma_step);
 *   #endif
 *       ma_fonction_a_mesurer();
 *   #if defined(TIMING_MEASURE)
 *       T_STOP(ma_step, ts_ma_step);
 *   #endif
 *   ...
 *       ts_print_one("ma_step", &ts_ma_step); // dans une impression periodique
 *
 * NB: quand TIMING_MEASURE est desactive, TimingStats/T_START/T_STOP/
 * ts_update/ts_print_one n'existent plus : le code appelant doit rester
 * derriere des #if defined(TIMING_MEASURE) (cf exemple ci-dessus). Les
 * fonctions "PrintTiming" publiques des modules (IO_Manager, Com_Interpreter,
 * ...) restent en revanche TOUJOURS declarees/appelables (no-op si
 * desactive), pour ne pas polluer les points d'appel avec des #ifdef.
 */

// Decommenter pour activer les mesures + impressions xil_printf periodiques
// sur les deux cœurs :
// #define TIMING_MEASURE

#include "xil_types.h"
#include "timer_manager.h"

#if defined(TIMING_MEASURE)

#include "xil_printf.h"

typedef struct {
    uint32_t min_us;
    uint32_t max_us;
    uint32_t last_us;
    uint64_t sum_us;
    uint32_t count;
} TimingStats;

#define T_START(id)       uint32_t _t_##id = Timer_us1
#define T_STOP(id, stats) ts_update(&(stats), Timer_us1 - _t_##id)

static inline void ts_update(TimingStats *ts, uint32_t dt_us)
{
    if (ts->count == 0U || dt_us < ts->min_us) { ts->min_us = dt_us; }
    if (dt_us > ts->max_us)                    { ts->max_us = dt_us; }
    ts->last_us = dt_us;
    ts->sum_us += dt_us;
    ts->count++;
}

static inline void ts_print_one(const char *name, TimingStats *ts)
{
    if (ts->count == 0U) { return; }
    xil_printf("  %-12s min=%4u max=%4u avg=%4u last=%4u us (n=%u)\r\n",
               name, (unsigned)ts->min_us, (unsigned)ts->max_us,
               (unsigned)(ts->sum_us / ts->count), (unsigned)ts->last_us,
               (unsigned)ts->count);
    ts->min_us = 0xFFFFFFFFU;
    ts->max_us = 0U;
    ts->sum_us = 0U;
    ts->count  = 0U;
}

/* Aide au declenchement periodique (impression toutes les period_ms),
 * basee sur le timer milliseconde deja existant (Timer_ms1). */
static inline uint8_t ts_trigger_ms(uint32_t period_ms, uint32_t *last_ms)
{
    const uint32_t now = (uint32_t)Timer_ms1;
    if ((uint32_t)(now - *last_ms) >= period_ms) {
        *last_ms = now;
        return 1U;
    }
    return 0U;
}

#endif /* TIMING_MEASURE */

#endif /* TIMING_STATS_H */
