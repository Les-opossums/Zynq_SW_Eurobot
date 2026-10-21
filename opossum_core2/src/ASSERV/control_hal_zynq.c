/**
 * @file control_hal_zynq.c
 * @brief Implementation Zynq (CPU1) des services plateforme de opossum_control
 *        (cf libs/opossum_control/src/control_hal.h).
 */

#include "asserv.h"
#include "../../../opossum_common/TIMER_MANAGER/timing_stats.h"
#include "xil_printf.h"

/* Message texte de suivi : console UART via xil_printf. */
void ctrl_hal_log(const char *msg)
{
    xil_printf("%s", msg);
}

/* Remontee FIABLE de motion_done a CORE0 (re-emise si le canal est occupe). */
void ctrl_hal_motion_done(uint32_t status)
{
    static struct {
        uint32_t motion_done;
    } local_data;

    local_data.motion_done = status;
    SEND_FIELD_RELIABLE(&local_data, motion_done);
}

/* ------------------------------------------------------------------------
 * Chronometrage des etapes des boucles (TIMING_MEASURE, timing_stats.h)
 * ------------------------------------------------------------------------ */
#if defined(TIMING_MEASURE)

static TimingStats ts_ctrl[CTRL_T_COUNT];
static uint32_t    ts_ctrl_start[CTRL_T_COUNT];

void ctrl_hal_timing_start(ctrl_timing_id_t id)
{
    ts_ctrl_start[id] = Timer_us1;
}

void ctrl_hal_timing_stop(ctrl_timing_id_t id)
{
    ts_update(&ts_ctrl[id], Timer_us1 - ts_ctrl_start[id]);
}

void ctrl_hal_zynq_print_timing(void)
{
    static const char *const names[CTRL_T_COUNT] = {
        "fast_odo", "fast_kalman", "fast_imu", "fast_fifo", "fast_total",
        "slow_motion", "slow_constr", "slow_pid"
    };
    for (int i = 0; i < CTRL_T_COUNT; i++) {
        ts_print_one(names[i], &ts_ctrl[i]);
    }
}

#else

void ctrl_hal_timing_start(ctrl_timing_id_t id) { (void)id; }
void ctrl_hal_timing_stop(ctrl_timing_id_t id)  { (void)id; }
void ctrl_hal_zynq_print_timing(void)           { }

#endif /* TIMING_MEASURE */
