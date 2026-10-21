/**
 * @file control_hal_pc.c
 * @brief Implementation PC des services plateforme de opossum_control
 *        (tests unitaires, simulation SIL).
 */

#include "control_hal.h"
#include "control_hal_pc.h"
#include <stdio.h>

static void log_stderr(const char *msg) { fputs(msg, stderr); }

static void (*g_log)(const char *msg) = log_stderr;
static uint32_t g_motion_done;
static uint32_t g_motion_done_count;

void ctrl_hal_pc_set_log(void (*log_fn)(const char *msg)) { g_log = log_fn; }
uint32_t ctrl_hal_pc_motion_done(void) { return g_motion_done; }
uint32_t ctrl_hal_pc_motion_done_count(void) { return g_motion_done_count; }

void ctrl_hal_log(const char *msg)
{
    if (g_log != NULL) {
        g_log(msg);
    }
}

void ctrl_hal_motion_done(uint32_t status)
{
    g_motion_done = status;
    g_motion_done_count++;
}

/* Pas de chronometrage sur PC (le temps simule n'a pas de sens ici). */
void ctrl_hal_timing_start(ctrl_timing_id_t id) { (void)id; }
void ctrl_hal_timing_stop(ctrl_timing_id_t id)  { (void)id; }
