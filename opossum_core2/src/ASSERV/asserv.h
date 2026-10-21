#ifndef ASSERV_H
#define ASSERV_H

/* Adaptateur Zynq (CPU1) de la bibliotheque opossum_control : services
 * plateforme utilises par asserv_loop.c et control_hal_zynq.c. */
#include <stdint.h>

#include "../../../opossum_common/APP_MOTORS/c610_feedback.h"
#include "../../../opossum_common/IO_config.h"
#include "../../../opossum_common/IPC_MANAGER/IPC_manager.h"
#include "../../../opossum_common/TIMER_MANAGER/timer_manager.h"
#include "../../../opossum_common/common_type.h"

/* Asservissement + Kalman, C pur (libs/opossum_control) */
#include "../../../libs/opossum_control/src/opossum_control.h"

#include "asserv_loop.h"

/* Impression des temps d'execution des etapes d'opossum_control
 * (control_hal_zynq.c, no-op si TIMING_MEASURE est desactive). */
void ctrl_hal_zynq_print_timing(void);

#endif /* ASSERV_H */
