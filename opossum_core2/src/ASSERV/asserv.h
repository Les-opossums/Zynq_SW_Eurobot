#ifndef ASSERV_H
#define ASSERV_H

/* Common platform services used by the CPU1 control application. */
#include <stdint.h>

#include "../../../opossum_common/APP_MOTORS/c610_feedback.h"
#include "../../../opossum_common/IO_config.h"
#include "../../../opossum_common/IPC_MANAGER/IPC_manager.h"
#include "../../../opossum_common/TIMER_MANAGER/timer_manager.h"
#include "../../../opossum_common/common_type.h"

/* State estimator. */
#include "../KALMAN/kalman.h"
#include "../KALMAN/kalman_FIFO.h"

/* Control stack. */
#include "PWM_Calculator.h"
#include "asserv_default.h"
#include "asserv_loop.h"
#include "motion.h"
#include "odometry.h"
#include "pid_speed.h"
#include "speed_constrainer.h"

#endif /* ASSERV_H */
