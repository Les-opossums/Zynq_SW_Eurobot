#ifndef MAIN_H
#define MAIN_H

#include <stdio.h>
#include <sleep.h>
#include <xil_io.h>
#include <xparameters.h>
#include <math.h>

#include "xpseudo_asm.h"
#include "xil_mmu.h"
#include "platform.h"
#include "xil_printf.h"
#include "xgpio.h"
#include "xscutimer.h"
#include "xil_exception.h"
#include "xparameters_ps.h"
#include "xscugic.h"
#include "xuartps.h"
#include "xplatform_info.h"
#include "xaxidma.h"


#include "xuartlite.h"

// driver part 
#include "Timer.h"
#include "UART.h"
#include "Interrupt.h"

// pl part
#include "PWM.h"
#include "QEI.h"
#include "AU.h"
#include "ws2812b.h"

#include "pump.h"
#include "Valve.h"
#include "Stepper.h"

//divers
#include "User.h"
#include "IHM.h"

//asserv part 
#include "Cmd_For_Move.h"

//com part
#include "Std_Com.h"
#include "interpreteur.h"

// shared memory part
#include "shared_memory.h"
#include "shared_memory_structure.h"
//#define DEBUG

// uart pl
#include "UART_PL.h"

#include "lib_lidar/ld19.h"
#include "lib_lidar/obstacle_extractor.h"
#include "lib_lidar/obstacle_tracker.h"

#endif // MAIN_H
