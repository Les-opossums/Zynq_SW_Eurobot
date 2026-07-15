#ifndef MAIN_H
#define MAIN_H

#define FEETECH

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
#include "xgpiops.h"
#include "xscutimer.h"
#include "xil_exception.h"
#include "xparameters_ps.h"
#include "xscugic.h"
#include "xuartps.h"
#include "xplatform_info.h"
#include "xil_cache.h"


#include "xuartlite.h"

// driver part 
#include "Timer.h"
#include "Interrupt.h"

// pl part
#include "WS2812B/ws2812b.h"

//divers
#include "User.h"
#include "IHM.h"

//asserv part 
#include "Cmd_For_Move.h"

//com part
#include "UART_COM/UART.h"
#include "UART_COM/Std_Com.h"
#include "INTERPRETEUR/interpreteur.h"
// ethernet part
#include "ETHERNET/ETH_driver.h"
#include "ETHERNET/robot_messages.h"
#include "ETH_receiver.h"

// shared memory part
#include "shared_memory.h"
#include "shared_memory_structure.h"
//#define DEBUG

//feetech part
#include "FEETECH/feetech_UART.h"
#include "FEETECH/feetech.h"
#include "FEETECH/feetech_Action.h"
// uart pl
#include "UART_PL.h"

// LD19 part
#include "LD19/ld19.h"


//IO manager_part
#include "IO_MANAGER/IO_manager.h"
#include "IO_MANAGER/IO_config.h"

#endif // MAIN_H
