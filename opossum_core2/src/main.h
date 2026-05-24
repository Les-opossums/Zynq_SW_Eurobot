#include <stdio.h>
#include <sleep.h>
#include "xil_io.h"
#include "xil_mmu.h"
#include "platform.h"
#include "xil_cache.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xpseudo_asm.h"
#include "xil_exception.h"
#include "xscutimer.h"
#include "xscugic.h"
#include "xcanps.h"
#include "xgpio.h"
#include <math.h>


#include "shared_memory.h"
#include "shared_memory_structure.h"

#include "Timer.h"

#include "interrupt.h"

#include "Asserv_Loop.h"
#include "CAN.h"
#include "AU.h"

#include "User.h"

#include "BNO085.h"