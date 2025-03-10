#include <stdio.h>
#include <xil_io.h>
#include <xparameters.h>
#include <math.h>

#include "platform.h"
#include "xil_printf.h"
#include "xgpio.h"
#include "xscutimer.h"
#include "xil_exception.h"
#include "xparameters_ps.h"
#include "xscugic.h"
#include "xcanps.h"
#include "xuartps.h"
#include "xplatform_info.h"

// driver part 
#include "Timer.h"
#include "CAN.h"
#include "UART.h"
#include "Interrupt.h"

// pl part
#include "PWM.h"
#include "QEI.h"
#include "AU.h"
#include "pump.h"

//divers
#include "User.h"

#include "ydlidar_driver.h"
#include "Serial.h"
#include "ydlidar_protocol.h"

//asserv part 
#include "Asserv_Loop.h"
#include "Cmd_For_Move.h"


//com part
#include "Std_Com.h"
#include "interpreteur.h"


//#define DEBUG
