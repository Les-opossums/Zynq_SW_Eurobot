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

#include "QEI.h"
#include "Timer.h"
#include "Interrupt.h"
#include "Asserv_Loop.h"
#include "PWM.h"
#include "CAN.h"
#include "UART.h"
#include "Std_Com.h"
#include "interpreteur.h"
#include "User.h"

#include "ydlidar_driver.h"
#include "Serial.h"
#include "ydlidar_protocol.h"

#include "xscugic.h"

//#define DEBUG
