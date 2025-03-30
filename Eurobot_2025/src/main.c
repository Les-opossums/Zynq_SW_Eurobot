#include "main.h"

int old_timer_ms1 = 0;
int Status = 0;

int main()
{
    u8 c;
    init_platform();

    Status = SetupInterruptSystem(&InterruptController);

    if (Status != XST_SUCCESS) {
        xil_printf("Interrupt Setup Failed\r\n");
    } else {
        xil_printf("Interrupt Setup Done\r\n");
    }

    Status = UART_Init();
    if (Status != XST_SUCCESS) {
        xil_printf("UART init failed\n\r");
        Status = 0;
    } else {
        xil_printf("UART init done\n\r");
        Status = 0;
    }

    Status = UART1_Init();
    if (Status != XST_SUCCESS) {
        xil_printf("UART1 init failed\n\r");
        Status = 0;
    } else {
        xil_printf("UART1 init done\n\r");
        Status = 0;
    }

    Status = Init_Timer_ms1();
    if (Status != XST_SUCCESS) {
        xil_printf("Timer init failed\n\r");
        Status = 0;
    } else {
        xil_printf("Timer init done\n\r");
        Status = 0;
    }

   Status = init_CAN();
   if (Status != XST_SUCCESS) {
       xil_printf("CAN init failed\n\r");
       Status = 0;
   } else {
       xil_printf("CAN init done\n\r");
       Status = 0;
   }

    // init_QEI();
//    PWM_Init();
    Std_Com_Init();
    // init_AU();
    Init_Pump();
    Init_Asserv();
    xil_printf("Init done\n\r");

    while(1){
        if (Timer_ms1 - old_timer_ms1 >= 100) {
            old_timer_ms1 = Timer_ms1;
            // xil_printf("Timer_ms1 = %d\n\r", Timer_ms1);
        }

        if (Get_Std_In(&c)) {
            Interp(c);
        }
        init_lidar_loop();
        // PWM_Loop();
        Std_Com_Loop();


        //////////////////////////////////// LIADAR //////////////////////////////////////
        Lidar_scan_Loop();
        Lidar_Calculation_loop();
        //////////////////////////////////////////////////////////////////////////////////
        // Can_Loop();
        // Asserv_Loop();
        // AU_Loop();
        Std_Com_Loop();
        // Pump_Loop();
    }
    cleanup_platform();
    return 0;
}

