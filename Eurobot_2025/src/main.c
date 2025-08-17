#include "main.h"

#define sev() __asm__("sev")
#define ARM1_STARTADR 0xFFFFFFF0
#define ARM1_BASEADDR 0x10080000

int old_timer_ms1 = 0;
int Status = 0;

int timer_lidar = 0;


LD19Instance LD19;


int main()
{
    //Disable cache on OCM    
    // S=b1 TEX=b100 AP=b11, Domain=b1111, C=b0, B=b0
    Xil_SetTlbAttributes(0xFFFF0000,0x14de2); 
    
    u8 c;
    u8 test;
    init_platform();

    

    print("ARM0: writing startaddress for ARM1\n\r");
    Xil_Out32(ARM1_STARTADR, ARM1_BASEADDR);
    dmb(); //waits until write has finished

    print("ARM0: sending the SEV to wake up ARM1\n\r");
    sev();

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

    Status = Init_Timer_ms1();
    if (Status != XST_SUCCESS) {
        xil_printf("Timer init failed\n\r");
        Status = 0;
    } else {
        xil_printf("Timer init done\n\r");
        Status = 0;
    }

    Status = UART_PL_Init();
    if (Status != XST_SUCCESS) {
        xil_printf("UART PL init failed\n\r");
        Status = 0;
    } else {
        xil_printf("UART PL init done\n\r");
        Status = 0;
    }

    // init_QEI();
    // PWM_Init();
    Std_Com_Init();
    // init_AU();
    // ws2812b_init();
    // init_switch();
    // Init_Pump();
    // Init_Valve();
    // Init_Asserv();
    // Init_Stepper();

    LD19_init(&LD19);

    init_shared_memory();

    xil_printf("Init done\n\r");

    u8 f = 0;
    while(1){
        if (Timer_ms1 - old_timer_ms1 >= 1000) {
            old_timer_ms1 = Timer_ms1;
        }

        if (LD19_readScan(&LD19, &UartLite)) {
            LD19_printScanTeleplot(&LD19);
        }

        if (Get_Std_In(&c)) {
            Interp(c);
        }

        

        // AU_Loop();
        // LED_loop();
        Std_Com_Loop();
        // Print_Position_loop();
        // if(AU_state == 1){
        //     LED_AU();
        //     Init_Pump();
        //     PWM_Init();
        // }else{
        //     // stepper functions
        //     Init_Stepper_Loop();
        //     Stepper_Loop();

        //     LED_CLASSIC_MODE();
        // PWM_Loop();

        //     Pump_Loop();
        //     Valve_Loop();
        // }
        // IHM_loop();

    }
    cleanup_platform();
    return 0;
}
