#include "main.h"
#include "lib_asserv/Lib_Asserv.h"

// #define COMM_VAL (*(volatile unsigned long *)(0xFFFF0000))
extern u32 MMUTable;

int old_timer_us1 = 0;
int old_timer_ms1 = 0;

int main()
{
    Xil_SetTlbAttributes(0xFFFF0000,0x14de2); 

    init_platform();
    print("CPU1: init_platform\n\r");


    int Status = SetupInterruptSystem(&InterruptController);

    if (Status != XST_SUCCESS) {
        xil_printf("Interrupt Setup Failed\r\n");
    } else {
        xil_printf("Interrupt Setup Done\r\n");
    }

    // initialise timer
    Status = Init_Timer_us1();
    if (Status != XST_SUCCESS) {
        xil_printf("CPU1: Timer initialization failed\n\r");
        return XST_FAILURE;
    }
    
    // initialise shared memory
    init_shared_memory();


    // Init_CAN();
    Init_AU();
    Init_Asserv();
    

    while(1){
        if(Timer_ms1 - old_timer_ms1 >= 100) {
            // printf("Position: %.4f, %.4f, %.4f\n", 
            //          (double)kalman_current_state.x[0], 
            //          (double)kalman_current_state.x[1], 
            //          (double)kalman_current_state.x[2]);
            old_timer_ms1 = Timer_ms1;
        }

        AU_Loop();
        check_for_cmd_loop();
        if(AU_state == 1){
            // motion_free();
            // Init_CAN_MOTOR_variables();
            // Init_Asserv();
        }else{
            // Asserv_Loop();
        }
    }

    cleanup_platform();
    return 0;
}
