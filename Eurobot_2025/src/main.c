#include "main.h"

#define sev() __asm__("sev")
#define ARM1_STARTADR 0xFFFFFFF0
#define ARM1_BASEADDR 0x10080000

int old_timer_ms1 = 0;
int Status = 0;
int f = 0;

XGpio gpio;
u32 data;

int main()
{
    //Disable cache on OCM    
    // S=b1 TEX=b100 AP=b11, Domain=b1111, C=b0, B=b0
    Xil_SetTlbAttributes(0xFFFF0000,0x14de2); 
    
    
    u8 c;
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

    Status = dma_init();
    if (Status != XST_SUCCESS) {
        xil_printf("DMA init failed\n\r");
        Status = 0;
    } else {
        xil_printf("DMA init done\n\r");
        Status = 0;
    }
    // Remplir buffer d'envoi
    for (int i = 0; i < DMA_BUFFER_SIZE; i++) {
        DMA_tx_buffer[i] = i & 0xFF;
    }

        
    XGpio_Initialize(&gpio, XPAR_AXI_GPIO_0_DEVICE_ID);
    XGpio_SetDataDirection(&gpio, 1, 0xFFFFFFFF); // canal 1 en input
    // init_QEI();
    // PWM_Init();
    Std_Com_Init();
    init_AU();
    // ws2812b_init();
    // init_switch();
    // Init_Pump();
    // Init_Valve();
    // Init_Asserv();
    // Init_Stepper();

    // init lidar_1 register
    // init_lidar(&lidar_1_reg);

    // init_shared_memory();

    xil_printf("Init done\n\r");
    while(1){
        if (Timer_ms1 - old_timer_ms1 >= 2000) {
            old_timer_ms1 = Timer_ms1;   
            data = XGpio_DiscreteRead(&gpio, 1);
            xil_printf("Data read from GPIO: %d\n\r", (int)data);

            // // Nettoyer caches avant DMA
            // Xil_DCacheFlushRange((UINTPTR)DMA_tx_buffer, DMA_BUFFER_SIZE);
            // Xil_DCacheInvalidateRange((UINTPTR)DMA_rx_buffer, DMA_BUFFER_SIZE);

            // xil_printf("Envoi vers FIFO...\r\n");
            // dma_send(DMA_tx_buffer, DMA_BUFFER_SIZE);

            // xil_printf("Réception depuis FIFO...\r\n");
            // dma_receive(DMA_rx_buffer, DMA_BUFFER_SIZE);

            // // Invalider cache après réception
            // Xil_DCacheInvalidateRange((UINTPTR)DMA_rx_buffer, DMA_BUFFER_SIZE);

            // // Vérif
            // for (int i = 0; i < 1024; i++) {
            //     xil_printf("rx[%d] = %d\r\n", i, DMA_rx_buffer[i]);
            // }

            // xil_printf("DMA test terminé\r\n");
        }


        if (Get_Std_In(&c)) {
            Interp(c);
        }


        AU_Loop();
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
