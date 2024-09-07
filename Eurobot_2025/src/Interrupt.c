#include "main.h"

XScuGic InterruptController;

int SetupInterruptSystem(XScuGic *GicInstancePtr) {
    int Status;
    XScuGic_Config *GicConfig;

    /*
     * Initialize the interrupt controller driver so that it is ready to
     * use.
    */
    GicConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
    if (NULL == GicConfig) {
        return XST_FAILURE;
    }

    Status = XScuGic_CfgInitialize(GicInstancePtr, GicConfig, 
                        GicConfig->CpuBaseAddress);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    // --------------------------------------------------------
    // ---------------------- Interrupts ----------------------
    // --------------------------------------------------------

    // ---------------------- Timer ----------------------
    // Connect and enable Timer interrupt
    Status = XScuGic_Connect(GicInstancePtr, TIMER_IRPT_INTR,
                    (Xil_InterruptHandler)TimerIntrHandler, &TimerInstance);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }
    XScuGic_Enable(GicInstancePtr, TIMER_IRPT_INTR);

    // ---------------------- CAN ----------------------
    // Connect and enable CAN interrupt
    Status = XScuGic_Connect(GicInstancePtr, CAN_IRPT_INTR,
                    (Xil_InterruptHandler)XCanPs_IntrHandler, &CanInstance);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }
    XScuGic_Enable(GicInstancePtr, CAN_IRPT_INTR);

    // ---------------------- UART 0 ----------------------
    // Connect and enable UART 0 interrupt
    Status = XScuGic_Connect(GicInstancePtr, UART0_IRPT_INTR,
                    (Xil_InterruptHandler)XUartPs_InterruptHandler, &UartInstance);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }
    XScuGic_Enable(GicInstancePtr, UART0_IRPT_INTR);

    // ---------------------- UART 1 ----------------------
    // Connect and enable UART 1 interrupt
    Status = XScuGic_Connect(GicInstancePtr, UART1_IRPT_INTR,
                    (Xil_InterruptHandler)XUartPs_InterruptHandler, &Uart1_Instance);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }
    XScuGic_Enable(GicInstancePtr, UART1_IRPT_INTR);

    // --------------------------------------------------------


    /*
     * Connect the interrupt controller interrupt handler to the hardware
     * interrupt handling logic in the processor.
    */
    Xil_ExceptionInit();

    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
                                (Xil_ExceptionHandler)XScuGic_InterruptHandler,
                                GicInstancePtr);
    Xil_ExceptionEnable();
    
    return XST_SUCCESS;
}
