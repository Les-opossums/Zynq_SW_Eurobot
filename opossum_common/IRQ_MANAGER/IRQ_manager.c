#include "IRQ_manager.h"

static XScuGic InterruptController;

int IRQ_Manager_Init(void) {
    int Status;
    XScuGic_Config *gICconfig;

    // Initialize the interrupt controller driver so that it is ready to use.
    gICconfig = XScuGic_LookupConfig(XPAR_SCUGIC_0_DEVICE_ID);
    if (gICconfig == NULL) {
        return XST_FAILURE;
    }

    // Initialize the interrupt controller
    Status = XScuGic_CfgInitialize(&InterruptController, gICconfig, gICconfig->CpuBaseAddress);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT, 
                                (Xil_ExceptionHandler)XScuGic_InterruptHandler, 
                                &InterruptController);

    return XST_SUCCESS;
}

int IRQ_Manager_Connect(u32 irq_id, Xil_InterruptHandler handler, void *callback_ref) {
    int Status;

    Status = XScuGic_Connect(&InterruptController, irq_id, handler, callback_ref);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    // Enable the interrupt
    XScuGic_Enable(&InterruptController, irq_id);

    return XST_SUCCESS;
}

void IRQ_Manager_Start(void) {
    // Enable interrupts in the processor.
    Xil_ExceptionEnable();
}

XScuGic *IRQ_Manager_GetInstance(void) {
    return &InterruptController;
}