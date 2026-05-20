#include "main.h"

XScuGic InterruptController;

// Déclaration externe de ta fonction de traitement de la mémoire partagée
extern void check_for_cmd_loop(void);

/**
 * @brief Routine de service d'interruption (ISR) pour la mémoire partagée.
 * Elle est appelée instantanément lorsque le Core 0 déclenche la SGI 14.
 */
static void SharedMem_InterruptHandler(void *CallbackRef) {
    (void)CallbackRef; // Évite le warning "unused variable"
    
    // On traite directement toutes les commandes reçues d'un coup
    check_for_cmd_loop();
}

int SetupInterruptSystem(XScuGic *GicInstancePtr) {
    int Status;
    XScuGic_Config *GicConfig;

    /*
     * Initialize the interrupt controller driver so that it is ready to use.
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

    // ---------------------- Mémoire Partagée (SGI) ----------------------
    // Connecte l'interruption logicielle 14 à notre Handler de réception
    Status = XScuGic_Connect(GicInstancePtr, SHARE_MEM_SGI_INT_ID,
                    (Xil_InterruptHandler)SharedMem_InterruptHandler, NULL);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }
    // Active l'écoute de cette SGI spécifique sur l'interface CPU du Core 1
    XScuGic_Enable(GicInstancePtr, SHARE_MEM_SGI_INT_ID);


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