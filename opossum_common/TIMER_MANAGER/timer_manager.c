#include "timer_manager.h"
#include "xstatus.h" 

volatile u32 Timer_ms1 = 0;

int Timer_Manager_Init(void) {
    // TODO: configurer un TTC (ou timer AXI) en interruption périodique 1ms
    // qui incrémente Timer_ms1 dans son ISR.
    return XST_SUCCESS;
}