#ifndef IRQ_MANAGER_H
#define IRQ_MANAGER_H

#include "xscugic.h"
#include "xil_exception.h"

int IRQ_Manager_Init(void);

int IRQ_Manager_Connect(u32 irq_id, Xil_InterruptHandler handler, void *callback_ref);

void IRQ_Manager_Start(void);

XScuGic *IRQ_Manager_GetInstance(void);

#endif