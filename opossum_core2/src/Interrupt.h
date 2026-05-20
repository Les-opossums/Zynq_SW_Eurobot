#ifndef INTERRUPT_H
#define INTERRUPT_H

#define INTC_DEVICE_ID          XPAR_SCUGIC_SINGLE_DEVICE_ID

#define CAN_IRPT_INTR           XPAR_XCANPS_1_INTR
#define TIMER_IRPT_INTR         XPAR_SCUTIMER_INTR
#define SHARE_MEM_SGI_INT_ID    14         // ID de l'interruption logicielle inter core

extern XScuGic InterruptController;

int SetupInterruptSystem(XScuGic *GicInstancePtr);

#endif // INTERRUPT_H