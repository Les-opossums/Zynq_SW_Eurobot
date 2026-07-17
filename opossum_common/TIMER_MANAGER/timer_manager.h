#ifndef TIMER_MANAGER_H
#define TIMER_MANAGER_H

#include "xil_types.h"

extern volatile int Timer_ms1;

int  Timer_Manager_Init(void);
void Delay_ms(int ms);

#endif /* TIMER_MANAGER_H */