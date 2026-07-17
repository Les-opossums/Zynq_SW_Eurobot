#ifndef TIMER_MANAGER_H
#define TIMER_MANAGER_H

#include "xil_types.h"

extern volatile u32 Timer_ms1;

int Timer_Manager_Init(void);

#endif /* TIMER_MANAGER_H */