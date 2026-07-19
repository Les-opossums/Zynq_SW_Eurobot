#ifndef COM_INTERPRETER_LOOP_H
#define COM_INTERPRETER_LOOP_H

void Com_Interpreter_Update(void);

// Impression du temps d'execution de la reception/interpretation UART.
// Toujours declaree/appelable : no-op si TIMING_MEASURE n'est pas active.
void Com_Interpreter_PrintTiming(void);

#endif