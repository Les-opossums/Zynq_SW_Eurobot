#ifndef TIMER_MANAGER_H
#define TIMER_MANAGER_H

#include "xil_types.h"

extern volatile int Timer_ms1;

int  Timer_Manager_Init(void);
void Delay_ms(int ms);

/* ================================================================== *
 * Timer microseconde (Global Timer du Cortex-A9)
 * ================================================================== *
 * Le Global Timer est un compteur UNIQUE, partage physiquement par les
 * deux cœurs (registres identiques vus des deux cotes). Timer_us_Init()
 * ne doit donc etre appele qu'UNE SEULE FOIS, par CPU0, AVANT le reveil
 * de CPU1 (sinon on reinitialiserait la base de temps de CPU1 pendant
 * qu'il tourne deja). CPU1 ne fait que lire via Timer_us_Get()/Timer_us1.
 */
void     Timer_us_Init(void);
uint32_t Timer_us_Get(void);

/* Compatibilite avec les macros T_START/T_STOP (cf asserv_loop.c) qui
 * utilisent Timer_us1 comme une simple valeur lisible. */
#define Timer_us1 (Timer_us_Get())

#endif /* TIMER_MANAGER_H */