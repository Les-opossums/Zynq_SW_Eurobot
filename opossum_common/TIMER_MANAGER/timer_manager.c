#include "timer_manager.h"
#include "xstatus.h"
#include "xscutimer.h"
#include "xparameters.h"
#include "xtime_l.h"
#include "../IRQ_MANAGER/IRQ_manager.h"

#define TIMER_1ms_LOAD_VALUE	332999
#define TIMER_1ms_PRESCALER		0x0

/* Le Global Timer tourne a COUNTS_PER_SECOND (= frequence CPU / 2, cf
 * xtime_l.h), soit exactement la meme horloge que celle utilisee ci-dessus
 * pour le XScuTimer 1ms (332999 + 1 = 333000 ticks/ms => 333 MHz). */
#define TIMER_US_TICKS_PER_US   ((uint32_t)(COUNTS_PER_SECOND / 1000000ULL))

volatile int Timer_ms1 = 0;

static XScuTimer TimerInstance;

static void Timer_IntrHandler(void *CallBackRef) {
    XScuTimer *TimerInstancePtr = (XScuTimer *) CallBackRef;

    if (XScuTimer_IsExpired(TimerInstancePtr)) {
        XScuTimer_ClearInterruptStatus(TimerInstancePtr);
        Timer_ms1 += 1;
    }
}

int Timer_Manager_Init(void) {
    int Status;
    XScuTimer_Config *ConfigPtr;

    ConfigPtr = XScuTimer_LookupConfig(XPAR_SCUTIMER_DEVICE_ID);
    if (ConfigPtr == NULL) {
        return XST_FAILURE;
    }

    Status = XScuTimer_CfgInitialize(&TimerInstance, ConfigPtr, ConfigPtr->BaseAddr);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    Status = XScuTimer_SelfTest(&TimerInstance);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    // Connexion via l'IRQ_Manager commun, plutôt que le TimerSetupIntrSystem custom d'origine
    Status = IRQ_Manager_Connect(XPAR_SCUTIMER_INTR, (Xil_InterruptHandler)Timer_IntrHandler, &TimerInstance);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    XScuTimer_EnableInterrupt(&TimerInstance);
    XScuTimer_SetPrescaler(&TimerInstance, TIMER_1ms_PRESCALER);
    XScuTimer_EnableAutoReload(&TimerInstance);
    XScuTimer_LoadTimer(&TimerInstance, TIMER_1ms_LOAD_VALUE);
    XScuTimer_Start(&TimerInstance);

    return XST_SUCCESS;
}

void Delay_ms(int ms) {
    int old_Timer = Timer_ms1;
    while (Timer_ms1 - old_Timer < ms);
}

/* ================================================================== *
 * Timer microseconde (Global Timer, partage CPU0/CPU1)
 * ================================================================== */

void Timer_us_Init(void) {
    /* Remet le compteur global a 0. A n'appeler qu'UNE FOIS, cote CPU0,
     * avant le reveil de CPU1 (cf commentaire dans le .h). */
    XTime_SetTime(0);
}

uint32_t Timer_us_Get(void) {
    XTime now;
    XTime_GetTime(&now);
    return (uint32_t)(now / TIMER_US_TICKS_PER_US);
}