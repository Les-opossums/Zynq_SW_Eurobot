#include "main.h"

XScuTimer TimerInstance;
XScuTimer_Config *ConfigPtr;
XScuTimer *TimerInstancePtr = &TimerInstance;

XScuGic IntcInstance;
XScuGic_Config *IntcConfig;

int Timer_ms1 = 0;

int old_Timer = 0;



int Init_Timer_ms1(void){
	int Status = 0;

	xil_printf("========================================\n");
	xil_printf("=           Init Timer_ms1             =\n");
	xil_printf("========================================\n");
	ConfigPtr = XScuTimer_LookupConfig(TIMER_DEVICE_ID);


	Status = XScuTimer_CfgInitialize(TimerInstancePtr, ConfigPtr, 
										ConfigPtr->BaseAddr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Perform a self-test to ensure that the hardware was built correctly.
	 */
	Status = XScuTimer_SelfTest(TimerInstancePtr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	// Status = TimerSetupIntrSystem(&IntcInstance, TimerInstancePtr, TIMER_IRPT_INTR);
	// if (Status != XST_SUCCESS) {
	// 	return XST_FAILURE;
	// }

	XScuTimer_EnableInterrupt(TimerInstancePtr);

	/*
	 * Steup prescaler
	*/
	XScuTimer_SetPrescaler(TimerInstancePtr, TIMER_1ms_PRESCALER);

	/*
	 * Enable Auto reload mode.
	*/
	XScuTimer_EnableAutoReload(TimerInstancePtr);

	/*
	 * Load the timer counter register.
	 */
	XScuTimer_LoadTimer(TimerInstancePtr, TIMER_1ms_LOAD_VALUE);

	/*
	 * Start the timer counter and then wait for it to timeout
	 */
	XScuTimer_Start(TimerInstancePtr);
	return XST_SUCCESS;
}


/*****************************************************************************/
/**
*
* This function is the Interrupt handler for the Timer interrupt of the
* Timer device. It is called on the expiration of the timer counter in
* interrupt context.
*
* @param	CallBackRef is a pointer to the callback function.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void TimerIntrHandler(void *CallBackRef)
{
	XScuTimer *TimerInstancePtr = (XScuTimer *) CallBackRef;

	/*
	 * Check if the timer counter has expired, checking is not necessary
	 * since that's the reason this function is executed, this just shows
	 * how the callback reference can be used as a pointer to the instance
	 * of the timer counter that expired, increment a shared variable so
	 * the main thread of execution can see the timer expired.
	 */
	if (XScuTimer_IsExpired(TimerInstancePtr)) {
		XScuTimer_ClearInterruptStatus(TimerInstancePtr);
		Timer_ms1 += 1;
	}
}

void Delay_ms(int ms) {
	u32 old_Timer = Timer_ms1;
	while (Timer_ms1 - old_Timer < ms);
}

