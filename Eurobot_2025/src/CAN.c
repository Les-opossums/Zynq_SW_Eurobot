#include "main.h"

XCanPs CanInstance; 	/* The instance of the CAN device */
/*
 * Buffers to hold frames to send and receive. These are declared as global so
 * that they are not on the stack.
 * These buffers need to be 32-bit aligned
 */
static u32 TxFrame[XCANPS_MAX_FRAME_SIZE_IN_WORDS];
static u32 RxFrame[XCANPS_MAX_FRAME_SIZE_IN_WORDS];

int angle_motor_1 = 0;
int angle_motor_2 = 0;
int angle_motor_3 = 0;

int torque_motor_1 = 0;
int torque_motor_2 = 0;
int torque_motor_3 = 0;

int speed_motor_1 = 0;
int speed_motor_2 = 0;
int speed_motor_3 = 0;

/*
 * Shared variables used to test the callbacks.
 */
volatile static int LoopbackError;	/* Asynchronous error occurred */
volatile static int RecvDone;		/* Received a frame */
volatile static int SendDone;		/* Frame was sent successfully */



int old_Can_Timer_ms1 = 0;
int can_loop_state = 0;
int init_can_done = 0;
int CAN_status = 0;
int counter = 0;
int nbr_loop = 0;

int motor1_current_order = 0;
int motor2_current_order = 0;
int motor3_current_order = 0;


void Can_Loop(void){
	switch (can_loop_state){
		case 0: 
			if (Timer_ms1 - old_Can_Timer_ms1 > 100){
				#ifdef DEBUG_CAN
					xil_printf("Motor 1: angle = %d, torque = %d, speed = %d\r\n", angle_motor_1, torque_motor_1, speed_motor_1);
					xil_printf("Motor 2: angle = %d, torque = %d, speed = %d\r\n", angle_motor_2, torque_motor_2, speed_motor_2);
					xil_printf("Motor 3: angle = %d, torque = %d, speed = %d\r\n", angle_motor_3, torque_motor_3, speed_motor_3);
				#endif
				old_Can_Timer_ms1 = Timer_ms1;

				TxFrame[2] = (u32)(motor2_current_order & 0xFF) << 24 | 
									((motor2_current_order >> 8) & 0xFF) << 16 | 
									((motor1_current_order & 0xFF) << 8) | 
									((motor1_current_order >> 8) & 0xFF);
				TxFrame[3] = (u32)(motor3_current_order & 0xFF) << 8 | 
									((motor3_current_order >> 8) & 0xFF);
				
				can_loop_state++;
			}
			break;
		case 1:
			SendFrame(&CanInstance);
			// can_loop_state = 0;
			can_loop_state++;
			break;
		case 2:
			if ((SendDone == TRUE) && (RecvDone == TRUE)){
				can_loop_state = 0;
				SendDone = FALSE;
				RecvDone = FALSE;
				if (LoopbackError == TRUE) {
					xil_printf("Error: Loopback Error occurred\r\n");
					LoopbackError = FALSE;
				}
			}
			break;		
	}
	
}



/*****************************************************************************/
/**
*
* This function is the init function for the CAN controller.
* It initializes the CAN controller with the following settings:
* - Baud Rate Prescalar
* - Bit Timing Register 0 (BTR0)
* - Bit Timing Register 1 (BTR1)
* - Acceptance Filter
*
******************************************************************************/
int init_CAN(void){
	XCanPs_Config *ConfigPtr;
	XCanPs *CanInstPtr = &CanInstance;

	// Initialize the TxFrame and RxFrame
	for (int i = 2; i < 8; i++){
		TxFrame[i] = 0;
		RxFrame[i] = 0;
	}

	int Status;
	// Initialize the CAN driver
	ConfigPtr = XCanPs_LookupConfig(CAN_DEVICE_ID);
	if (ConfigPtr == NULL) {
		xil_printf("Error: Unable to lookup CAN device\r\n");
		return XST_FAILURE;
	}else{
		xil_printf("CAN device found\r\n");
	}

	XCanPs_CfgInitialize(CanInstPtr, ConfigPtr, ConfigPtr->BaseAddr);
	Status = XCanPs_SelfTest(CanInstPtr);
	if (Status != XST_SUCCESS) {
		xil_printf("Error: Unable to initialize CAN device\r\n");
		return XST_FAILURE;
	} else {
		xil_printf("CAN device initialized\r\n");
	}

	/*
	 *configure the CAN controller
	 */
	Status = Config(CanInstPtr);
	if (Status != XST_SUCCESS) {
		xil_printf("Error: Unable to configure CAN device\r\n");
		return XST_FAILURE;
	} else {
		xil_printf("CAN device configured\r\n");
	}
    
    /*
	 * Configer filters.
	*/
    Status = CAN_configure_filters();
	if (Status != XST_SUCCESS) {
		xil_printf("Error: Unable to configure filters\r\n");
		return XST_FAILURE;
	} else {
		xil_printf("Filters configured\r\n");
	}

	/*
	 * Set interrupt handlers.
	 */
	XCanPs_SetHandler(CanInstPtr, XCANPS_HANDLER_SEND,
			  (void *)SendHandler, (void *)CanInstPtr);
	XCanPs_SetHandler(CanInstPtr, XCANPS_HANDLER_RECV,
			  (void *)RecvHandler, (void *)CanInstPtr);
	XCanPs_SetHandler(CanInstPtr, XCANPS_HANDLER_ERROR,
			  (void *)ErrorHandler, (void *)CanInstPtr);
	XCanPs_SetHandler(CanInstPtr, XCANPS_HANDLER_EVENT,
			  (void *)EventHandler, (void *)CanInstPtr);

	/*
	 * Initialize the flags.
	 */
	SendDone = FALSE;
	RecvDone = FALSE;
	LoopbackError = FALSE;

	/*
	 * Enable the interrupt of the CAN device.
	 */
	XCanPs_IntrEnable(CanInstPtr, XCANPS_IXR_ALL);

	XCanPs_EnterMode(CanInstPtr, XCANPS_MODE_NORMAL);
	while (XCanPs_GetMode(CanInstPtr) != XCANPS_MODE_NORMAL);
	// Perform any other initialization steps or start your application here
	xil_printf("CAN initialized successfully\r\n");
	return XST_SUCCESS;
}

int CAN_configure_filters(void){
    int Status;
	Status = XCanPs_IsAcceptFilterBusy(&CanInstance);
	if (Status != XST_SUCCESS) {
		xil_printf("Error: Acceptance filter is busy\r\n");
		return XST_FAILURE;
	}
    // configure acceptance filter 0
    XCanPs_AcceptFilterDisable(&CanInstance, XCANPS_AFR_UAF1_MASK);
    Status = XCanPs_AcceptFilterSet(&CanInstance, XCANPS_AFR_UAF1_MASK, CAN_MOTOR_FILTER_MASK, CAN_MOTOR_1_ID); // FilterIndex = 0, MaskValue = 0x0203 , IdValue = 0
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }
    XCanPs_AcceptFilterEnable(&CanInstance, XCANPS_AFR_UAF1_MASK);

     // configure acceptance filter 1
    XCanPs_AcceptFilterDisable(&CanInstance, XCANPS_AFR_UAF2_MASK);
    Status = XCanPs_AcceptFilterSet(&CanInstance, XCANPS_AFR_UAF2_MASK, CAN_MOTOR_FILTER_MASK, CAN_MOTOR_2_ID); // FilterIndex = 1, MaskValue = 0x0203 , IdValue = 0
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }
    XCanPs_AcceptFilterEnable(&CanInstance, XCANPS_AFR_UAF2_MASK);

	// configure acceptance filter 2
	XCanPs_AcceptFilterDisable(&CanInstance, XCANPS_AFR_UAF3_MASK);
	Status = XCanPs_AcceptFilterSet(&CanInstance, XCANPS_AFR_UAF3_MASK, CAN_MOTOR_FILTER_MASK, CAN_MOTOR_3_ID); // FilterIndex = 2, MaskValue = 0x0203 , IdValue = 0
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	XCanPs_AcceptFilterEnable(&CanInstance, XCANPS_AFR_UAF3_MASK);

	return XST_SUCCESS;
}

void CAN_create_message(CAN_Message *message, uint16_t id, uint8_t buffer, uint8_t *payload, uint8_t valid_bytes){
    message->id = id;
    message->buffer = buffer;
    message->valid_bytes = valid_bytes;
    for(int i = 0; i <= valid_bytes; i++){
        message->payload[i] = payload[i];
    }
}

void CAN_transmit(XCanPs *InstancePtr){

    u8 *FramePtr;
	int Index;
	int Status;

	/*
	 * Create correct values for Identifier and Data Length Code Register.
	 */
	TxFrame[0] = (u32)XCanPs_CreateIdValue((u32)ESC_TX_MESSAGE_ID, 0, 0, 0, 0);
	TxFrame[1] = (u32)XCanPs_CreateDlcValue((u32)FRAME_DATA_LENGTH);

	/*
	 * Now fill in the data field with known values so we can verify them
	 * on receive.
	 */
	FramePtr = (u8 *)(&TxFrame[2]);
	for (Index = 0; Index < FRAME_DATA_LENGTH; Index++) {
		*FramePtr++ = (u8)Index;
	}

	/*
	 * Now wait until the TX FIFO is not full and send the frame.
	 */
	while (XCanPs_IsTxFifoFull(InstancePtr) == TRUE);

	Status = XCanPs_Send(InstancePtr, TxFrame);
	if (Status != XST_SUCCESS) {
		xil_printf("Error: Unable to send frame\r\n");
		/*
		 * The frame could not be sent successfully.
		 */
		LoopbackError = TRUE;
		SendDone = TRUE;
		RecvDone = TRUE;
	}
}


/*******************************************************/
/**
 * This function configures the CAN controller with the following settings:
 * - Baud Rate Prescalar
 * - Bit Timing Register 0 (BTR0)
 * - Bit Timing Register 1 (BTR1)
 * 
 * @param	InstancePtr is a pointer to the XCanPs instance.
 * 
 * @return	None.
 * 
 * @note		None.
 */
/*******************************************************/
int Config(XCanPs *InstancePtr){
	/*
	 * Enter configuration mode
	 */
	int status;
	XCanPs_EnterMode(InstancePtr, XCANPS_MODE_CONFIG);
	while(XCanPs_GetMode(InstancePtr) != XCANPS_MODE_CONFIG);
	xil_printf("CAN in config mode\r\n");
	/*
	 * Set Baud Rate Prescalar BRPR
	 * and Bit Timing Register 0 (BTR0) and Bit Timing Register 1 (BTR1)
	 */
	status = XCanPs_SetBaudRatePrescaler(InstancePtr, BRPR_BAUD_PRESCALAR);
	if (status != XST_SUCCESS) {
		xil_printf("Error: Unable to set the baud rate prescalar\r\n");
		return XST_FAILURE;
	}
	status = XCanPs_SetBitTiming(InstancePtr, 	BTR_SYNCJUMPWIDTH,
										BTR_SECOND_TIMESEGMENT, 
										BTR_FIRST_TIMESEGMENT);
	if (status != XST_SUCCESS) {
		xil_printf("Error: Unable to set the bit timing\r\n");
		return XST_FAILURE;
	}
	return XST_SUCCESS;
}

/*******************************************************/
/**
 * This function sends a CAN frame.
 * 
 * @param	InstancePtr is a pointer to the XCanPs instance.
 * 
 * @return	None.
 * 
 * @note		None.
 */
/*******************************************************/
void SendFrame(XCanPs *InstancePtr){
	int Status;
	
	/*
	 * Create correct values for Identifier and Data Length Code Register.
	 */
	TxFrame[0] = (u32)XCanPs_CreateIdValue((u32)ESC_TX_MESSAGE_ID, 0, 0, 0, 0);
	TxFrame[1] = (u32)XCanPs_CreateDlcValue((u32)FRAME_DATA_LENGTH);

	/*
	 * Now wait until the TX FIFO is not full and send the frame.
	 */
	while (XCanPs_IsTxFifoFull(InstancePtr) == TRUE);
	
	Status = XCanPs_Send(InstancePtr, TxFrame);
	if (Status != XST_SUCCESS) {
		xil_printf("Error: Unable to send frame\r\n");
		/*
		 * The frame could not be sent successfully.
		 */
		LoopbackError = TRUE;
		SendDone = TRUE;
		RecvDone = TRUE;
	}
}

/*******************************************************/
/**
*
* Callback function (called from interrupt handler) to handle confirmation of
* transmit events when in interrupt mode.
*
* @param	CallBackRef is the callback reference passed from the interrupt
*		handler, which in our case is a pointer to the driver instance.
*
* @return	None.
*
* @note		This function is called by the driver within interrupt context.
*
******************************************************************************/
void SendHandler(void *CallBackRef)
{
	/*
	 * The frame was sent successfully. Notify the task context.
	 */
	#ifdef DEBUG_CAN
		xil_printf("Frame sent\r\n");
	#endif
	SendDone = TRUE;
}

/*****************************************************************************/
/**
*
* Callback function (called from interrupt handler) to handle frames received in
* interrupt mode.  This function is called once per frame received.
* The driver's receive function is called to read the frame from RX FIFO.
*
* @param	CallBackRef is the callback reference passed from the interrupt
*		handler, which in our case is a pointer to the device instance.
*
* @return	None.
*
* @note		This function is called by the driver within interrupt context.
*
******************************************************************************/
void RecvHandler(void *CallBackRef)
{
	XCanPs *CanPtr = (XCanPs *)CallBackRef;
	int Status;

	Status = XCanPs_Recv(CanPtr, RxFrame);
	if (Status != XST_SUCCESS) {
		xil_printf("Error: Unable to receive frame\r\n");
		LoopbackError = TRUE;
		RecvDone = TRUE;
		return;
	}

	// If the frame identifier correspond to motor 1
	uint8_t byte1, byte2, byte3, byte4;
	if (RxFrame[0] == (u32)XCanPs_CreateIdValue((u32)CAN_MOTOR_1_ID, 0, 0, 0, 0)){
		byte1 = (RxFrame[2] >> 24) & 0xFF;
		byte2 = (RxFrame[2] >> 16) & 0xFF;
		byte3 = (RxFrame[2] >> 8) & 0xFF;
		byte4 = RxFrame[2] & 0xFF;

		angle_motor_1 = (int16_t)((byte4 << 8) | byte3);
		speed_motor_1 = (int16_t)((byte2 << 8) | byte1);

		byte1 = (RxFrame[3] >> 8) & 0xFF;
		byte2 = RxFrame[3] & 0xFF;

		torque_motor_1 = (int16_t)((byte2 << 8) | byte1);

	}else if (RxFrame[0] == (u32)XCanPs_CreateIdValue((u32)CAN_MOTOR_2_ID, 0, 0, 0, 0)){
		byte1 = (RxFrame[2] >> 24) & 0xFF;
		byte2 = (RxFrame[2] >> 16) & 0xFF;
		byte3 = (RxFrame[2] >> 8) & 0xFF;
		byte4 = RxFrame[2] & 0xFF;

		angle_motor_2 = (int16_t)((byte4 << 8) | byte3);
		speed_motor_2 = (int16_t)((byte2 << 8) | byte1);

		byte1 = (RxFrame[3] >> 8) & 0xFF;
		byte2 = RxFrame[3] & 0xFF;

		torque_motor_2 = (int16_t)((byte2 << 8) | byte1);
	}else if(RxFrame[0] == (u32)XCanPs_CreateIdValue((u32)CAN_MOTOR_3_ID, 0, 0, 0, 0)){
		byte1 = (RxFrame[2] >> 24) & 0xFF;
		byte2 = (RxFrame[2] >> 16) & 0xFF;
		byte3 = (RxFrame[2] >> 8) & 0xFF;
		byte4 = RxFrame[2] & 0xFF;

		angle_motor_3 = (int16_t)((byte4 << 8) | byte3);
		speed_motor_3 = (int16_t)((byte2 << 8) | byte1);

		byte1 = (RxFrame[3] >> 8) & 0xFF;
		byte2 = RxFrame[3] & 0xFF;

		torque_motor_3 = (int16_t)((byte2 << 8) | byte1);
	}
	RecvDone = TRUE;
}

/*****************************************************************************/
/**
*
* Callback function (called from interrupt handler) to handle error interrupt.
* Error code read from Error Status register is passed into this function.
*
* @param	CallBackRef is the callback reference passed from the interrupt
*		handler, which in our case is a pointer to the driver instance.
* @param	ErrorMask is a bit mask indicating the cause of the error.
*		Its value equals 'OR'ing one or more XCANPS_ESR_* defined in
*		xcanps_hw.h.
*
* @return	None.
*
* @note		This function is called by the driver within interrupt context.
*
******************************************************************************/
void ErrorHandler(void *CallBackRef, u32 ErrorMask)
{
	XCanPs *CanPtr = (XCanPs *)CallBackRef;

	if (ErrorMask & XCANPS_ESR_ACKER_MASK) {
		/*
		 * ACK Error handling code should be put here.
		 */
		// xil_printf("ACK Error\r\n");
	}

	if (ErrorMask & XCANPS_ESR_BERR_MASK) {
		/*
		 * Bit Error handling code should be put here.
		 */
		// xil_printf("Bit Error\r\n");
	}

	if (ErrorMask & XCANPS_ESR_STER_MASK) {
		/*
		 * Stuff Error handling code should be put here.
		 */
		// xil_printf("Stuff Error\r\n");
	}

	if (ErrorMask & XCANPS_ESR_FMER_MASK) {
		/*
		 * Form Error handling code should be put here.
		 */
		// xil_printf("Form Error\r\n");
	}

	if (ErrorMask & XCANPS_ESR_CRCER_MASK) {
		/*
		 * CRC Error handling code should be put here.
		 */
		// xil_printf("CRC Error\r\n");
	}
	XCanPs_ClearBusErrorStatus(CanPtr, ErrorMask);

	/*
	 * Set the shared variables.
	 */
	LoopbackError = TRUE;
	RecvDone = TRUE;
	SendDone = TRUE;
}

/*****************************************************************************/
/**
*
* Callback function (called from interrupt handler) to handle the following
* interrupts:
*   - XCANPS_IXR_BSOFF_MASK:	Bus Off Interrupt
*   - XCANPS_IXR_RXOFLW_MASK:	RX FIFO Overflow Interrupt
*   - XCANPS_IXR_RXUFLW_MASK:	RX FIFO Underflow Interrupt
*   - XCANPS_IXR_TXBFLL_MASK:	TX High Priority Buffer Full Interrupt
*   - XCANPS_IXR_TXFLL_MASK:	TX FIFO Full Interrupt
*   - XCANPS_IXR_WKUP_MASK:	Wake up Interrupt
*   - XCANPS_IXR_SLP_MASK:	Sleep Interrupt
*   - XCANPS_IXR_ARBLST_MASK:	Arbitration Lost Interrupt
*
*
* @param	CallBackRef is the callback reference passed from the
*		interrupt Handler, which in our case is a pointer to the
*		driver instance.
* @param	IntrMask is a bit mask indicating pending interrupts.
*		Its value equals 'OR'ing one or more of the XCANPS_IXR_*_MASK
*		value(s) mentioned above.
*
* @return	None.
*
* @note		This function is called by the driver within interrupt context.
* 		This function should be changed to meet specific application
*		needs.
*
******************************************************************************/
void EventHandler(void *CallBackRef, u32 IntrMask)
{
	XCanPs *CanPtr = (XCanPs *)CallBackRef;

	if (IntrMask & XCANPS_IXR_BSOFF_MASK) {
		/*
		 * Entering Bus off status interrupt requires
		 * the CAN device be reset and reconfigured.
		 */
		XCanPs_Reset(CanPtr);
		Config(CanPtr);
		return;
	}

	if (IntrMask & XCANPS_IXR_RXOFLW_MASK) {
		/*
		 * Code to handle RX FIFO Overflow Interrupt should be put here.
		 */
		XCanPs_IntrClear(CanPtr, XCANPS_IXR_RXOFLW_MASK);
		xil_printf("RX FIFO Overflow\r\n");
	}

	if (IntrMask & XCANPS_IXR_RXUFLW_MASK) {
		/*
		 * Code to handle RX FIFO Underflow Interrupt
		 * should be put here.
		 */
		xil_printf("RX FIFO Underflow\r\n");
	}

	if (IntrMask & XCANPS_IXR_TXBFLL_MASK) {
		/*
		 * Code to handle TX High Priority Buffer Full
		 * Interrupt should be put here.
		 */
		xil_printf("TX High Priority Buffer Full\r\n");
	}

	if (IntrMask & XCANPS_IXR_TXFLL_MASK) {
		/*
		 * Code to handle TX FIFO Full Interrupt should be put here.
		 */
		xil_printf("TX FIFO Full\r\n");
	}

	if (IntrMask & XCANPS_IXR_WKUP_MASK) {
		/*
		 * Code to handle Wake up from sleep mode Interrupt
		 * should be put here.
		 */
		xil_printf("Wake up from sleep mode\r\n");
	}

	if (IntrMask & XCANPS_IXR_SLP_MASK) {
		/*
		 * Code to handle Enter sleep mode Interrupt should be put here.
		 */
		xil_printf("Enter sleep mode\r\n");
	}

	if (IntrMask & XCANPS_IXR_ARBLST_MASK) {
		/*
		 * Code to handle Lost bus arbitration Interrupt
		 * should be put here.
		 */
		xil_printf("Lost bus arbitration\r\n");
	}
}

uint8_t Motor_cmd(void) {
    u32 id;
    if (Get_Param_u32(&id)){
        return PARAM_ERROR_CODE;
    }
	float current;
    if (Get_Param_Float(&current)){
        return PARAM_ERROR_CODE;
    }
	xil_printf("motor %d, current: %d\n\r", id, current);
	if (id == 1){
		motor1_current_order = (int)(current * 1000);
	}else if (id == 2){
		motor2_current_order = (int)(current * 1000);
	}else if (id == 3){
		motor3_current_order = (int)(current * 1000);
	}
    return 0;
}
