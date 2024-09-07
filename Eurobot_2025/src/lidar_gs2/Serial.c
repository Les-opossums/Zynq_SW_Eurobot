#include "../main.h"


XUartPs Uart1_Instance;

u8 SendBuffer[UART1_BUFFER_SIZE];
u16 i_TX_CMD_Buff_TODO = 0;
u16 i_TX_CMD_Buff_DONE = 0;

u8 RecvBuffer[UART1_BUFFER_SIZE];
u16 i_RX_CMD_Buff_TODO = 0;
u16 i_RX_CMD_Buff_DONE = 0;

/*
 * The following counters are used to determine when the entire buffer has
 * been sent and received.
 */
volatile int TotalReceivedCount;
volatile int TotalSentCount;
int TotalErrorCount;

int UART1_Init(void) {
    XUartPs_Config *ConfigPtr;
    XUartPs *UartInstPtr = &Uart1_Instance;

    int Status;
    int Index;

	/*
     * Initialize the send buffer bytes and
     * the receive buffer.
     */
    for (Index = 0; Index < UART1_BUFFER_SIZE; Index++) {
        SendBuffer[Index] = 0;
        RecvBuffer[Index] = 0;
    }

	// Initialize the UART driver
    ConfigPtr = XUartPs_LookupConfig(UART1_DEVICE_ID);
    if (ConfigPtr == NULL) {
		xil_printf("Unable to find UART device\r\n");
        return XST_FAILURE;
    }else{
		xil_printf("UART device found\r\n");
	}

    XUartPs_CfgInitialize(UartInstPtr, ConfigPtr, ConfigPtr->BaseAddress);
	Status = XUartPs_SelfTest(UartInstPtr);
    if (Status != XST_SUCCESS) {
		xil_printf("Error: Unable to initialize UART device\r\n");
        return XST_FAILURE;
    }else{
		xil_printf("UART device initialized\r\n");
	}

    XUartPs_SetBaudRate(UartInstPtr, BAUDRATE_UART1);
    
	XUartPs_SetRecvTimeout(UartInstPtr, 8);
	XUartPs_SetHandler(UartInstPtr, (XUartPs_Handler)UART1_Handler, UartInstPtr); 

    /*
	 * Enable the interrupt of the UART so interrupts will occur, setup
	 * a local loopback so data that is sent will be received.
	 */
    XUartPs_SetInterruptMask(UartInstPtr, XUARTPS_IXR_TOUT | XUARTPS_IXR_PARITY | XUARTPS_IXR_FRAMING |
										XUARTPS_IXR_OVER | XUARTPS_IXR_TXEMPTY | XUARTPS_IXR_RXFULL |
										XUARTPS_IXR_RXOVR);


	/* Set the UART in Normal Mode */
	XUartPs_SetOperMode(UartInstPtr, XUARTPS_OPER_MODE_NORMAL);

	return XST_SUCCESS;

}


/**************************************************************************/
/**
*
* This function is the handler which performs processing to handle data events
* from the device.  It is called from an interrupt context. so the amount of
* processing should be minimal.
*
* This handler provides an example of how to handle data for the device and
* is application specific.
*
* @param	CallBackRef contains a callback reference from the driver,
*		in this case it is the instance pointer for the XUartPs driver.
* @param	Event contains the specific kind of event that has occurred.
* @param	EventData contains the number of bytes sent or received for sent
*		and receive events.
*
* @return	None.
*
* @note		None.
*
***************************************************************************/
void UART1_Handler(void *CallBackRef, u32 Event, unsigned int EventData)
{
	XUartPs *Uart1_InstancePtr = (XUartPs *)CallBackRef;
	/* All of the data has been sent */
	if (Event == XUARTPS_EVENT_SENT_DATA) {
		// xil_printf("Data sent\r\n");
	}

	/* All of the data has been received */
	if (Event == XUARTPS_EVENT_RECV_DATA) {
		// xil_printf("Data received\r\n");
		u16 i = i_RX_CMD_Buff_TODO;
		while (XUartPs_IsReceiveData(Uart1_InstancePtr->Config.BaseAddress)) {
            RecvBuffer[i] = XUartPs_ReadReg(Uart1_InstancePtr->Config.BaseAddress, XUARTPS_FIFO_OFFSET);
            i++;
            if (i == UART_BUFFER_SIZE) {  // Check buffer overflow
                i = 0;
            }
        }
		i_RX_CMD_Buff_TODO = i;
	}

	/*
	 * Data was received, but not the expected number of bytes, a
	 * timeout just indicates the data stopped for 8 character times
	 */
	if (Event == XUARTPS_EVENT_RECV_TOUT) {
		TotalReceivedCount = EventData;
	}

	/*
	 * Data was received with an error, keep the data but determine
	 * what kind of errors occurred
	 */
	if (Event == XUARTPS_EVENT_RECV_ERROR) {
		TotalReceivedCount = EventData;
		TotalErrorCount++;
	}

	/*
	 * Data was received with an parity or frame or break error, keep the data
	 * but determine what kind of errors occurred. Specific to Zynq Ultrascale+
	 * MP.
	 */
	if (Event == XUARTPS_EVENT_PARE_FRAME_BRKE) {
		TotalReceivedCount = EventData;
		TotalErrorCount++;
	}

	/*
	 * Data was received with an overrun error, keep the data but determine
	 * what kind of errors occurred. Specific to Zynq Ultrascale+ MP.
	 */
	if (Event == XUARTPS_EVENT_RECV_ORERR) {
		TotalReceivedCount = EventData;
		TotalErrorCount++;
	}
}

u16 Place_In_Uart1_Cmd(void) {
	u16 In_Buff = (UART1_BUFFER_SIZE + i_TX_CMD_Buff_TODO - i_TX_CMD_Buff_DONE) % UART1_BUFFER_SIZE;
	return (UART1_BUFFER_SIZE - 1 - In_Buff);
}

void Send_Uart1_Cmd(uint8_t symbole) {
	XUartPs_Send(&Uart1_Instance, &symbole, 1);
}

void Send_Uart1_Buff_Cmd(uint8_t Buff[], uint8_t Len) {
	XUartPs_Send(&Uart1_Instance, Buff, Len);
}

uint8_t Get_Uart1_Cmd(uint8_t *c) {
	if (i_RX_CMD_Buff_DONE != i_RX_CMD_Buff_TODO) { 
		*c = RecvBuffer[i_RX_CMD_Buff_DONE];
		i_RX_CMD_Buff_DONE++;
		if (i_RX_CMD_Buff_DONE >= UART1_BUFFER_SIZE)
			i_RX_CMD_Buff_DONE = 0;
		return 1;
	} else {
		return 0;
	}
}