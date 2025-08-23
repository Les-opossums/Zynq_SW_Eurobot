#ifndef UART_H
#define UART_H

#ifndef UART_DEVICE_ID
#define UART_DEVICE_ID		XPAR_XUARTPS_0_DEVICE_ID
#endif

#define UART_BUFFER_SIZE	100

#define BAUDRATE            921600

extern XUartPs UartInstance;

int UART_Init(void);
void Handler(void *CallBackRef, u32 Event, unsigned int EventData);
void Send_Uart_Cmd(uint8_t symbole);
void Send_Uart_Buff_Cmd(uint8_t Buff[], uint8_t Len);
uint8_t Get_Uart_Cmd(uint8_t *c);
u16 Place_In_Uart_Cmd(void);

#endif // UART_H