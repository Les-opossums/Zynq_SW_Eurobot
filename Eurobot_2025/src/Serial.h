#ifndef UART1_DEVICE_ID
#define UART1_DEVICE_ID		XPAR_XUARTPS_1_DEVICE_ID
#endif

#define UART1_BUFFER_SIZE	100

#define BAUDRATE_UART1      921600

extern XUartPs Uart1_Instance;

int UART1_Init(void);
void UART1_Handler(void *CallBackRef, u32 Event, unsigned int EventData);
void Send_Uart1_Cmd(uint8_t symbole);
void Send_Uart1_Buff_Cmd(uint8_t Buff[], uint8_t Len);
uint8_t Get_Uart1_Cmd(uint8_t *c);
u16 Place_In_Uart1_Cmd(void);
