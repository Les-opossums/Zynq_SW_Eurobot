#include "main.h"

int AU_state = 0;
int previous_AU_state = 0;

XGpio AU;

void init_AU(void){
	// Init QEI
    XGpio_Initialize(&AU, XPAR_AXI_GPIO_18_DEVICE_ID);

    XGpio_SetDataDirection(&AU, 1, 1);
}


void AU_Loop(void){
	AU_state = XGpio_DiscreteRead(&AU, 1);
    if (AU_state != previous_AU_state){
        xil_printf("AU %d\n\r", AU_state);
        eth_send_frame(ETH_MSG_AU, &AU_state, sizeof(AU_state));
        previous_AU_state = AU_state;
    }
}
