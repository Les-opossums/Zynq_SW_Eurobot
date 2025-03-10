#include "main.h"

int AU_state;

XGpio AU;

void init_AU(void){
	// Init QEI
    XGpio_Initialize(&AU, XPAR_AXI_GPIO_18_DEVICE_ID);

    XGpio_SetDataDirection(&AU, 1, 1);
}

void AU_Loop(void){
	AU_state = XGpio_DiscreteRead(&AU, 1);
    if (AU_state == 0){
        xil_printf("AU 0");
    }
}
