#include "main.h"

XGpio Pump;
uint8_t pump_state[6];
uint8_t pump_cmd = 0;

void Init_Pump(void){
    XGpio_Initialize(&Pump, XPAR_AXI_GPIO_10_DEVICE_ID);
    XGpio_SetDataDirection(&Pump, 1, 0);
    for (int i = 0; i < 6; i++){
        pump_state[i] = 0;
    }
}

uint32_t old_pump_Timer_ms1 = 0;

void Pump_Loop(void){
    if(Timer_ms1 - old_pump_Timer_ms1 > 10){
        old_pump_Timer_ms1 = Timer_ms1;
        pump_cmd = pump_state[0] | (pump_state[1] << 1) | (pump_state[2] << 2) | (pump_state[3] << 3) | (pump_state[4] << 4) | (pump_state[5] << 5);
        XGpio_DiscreteWrite(&Pump, 1, pump_cmd);
    }
}

uint8_t Pump_cmd(void){
    u32 id;
    if (Get_Param_u32(&id)){
        return PARAM_ERROR_CODE;
    }
	u32 state;
    if (Get_Param_u32(&state)){
        return PARAM_ERROR_CODE;
    }
	
    xil_printf("Pump number %d, state: %d\n\r", id, state);
    if (state == 0){
        pump_state[id-1] = 1;
    }else{
        pump_state[id-1] = 0;
    }
    return 0;
}