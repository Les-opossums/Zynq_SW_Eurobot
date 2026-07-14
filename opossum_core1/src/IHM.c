#include "main.h"

XGpio YELLOW_SWITCH;
XGpio GREEN_SWITCH;
XGpio BLUE_SWITCH;
XGpio LEASH;

int yellow_state;
int green_state; 
int blue_state;
int leash_state;

int previous_yellow_state;
int previous_green_state;
int previous_blue_state;
int previous_leash_state;

int timer_match = 0;
int start_timer_match = 0;


int current_mode = 0;

void init_switch(void){
//    // init IHM switches
//    XGpio_Initialize(&YELLOW_SWITCH, XPAR_AXI_GPIO_23_DEVICE_ID);
//    XGpio_Initialize(&GREEN_SWITCH, XPAR_AXI_GPIO_21_DEVICE_ID);
//    XGpio_Initialize(&BLUE_SWITCH, XPAR_AXI_GPIO_22_DEVICE_ID);
//
//    //init leash input
   XGpio_Initialize(&LEASH, XPAR_AXI_GPIO_20_DEVICE_ID);
//
//    // set switches as input
//    XGpio_SetDataDirection(&YELLOW_SWITCH, 1, 1);
//    XGpio_SetDataDirection(&GREEN_SWITCH, 1, 1);
//    XGpio_SetDataDirection(&BLUE_SWITCH, 1, 1);
//
//    // set leash as input
   XGpio_SetDataDirection(&LEASH, 1, 1);
}

int validation_blue = 0;
int validation_yellow = 0;

void IHM_loop(void){
//    yellow_state = XGpio_DiscreteRead(&YELLOW_SWITCH, 1);
//    green_state = XGpio_DiscreteRead(&GREEN_SWITCH, 1);
//    blue_state = XGpio_DiscreteRead(&BLUE_SWITCH, 1);
//
//    if (validation_blue == 1 && validation_state ==1){
//        current_mode = 20;
//        validation_blue = 0;
//        validation_state = 0;
//    }
//
//    if (validation_yellow == 1 && validation_state ==1){
//        current_mode = 30;
//        validation_yellow = 0;
//        validation_state = 0;
//    }
//
//
//
//    if (yellow_state != previous_yellow_state){
//        previous_yellow_state = yellow_state;
//        if(yellow_state == 0){
//            printf("YELLOWSWITCH\n");
//            if(AU_state == 0){
//
//                if(current_mode == 1){ // start selection color
//                    current_mode = 10;
//                    validation_blue = 1;
//                } else if (current_mode == 20){ //abort selection color
//                    current_mode = 1;
//                }
//            }
//        }
//    }
//    if (green_state != previous_green_state){
//        previous_green_state = green_state;
//        if(green_state == 0){
//            printf("GREENSWITCH\n");
//            if(AU_state == 0){
//
//                if(current_mode == 0){ // start selection color
//                    current_mode = 1;
//                } else if (current_mode == 1){ //abort selection color
//                    current_mode = 0;
//                }
//            }
//        }
//    }
//    if (blue_state != previous_blue_state){
//        previous_blue_state = blue_state;
//        if(blue_state == 0){
//            printf("BLUESWITCH\n");
//            if(AU_state == 0){
//                if(current_mode == 1){ // start selection color
//                    current_mode = 10;
//                    validation_yellow = 1;
//                }else if(current_mode == 30){ //abort selection color
//                    current_mode = 1;
//                }
//            }
//        }
//    }
//
   leash_state = XGpio_DiscreteRead(&LEASH, 1);
   if (leash_state != previous_leash_state){
       previous_leash_state = leash_state;
       if(leash_state == 1){
           printf("LEASH\n");
           eth_send_frame(ETH_MSG_LEASH, &leash_state, sizeof(leash_state));
           current_mode = 60;
           start_timer_match = Timer_ms1;
       }else{
           current_mode = 0;
       }
   }
   if(leash_state == 1){
       timer_match = Timer_ms1 - start_timer_match;
   }
}

uint8_t Version_cmd(void) {
    printf("VERSION ZYNQ\n");
    return 0;
}
