#include "../main.h"

LED led[NBR_LED];
LED color;

XGpio LED_ADDR;
XGpio LED_DATA;

int LED_old_timer_ms1 = 0;

void ws2812b_init(){
   XGpio_Initialize(&LED_ADDR, XPAR_AXI_GPIO_0_DEVICE_ID);
   XGpio_Initialize(&LED_DATA, XPAR_AXI_GPIO_1_DEVICE_ID);

   XGpio_SetDataDirection(&LED_ADDR, 1, 0);
   XGpio_SetDataDirection(&LED_DATA, 1, 0);

   for (int i = 0; i < NBR_LED; i++){
       led[i].red = 0;
       led[i].green = 0;
       led[i].blue = 0;
   }
}

void ws2812b_set_color(int led_id, LED color){

   if (led_id < 0 || led_id > NBR_LED){
       xil_printf("LED id out of range\n\r");
   }else{
       led[led_id] = color;
   }
}

uint32_t led_color = 0x0000FF;
uint32_t led_id = 0;
uint32_t led_color1 = 0x0100000;   //BRG
uint32_t led_color2 = 0xFF0000;   //BRG
uint8_t  cpt = 0;
uint8_t led_animation_mode = 40;
uint8_t led_mode_state = 0;
uint32_t led_mode_timer_old = 0;

void LED_loop(){
   if (Timer_ms1 - LED_old_timer_ms1 > 10){
       LED_old_timer_ms1 = Timer_ms1;
       for (int i = 0; i < NBR_LED-1; i++){
           XGpio_DiscreteWrite(&LED_ADDR, 1, i);
           XGpio_DiscreteWrite(&LED_DATA, 1, (led[i].blue << 16) | (led[i].red << 8) | led[i].green);
       }
   }
   LED_MODE();
}

uint8_t sens_animation_au = 0;

uint8_t chargement_blue = 0;
uint8_t chargement_jaune = 0;

int validation_state = 0;
int cpt2 =  NBR_LED/2;

void LED_MODE(){
   switch(led_animation_mode){
       case 0: // default color
           for (int i = 0; i < NBR_LED-1; i++){
               led[i].red = 0;
               led[i].green = 0;
               led[i].blue = 0;
           }
           break;
    //    case 1: //waiting for color choice
    //        if(Timer_ms1 - led_mode_timer_old > 20){
    //            led_mode_timer_old = Timer_ms1;
    //            for (int i = 0; i < NBR_LED-1; i++){
    //                if (sens_animation_au == 0){
    //                    if(i < cpt){ // yellow
    //                        led[i].red = 0x10;
    //                        led[i].green = 0x10;
    //                        led[i].blue = 0;
    //                    }else{
    //                        led[i].red = 0;
    //                        led[i].green = 0;
    //                        led[i].blue = 0x10;
    //                    }
    //                }else{
    //                    if(i < cpt){
    //                        led[i].red = 0;
    //                        led[i].green = 0;
    //                        led[i].blue = 0x10;
    //                    }else{
    //                        led[i].red = 0x10;
    //                        led[i].green = 0x10;
    //                        led[i].blue = 0;
    //                    }
    //                }
    //            }
    //            cpt++;
    //            if(cpt > NBR_LED-1){
    //                cpt = 0;
    //                sens_animation_au = !sens_animation_au;
    //            }
    //        }
    //        break;
    //    case 10: // validation with green color
    //        if(Timer_ms1 - led_mode_timer_old > 20){
    //            led_mode_timer_old = Timer_ms1;
    //            for (int i = 0; i < NBR_LED-1; i++){
    //                if(i < cpt){
    //                    led[i].red = 0;
    //                    led[i].green = 0x10;
    //                    led[i].blue = 0;
    //                }else{
    //                    led[i].red = 0;
    //                    led[i].green = 0;
    //                    led[i].blue = 0;
    //                }
    //            }
    //            if (cpt < NBR_LED){
    //                cpt++;
    //            }else{
    //                validation_state = 1;
    //            }
    //        }
    //        break;
    //    case 20: // team color yellow
    //        if (chargement_jaune == 0){
    //            if(Timer_ms1 - led_mode_timer_old > 20){
    //                led_mode_timer_old = Timer_ms1;
    //                for (int i = 0; i < NBR_LED-1; i++){
    //                    if(i < cpt){
    //                        led[i].red = 0x10;
    //                        led[i].green = 0x10;
    //                        led[i].blue = 0;
    //                    }else{
    //                        led[i].red = 0;
    //                        led[i].green = 0;
    //                        led[i].blue = 0;
    //                    }
    //                }
    //                cpt++;
    //                if(cpt > NBR_LED){
    //                    cpt = 0;
    //                    chargement_jaune = 1;
    //                }
    //            }
    //        }else{
    //            if(Timer_ms1 - led_mode_timer_old > 20){
    //                led_mode_timer_old = Timer_ms1;
    //                for (int i = 0; i < NBR_LED-1; i++){
    //                    if(i == cpt){
    //                        led[i].red = 0xFF;
    //                        led[i].green = 0xFF;
    //                        led[i].blue = 0;
    //                    }else{
    //                        led[i].red = 0x10;
    //                        led[i].green = 0x10;
    //                        led[i].blue = 0;
    //                    }

    //                }
    //                cpt++;
    //                if(cpt > NBR_LED){
    //                    cpt = 0;
    //                }
    //            }
    //        }
    //        break;
    //    case 30: // team color blue
    //        if (chargement_blue == 0){
    //            if(Timer_ms1 - led_mode_timer_old > 20){
    //                led_mode_timer_old = Timer_ms1;
    //                for (int i = 0; i < NBR_LED-1; i++){
    //                    if(i < cpt){
    //                        led[i].red = 0;
    //                        led[i].green = 0;
    //                        led[i].blue = 0x10;
    //                    }else{
    //                        led[i].red = 0;
    //                        led[i].green = 0;
    //                        led[i].blue = 0;
    //                    }
    //                }
    //                cpt++;
    //                if(cpt > NBR_LED){
    //                    cpt = 0;
    //                    chargement_blue = 1;
    //                }
    //            }
    //        }else{
    //            if(Timer_ms1 - led_mode_timer_old > 20){
    //                led_mode_timer_old = Timer_ms1;
    //                for (int i = 0; i < NBR_LED-1; i++){
    //                    if(i == cpt){
    //                        led[i].red = 0;
    //                        led[i].green = 0;
    //                        led[i].blue = 0xFF;
    //                    }else{
    //                        led[i].red = 0;
    //                        led[i].green = 0;
    //                        led[i].blue = 0x10;
    //                    }

    //                }
    //                cpt++;
    //                if(cpt > NBR_LED){
    //                    cpt = 0;
    //                }
    //            }
    //        }
    //        break;
       case 40: // case of AU
           if(Timer_ms1 - led_mode_timer_old > 10){
               led_mode_timer_old = Timer_ms1;
               for (int i = 0; i < NBR_LED-1; i++){
                   if (sens_animation_au == 0){
                       if(i < cpt){
                           led[i].red = 0xFF;
                           led[i].green = 0;
                           led[i].blue = 0;
                       }else{
                           led[i].red = 0;
                           led[i].green = 0;
                           led[i].blue = 0;
                       }
                   }else{
                       if(i < cpt){
                           led[i].red = 0;
                           led[i].green = 0;
                           led[i].blue = 0;
                       }else{
                           led[i].red = 0xFF;
                           led[i].green = 0;
                           led[i].blue = 0;
                       }
                   }
               }
               cpt++;
               if(cpt > NBR_LED){
                   cpt = 0;
                   sens_animation_au = !sens_animation_au;
               }
           }
           break;
       case 60: // leash
           if(Timer_ms1 - led_mode_timer_old > 20){
               led_mode_timer_old = Timer_ms1;
               cpt = (timer_match*44)/100000;
               if (cpt < NBR_LED-1){
                   for (int i = 0; i < NBR_LED-1; i++){
                       if(i < cpt){
                           led[i].red = 0x10;
                           led[i].green = 0x10;
                           led[i].blue = 0x10;
                       }else{
                           led[i].red = 0;
                           led[i].green = 0;
                           led[i].blue = 0;
                       }
                   }
               }
           }
           break;

   }
}


void LED_AU(void){
   led_animation_mode = 40;
}

void LED_CLASSIC_MODE(void){
   led_animation_mode = current_mode;
}

uint8_t LED_cmd(void) {
   u32 id;
   if (Get_Param_u32(&id)){
       return PARAM_ERROR_CODE;
   }
	u32 red;
   if (Get_Param_u32(&red)){
       return PARAM_ERROR_CODE;
   }
   u32 green;
   if (Get_Param_u32(&green)){
       return PARAM_ERROR_CODE;
   }
   u32 blue;
   if (Get_Param_u32(&blue)){
       return PARAM_ERROR_CODE;
   }
   if (red < 0 || red > 255 || green < 0 || green > 255 || blue < 0 || blue > 255){
       xil_printf("Color out of range\n\r");
   }else {
       xil_printf("LED number %d, color: %d, %d, %d\n\r", id, red, green, blue);
       led_color = (green << 16) | (red << 8) | blue;
   }
   return 0;
}

