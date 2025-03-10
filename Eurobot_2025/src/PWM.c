#include "main.h"

Servo servo[NBR_SERVO];

int old_Timer_ms1 = 0;

void Init_Servo(Servo *servo, int axi_id, int default_pos, int min_pos, int max_pos, int to_do){
    servo->axi_id       = axi_id;
    servo->default_pos  = default_pos;
    servo->min_pos      = min_pos;
    servo->max_pos      = max_pos;
    servo->pos          = default_pos;
    servo->current_pos  = default_pos;
    servo->to_do        = to_do;
}

void PWM_Init(void)
{
    int gpio_id = XPAR_AXI_GPIO_2_DEVICE_ID; //change XPAR_AXI_GPIO_2_DEVICE_ID with the correct ID when all pwm will be created
    for (int i = 0; i < NBR_SERVO; i++){
        switch (i){
            case 0:
                gpio_id = XPAR_AXI_GPIO_2_DEVICE_ID;
                break;
            case 1:
                gpio_id = XPAR_AXI_GPIO_3_DEVICE_ID;
                break;
            case 2:
                gpio_id = XPAR_AXI_GPIO_4_DEVICE_ID;
                break;
            case 3:
                gpio_id = XPAR_AXI_GPIO_5_DEVICE_ID;
                break;
            case 4:
                gpio_id = XPAR_AXI_GPIO_6_DEVICE_ID;
                break;
            case 5:
                gpio_id = XPAR_AXI_GPIO_7_DEVICE_ID;
                break;
            case 6:
                gpio_id = XPAR_AXI_GPIO_8_DEVICE_ID;
                break;
            case 7:
                gpio_id = XPAR_AXI_GPIO_9_DEVICE_ID;
                break;
            case 8:
                gpio_id = XPAR_AXI_GPIO_10_DEVICE_ID;
                break;
            // case 9:
            //     gpio_id = XPAR_AXI_GPIO_11_DEVICE_ID;
            //     break;
            // case 10:
            //     gpio_id = XPAR_AXI_GPIO_12_DEVICE_ID;
            //     break;
            // case 11:
            //     gpio_id = XPAR_AXI_GPIO_13_DEVICE_ID;
            //     break;
            // case 12:
            //     gpio_id = XPAR_AXI_GPIO_14_DEVICE_ID;
            //     break;
            // case 13:
            //     gpio_id = XPAR_AXI_GPIO_15_DEVICE_ID;
            //     break;
            // case 14:
            //     gpio_id = XPAR_AXI_GPIO_16_DEVICE_ID;
            //     break;
            // case 15:
            //     gpio_id = XPAR_AXI_GPIO_17_DEVICE_ID;
            //     break;
        }
        Init_Servo(&servo[i], gpio_id, DEFAULT_ANGLE, DEFAULT_ANGLE_MIN, DEFAULT_ANGLE_MAX, 0); 
        XGpio_Initialize(&servo[i].gpio, servo[i].axi_id);
	    XGpio_SetDataDirection(&servo[i].gpio, 1, 0);
    }
}


int pwm_loop_state = 0;
int increment = 10;


void PWM_Loop(void){
    switch (pwm_loop_state){
        case 0:
            if (Timer_ms1 - old_Timer_ms1 > 10){
                old_Timer_ms1 = Timer_ms1;
                pwm_loop_state = 1;
            }
            break;
        case 1:
            for (int i = 0; i < NBR_SERVO; i++){
                if(servo[i].to_do == 1){
                    XGpio_DiscreteWrite(&servo[i].gpio, 1, servo[i].pos);
                    servo[i].to_do = 0;    
                }
            }
            pwm_loop_state = 0;
            break;
    }
}

void write_servo(int id, int angle){
    if (angle < 0 || angle > 180 || id < 1 || id > NBR_SERVO){
        xil_printf("Command invalid \n\r");
        return;
    }
    servo[id-1].pos = angle;
    servo[id-1].to_do = 1;
}

uint8_t Servo_cmd(void) {
    u32 id;
    if (Get_Param_u32(&id)){
        return PARAM_ERROR_CODE;
    }
	u32 angle;
    if (Get_Param_u32(&angle)){
        return PARAM_ERROR_CODE;
    }
	if (angle < 0 || angle > 180){
        xil_printf("Angle out of range\n\r");
	}else {
        xil_printf("Servo number %d, angle: %d\n\r", id, angle);
        servo[id-1].pos = angle;
        servo[id-1].to_do = 1;
	}
    return 0;
}
