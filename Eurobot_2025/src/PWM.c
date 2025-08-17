#include "main.h"

// Servo servo[NBR_SERVO];

// int old_Timer_ms1 = 0;

// void Init_Servo(Servo *servo, int axi_addr, int default_pos, int min_pos, int max_pos, int step, int to_do)
// {
//     servo->axi_addr = axi_addr;
//     servo->default_pos = default_pos;
//     servo->current_pos = default_pos;
//     servo->min_pos = min_pos;
//     servo->max_pos = max_pos;
//     servo->pos = default_pos;
//     servo->step = step;
//     servo->to_do = to_do;
// }


// void PWM_Init(void)
// {
//     int angle = DEFAULT_ANGLE;
//     int angle_min = DEFAULT_ANGLE_MIN;
//     int angle_max = DEFAULT_ANGLE_MAX;
//     int step = DEFAULT_ANGLE_STEP;
    
//     for (int i = 0; i < 8; i++){
//         switch (i){
//             // case 0:
//             //     gpio_id = XPAR_AXI_GPIO_2_DEVICE_ID;
//             //     angle = SERVO_1_ANGLE_MAX;
//             //     angle_min = SERVO_1_ANGLE_MIN;
//             //     angle_max = SERVO_1_ANGLE_MAX;
//             //     break;
//             // case 1:
//             //     gpio_id = XPAR_AXI_GPIO_3_DEVICE_ID;
//             //     angle = SERVO_2_ANGLE_MIN;
//             //     angle_min = SERVO_2_ANGLE_MIN;
//             //     angle_max = SERVO_2_ANGLE_MAX;
//             //     break;
//             // case 2:
//             //     gpio_id = XPAR_AXI_GPIO_4_DEVICE_ID;
//             //     break;
//             // case 3:
//             //     gpio_id = XPAR_AXI_GPIO_5_DEVICE_ID;
//             //     break;
//             // case 4:
//             //     gpio_id = XPAR_AXI_GPIO_6_DEVICE_ID;
//             //     break;
//             // case 5:
//             //     gpio_id = XPAR_AXI_GPIO_7_DEVICE_ID;
//             //     break;
//             // case 6:
//             //     gpio_id = XPAR_AXI_GPIO_8_DEVICE_ID;
//             //     break;
//             // case 7:
//             //     gpio_id = XPAR_AXI_GPIO_9_DEVICE_ID;
//             //     break;
//             // case 8:
//             //     gpio_id = XPAR_AXI_GPIO_10_DEVICE_ID;
//             //     break;
//         }

//         Init_Servo(&servo[i], AXI_PWM_BASEADDR + (i * 4), angle, angle_min, angle_max, step, 0);
//     }
// }


// int pwm_loop_state = 0;
// int increment = DEFAULT_ANGLE_STEP;


// void PWM_Loop(void){
//     switch (pwm_loop_state){
//         case 0:
//             if (Timer_ms1 - old_Timer_ms1 > 20){
//                 old_Timer_ms1 = Timer_ms1;
//                 pwm_loop_state = 1;
//             }
//             break;
//         case 1:
//             for (int i = 0; i < NBR_SERVO; i++){
//                 if(servo[i].to_do == 1){
//                     if(servo[i].pos > servo[i].current_pos){
//                         if (servo[i].current_pos + increment < servo[i].pos){
//                             servo[i].current_pos += increment;
//                         }else{
//                             servo[i].current_pos = servo[i].pos;
//                         }
//                     }else if(servo[i].pos < servo[i].current_pos){
//                         if (servo[i].current_pos - increment > servo[i].pos){
//                             servo[i].current_pos -= increment;
//                         }else{
//                             servo[i].current_pos = servo[i].pos;
//                         }
//                     }
//                     *((volatile uint32_t*)servo[i].axi_addr) = (uint32_t)servo[i].current_pos;
//                     if (servo[i].current_pos == servo[i].pos){
//                         servo[i].to_do = 0;
//                     }  
//                 }
//             }
//             pwm_loop_state = 0;
//             break;
//     }
// }

// void write_servo(int id, int angle){
//     if (angle < 0 || angle > 180 || id < 1 || id > NBR_SERVO){
//         xil_printf("Command invalid \n\r");
//         return;
//     }
//     servo[id-1].pos = angle;
//     servo[id-1].to_do = 1;
// }

// uint8_t Servo_cmd(void) {
//     u32 id;
//     if (Get_Param_u32(&id)){
//         return PARAM_ERROR_CODE;
//     }
// 	u32 angle;
//     if (Get_Param_u32(&angle)){
//         return PARAM_ERROR_CODE;
//     }
// 	if (angle < servo[id - 1].min_pos || angle > servo[id - 1].max_pos){
//         xil_printf("Angle out of range\n\r");
// 	}else {
//         xil_printf("Servo number %d, angle: %d\n\r", id, angle);
//         servo[id-1].pos = angle;
//         servo[id-1].to_do = 1;
// 	}
//     return 0;
// }
