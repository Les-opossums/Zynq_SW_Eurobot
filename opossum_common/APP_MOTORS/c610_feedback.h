#ifndef C610_FEEDBACK_H
#define C610_FEEDBACK_H

#include "../IO_MANAGER/DRIVER_CAN/driver_can_io.h"

#define CAN_MOTOR_1_ID     0x201
#define CAN_MOTOR_2_ID     0x202
#define CAN_MOTOR_3_ID     0x203
#define CAN_MOTOR_4_ID     0x204
#define ESC_TX_MESSAGE_ID  0x200

extern int angle_motor_1, angle_motor_2, angle_motor_3, angle_motor_4;
extern int speed_motor_1, speed_motor_2, speed_motor_3, speed_motor_4;
extern int torque_motor_1, torque_motor_2, torque_motor_3, torque_motor_4;

void C610_Motor1_Callback(void *app_ctx, const u32 *frame_words, u8 dlc);
void C610_Motor2_Callback(void *app_ctx, const u32 *frame_words, u8 dlc);
void C610_Motor3_Callback(void *app_ctx, const u32 *frame_words, u8 dlc);
void C610_Motor4_Callback(void *app_ctx, const u32 *frame_words, u8 dlc);

void Init_CAN_MOTOR_variables(void);

/**
 * @brief Envoie les consignes de courant aux 4 ESC C610 (trame ID 0x200,
 * 2 octets big-endian par moteur, ordre motor1..motor4).
 */
void CAN_transmit_motor(can_io_context_t *ctx, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

#endif /* C610_FEEDBACK_H */