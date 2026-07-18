#ifndef C610_FEEDBACK_H
#define C610_FEEDBACK_H

#include "../IO_MANAGER/DRIVER_CAN/driver_can_io.h"

#define CAN_MOTOR_1_ID     0x201
#define CAN_MOTOR_2_ID     0x202
#define CAN_MOTOR_3_ID     0x203
#define CAN_MOTOR_4_ID     0x204
#define ESC_TX_MESSAGE_ID  0x200

typedef struct {
    int angle_motor;
    int speed_motor;
    int torque_motor;
} C610_MotorFeedback_t;

extern C610_MotorFeedback_t motor_feedback[4];

void C610_Motor1_Callback(void *app_ctx, const u32 *frame_words, u8 dlc);
void C610_Motor2_Callback(void *app_ctx, const u32 *frame_words, u8 dlc);
void C610_Motor3_Callback(void *app_ctx, const u32 *frame_words, u8 dlc);
void C610_Motor4_Callback(void *app_ctx, const u32 *frame_words, u8 dlc);

void Init_CAN_MOTOR_variables(void);

/**
 * @brief Envoie les consignes de courant aux 4 ESC C610 (trame ID 0x200,
 * 2 octets big-endian par moteur, ordre motor1..motor4).
 */
void CAN_transmit_motor(can_io_context_t *ctx, const int16_t *motors, uint16_t nb_motors);

#endif /* C610_FEEDBACK_H */