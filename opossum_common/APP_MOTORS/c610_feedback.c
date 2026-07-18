#include "c610_feedback.h"

C610_MotorFeedback_t motor_feedback[4];

static void decode_c610(const u32 *frame_words, int *angle, int *speed, int *torque) {
    u8 b1 = (frame_words[2] >> 24) & 0xFF;
    u8 b2 = (frame_words[2] >> 16) & 0xFF;
    u8 b3 = (frame_words[2] >> 8) & 0xFF;
    u8 b4 = frame_words[2] & 0xFF;
    *angle = (int16_t)((b4 << 8) | b3);
    *speed = (int16_t)((b2 << 8) | b1);

    b1 = (frame_words[3] >> 8) & 0xFF;
    b2 = frame_words[3] & 0xFF;
    *torque = (int16_t)((b2 << 8) | b1);
}

void C610_Motor1_Callback(void *app_ctx, const u32 *frame_words, u8 dlc) {
    (void)app_ctx; (void)dlc;
    decode_c610(frame_words, &motor_feedback[0].angle_motor, &motor_feedback[0].speed_motor, &motor_feedback[0].torque_motor);
}
void C610_Motor2_Callback(void *app_ctx, const u32 *frame_words, u8 dlc) {
    (void)app_ctx; (void)dlc;
    decode_c610(frame_words, &motor_feedback[1].angle_motor, &motor_feedback[1].speed_motor, &motor_feedback[1].torque_motor);
}
void C610_Motor3_Callback(void *app_ctx, const u32 *frame_words, u8 dlc) {
    (void)app_ctx; (void)dlc;
    decode_c610(frame_words, &motor_feedback[2].angle_motor, &motor_feedback[2].speed_motor, &motor_feedback[2].torque_motor);
}
void C610_Motor4_Callback(void *app_ctx, const u32 *frame_words, u8 dlc) {
    (void)app_ctx; (void)dlc;
    decode_c610(frame_words, &motor_feedback[3].angle_motor, &motor_feedback[3].speed_motor, &motor_feedback[3].torque_motor);
}

void Init_CAN_MOTOR_variables(void) {
    for (int i = 0; i < 4; i++) {
        motor_feedback[i].angle_motor = 0;
        motor_feedback[i].speed_motor = 0;
        motor_feedback[i].torque_motor = 0;
    }
}

void CAN_transmit_motor(can_io_context_t *ctx, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {
    u8 payload[8];
    payload[0] = (u8)((motor1 >> 8) & 0xFF);
    payload[1] = (u8)(motor1 & 0xFF);
    payload[2] = (u8)((motor2 >> 8) & 0xFF);
    payload[3] = (u8)(motor2 & 0xFF);
    payload[4] = (u8)((motor3 >> 8) & 0xFF);
    payload[5] = (u8)(motor3 & 0xFF);
    payload[6] = (u8)((motor4 >> 8) & 0xFF);
    payload[7] = (u8)(motor4 & 0xFF);

    CAN_IO_Send(ctx, ESC_TX_MESSAGE_ID, payload, CAN_IO_FRAME_DATA_LENGTH);
}