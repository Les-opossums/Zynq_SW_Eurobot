#include "main.h"

uint8_t lidar_init = 0;


uint8_t Lidar_Frame_Buf[LIDAR_BUFF_SIZE];
uint16_t i_Lidar_In_Buff_TODO = 0;
uint16_t i_Lidar_In_Buff_DONE = 0;

void create_frame(uint8_t *frame, uint8_t device_id, uint8_t cmd) {
    frame[0] = (PACKET_HEADER >> 24) & 0xFF;
    frame[1] = (PACKET_HEADER >> 16) & 0xFF;
    frame[2] = (PACKET_HEADER >> 8) & 0xFF;
    frame[3] = (PACKET_HEADER) & 0xFF;
    frame[4] = device_id;
    frame[5] = cmd;
    frame[6] = 0;
    frame[7] = 0;
    frame[8] = Compute_Checksum(frame, 8);
}

uint8_t Compute_Checksum(uint8_t *frame, uint8_t size) {
    uint8_t checksum = 0;
    for (uint8_t i = 4; i < size; i++) {
        checksum ^= frame[i];
    }
    return checksum;
}

void send_lidar_cmd(uint8_t cmd, uint8_t device_id) {
    uint8_t frame[9];
    create_frame(frame, GS2_DEVICE1_ADDRESS, cmd);
    Send_Uart1_Buff_Cmd(frame, sizeof(frame));
}

    
void init_lidar() {
    if (lidar_init == 0) {
        send_lidar_cmd(GS_LIDAR_CMD_GET_ADDRESS, GS2_DEVICE1_ADDRESS);
        lidar_init = 1;
        xil_printf("init done = 1\r\n");
    }

}

void Lidar_Loop() {
    uint8_t c;
    if (Get_Uart1_Cmd(&c)) {
        Lidar_Frame_Receive(&c, 1);
        xil_printf("%d\n", c);
    } 
}

void Lidar_Frame_Receive (uint8_t Buff[], uint8_t Len)
{
	uint8_t i;
	for (i = 0; i < Len; i++) {
		Lidar_Frame_Buf[i_Lidar_In_Buff_TODO] = Buff[i];
        i_Lidar_In_Buff_TODO++;
        if (i_Lidar_In_Buff_TODO >= STD_COM_SIZE_BUFF)
            i_Lidar_In_Buff_TODO = 0;
	}
}
