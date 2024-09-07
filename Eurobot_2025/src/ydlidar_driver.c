#include "main.h"

uint8_t lidar_init = 0;


uint8_t Lidar_Frame_Buf[LIDAR_BUFF_SIZE];
uint16_t i_Lidar_In_Buff_TODO = 0;
uint16_t i_Lidar_In_Buff_DONE = 0;

gs2_lidar_frame gs2_lidar_frame_buf[GS2_LIDAR_FRAME_BUFF_SIZE];
uint8_t cpt_frame = 0;

uint8_t lidar_state = 0;
uint32_t lidar_timer = 0;

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
    create_frame(frame, device_id, cmd);
    Send_Uart1_Buff_Cmd(frame, sizeof(frame));
}

    
void init_lidar_loop() {
    if (lidar_init == 0) {
        switch(lidar_state){
            case 0:
                send_lidar_cmd(GS_LIDAR_CMD_GET_ADDRESS, GS2_DEVICE1_ADDRESS);
                lidar_timer = Timer_ms1;
                lidar_state++;
                break;
            case 1:
                if (Timer_ms1 - lidar_timer > 1000) {
                    lidar_timer = Timer_ms1;
                    lidar_state++;
                    xil_printf("lidar_state: %d\n\r", lidar_state);
                }
                break;
            case 2:
                send_lidar_cmd(GS_LIDAR_CMD_GET_VERSION, GS2_DEVICE1_ADDRESS);
                lidar_state++;
                break;
            case 3:
                if (Timer_ms1 - lidar_timer > 1000) {
                    lidar_timer = Timer_ms1;
                    lidar_state++;
                    xil_printf("lidar_state: %d\n\r", lidar_state);
                }
                break;
            case 4:
                send_lidar_cmd(GS_LIDAR_CMD_GET_PARAMETER, GS2_DEVICE1_ADDRESS);
                lidar_state++;
                break;
            case 5:
                if (Timer_ms1 - lidar_timer > 1000) {
                    lidar_timer = Timer_ms1;
                    lidar_state++;
                    xil_printf("lidar_state: %d\n\r", lidar_state);
                }
                break;
            case 6:
                send_lidar_cmd(GS_LIDAR_CMD_SCAN, GS2_DEVICE1_ADDRESS);
                lidar_state = 0;
                lidar_init = 1;
                break;
        }

    }

}

gs2_lidar_frame gs2_lidar_frame_buf[GS2_LIDAR_FRAME_BUFF_SIZE];

void Lidar_Loop() {
    uint8_t c;
    if (Get_Uart1_Cmd(&c)) {
        Lidar_Frame_Receive(&c, 1);
    }
    // xil_printf("c : %d\n\r", c);
    // recreate the frame
    if(i_Lidar_In_Buff_TODO > i_Lidar_In_Buff_DONE){
        if(i_Lidar_In_Buff_TODO == 1){
            gs2_lidar_frame_buf[cpt_frame].syncByte0 = Lidar_Frame_Buf[i_Lidar_In_Buff_DONE];
        }else if(i_Lidar_In_Buff_TODO == 2){
            gs2_lidar_frame_buf[cpt_frame].syncByte1 = Lidar_Frame_Buf[i_Lidar_In_Buff_DONE];
        }else if(i_Lidar_In_Buff_TODO == 3){
            gs2_lidar_frame_buf[cpt_frame].syncByte2 = Lidar_Frame_Buf[i_Lidar_In_Buff_DONE];
        }else if(i_Lidar_In_Buff_TODO == 4){
            gs2_lidar_frame_buf[cpt_frame].syncByte3 = Lidar_Frame_Buf[i_Lidar_In_Buff_DONE];
        }else if(i_Lidar_In_Buff_TODO == 5){
            gs2_lidar_frame_buf[cpt_frame].address = Lidar_Frame_Buf[i_Lidar_In_Buff_DONE];
        }else if(i_Lidar_In_Buff_TODO == 6){
            gs2_lidar_frame_buf[cpt_frame].cmd = Lidar_Frame_Buf[i_Lidar_In_Buff_DONE];
        }else if(i_Lidar_In_Buff_TODO == 7){
            gs2_lidar_frame_buf[cpt_frame].size = (Lidar_Frame_Buf[i_Lidar_In_Buff_DONE]<<8);
        }else if(i_Lidar_In_Buff_TODO == 8){
            gs2_lidar_frame_buf[cpt_frame].size |= Lidar_Frame_Buf[i_Lidar_In_Buff_DONE];

        }else if(i_Lidar_In_Buff_TODO <= gs2_lidar_frame_buf[cpt_frame].size + 8){
            gs2_lidar_frame_buf[cpt_frame].data[i_Lidar_In_Buff_DONE - 8] = Lidar_Frame_Buf[i_Lidar_In_Buff_DONE];
        }else{
            i_Lidar_In_Buff_DONE = 0;
            i_Lidar_In_Buff_TODO = 0;

            xil_printf("frame received\n\r");
            xil_printf("syncByte0: %d\n\r", gs2_lidar_frame_buf[cpt_frame].syncByte0);
            xil_printf("syncByte1: %d\n\r", gs2_lidar_frame_buf[cpt_frame].syncByte1);
            xil_printf("syncByte2: %d\n\r", gs2_lidar_frame_buf[cpt_frame].syncByte2);
            xil_printf("syncByte3: %d\n\r", gs2_lidar_frame_buf[cpt_frame].syncByte3);
            xil_printf("address: %d\n\r", gs2_lidar_frame_buf[cpt_frame].address);
            xil_printf("cmd: %d\n\r", gs2_lidar_frame_buf[cpt_frame].cmd);
            xil_printf("size: %d\n\r", gs2_lidar_frame_buf[cpt_frame].size);
            if (gs2_lidar_frame_buf[cpt_frame].size > 0){
                xil_printf("data: ");
                for(uint16_t i = 0; i < gs2_lidar_frame_buf[cpt_frame].size; i++){
                    xil_printf("%d ", gs2_lidar_frame_buf[cpt_frame].data[i]);
                }
                xil_printf("\n\r");
            }

            cpt_frame++;
            if(cpt_frame >= GS2_LIDAR_FRAME_BUFF_SIZE){
                cpt_frame = 0;
            }
        }
        i_Lidar_In_Buff_DONE++;
    }

}

void Lidar_Frame_Receive (uint8_t Buff[], uint8_t Len)
{
	uint8_t i;
	for (i = 0; i < Len; i++) {
		Lidar_Frame_Buf[i_Lidar_In_Buff_TODO] = Buff[i];
        i_Lidar_In_Buff_TODO++;
        if (i_Lidar_In_Buff_TODO >= LIDAR_BUFF_SIZE)
            i_Lidar_In_Buff_TODO = 0;
	}
}


uint8_t Lidar_Start_Cmd(void) {
    send_lidar_cmd(GS_LIDAR_CMD_SCAN, GS2_DEVICE1_ADDRESS);
    return 0;
}

uint8_t Lidar_Restart_Cmd(void) {
    send_lidar_cmd(GS_LIDAR_CMD_RESTART, GS2_DEVICE1_ADDRESS);
    return 0;
}

uint8_t Lidar_Stop_Cmd(void) {
    send_lidar_cmd(GS_LIDAR_CMD_STOP, GS2_DEVICE1_ADDRESS);
    return 0;
}
