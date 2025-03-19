#include "main.h"

uint8_t lidar_init = 0;


uint8_t Lidar_Frame_Buf[LIDAR_BUFF_SIZE];

gs2_lidar_frame gs2_lidar_frame_buf[GS2_LIDAR_FRAME_BUFF_SIZE];
uint16_t i_Lidar_In_Buff_TODO = 0;
uint16_t i_Lidar_In_Buff_DONE = 0;
uint8_t cpt_frame = 0;

uint8_t lidar_gs2_state = 0;
uint8_t header_nbr_byte = 0;
uint16_t data_length = 0;

uint8_t lidar_state = 0;
uint32_t lidar_timer = 0;

uint8_t lidar_error_nbr = 0;

uint8_t lidar_action = 0;

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
                printf("Send get address\n");
                send_lidar_cmd(GS_LIDAR_CMD_GET_ADDRESS, GS2_ALL_DEVICE_ADDRESS);
                lidar_timer = Timer_ms1;
                lidar_state++;
                break;
            case 1:
                if (Timer_ms1 - lidar_timer > 2000) {
                    lidar_timer = Timer_ms1;
                    lidar_state++;
                }
                break;
            case 2:
                printf("\n\n\n");
                printf("Send get version\n");
                send_lidar_cmd(GS_LIDAR_CMD_GET_VERSION, GS2_ALL_DEVICE_ADDRESS);
                lidar_timer = Timer_ms1;
                lidar_state++;
                break;
            case 3:
                if (Timer_ms1 - lidar_timer > 2000) {
                    lidar_timer = Timer_ms1;
                    lidar_state++;
                }
                break;
            case 4:
                printf("\n\n\n");
                printf("Send get parameter\n");
                send_lidar_cmd(GS_LIDAR_CMD_GET_PARAMETER, GS2_ALL_DEVICE_ADDRESS);
                lidar_timer = Timer_ms1;
                lidar_state++;

                break;
            case 5:
                if (Timer_ms1 - lidar_timer > 2000) {
                    lidar_timer = Timer_ms1;
                    lidar_state++;
                }
                break;
            case 6:
                printf("\n\n\n");
                printf("Send scan\n");
                send_lidar_cmd(GS_LIDAR_CMD_SCAN, GS2_ALL_DEVICE_ADDRESS);
                lidar_state = 0;
                lidar_init = 1;
                break;
        }

    }

}

int byte_ctr = 0;

void Lidar_scan_Loop() {
    uint8_t c;
    if (Get_Uart1_Cmd(&c)) {
        byte_ctr++;
        switch(lidar_gs2_state){
            // ##################################################################
            // #                                                                #
            // #                          PAQUET HEADER                         #
            // #                                                                #
            // ##################################################################
            case 0:
                if (c == 0xA5) { 
                    if (header_nbr_byte == 0) {
                        gs2_lidar_frame_buf[cpt_frame].syncByte0 = c;
                    } else if (header_nbr_byte == 1) {
                        gs2_lidar_frame_buf[cpt_frame].syncByte1 = c;
                    } else if (header_nbr_byte == 2) {
                        gs2_lidar_frame_buf[cpt_frame].syncByte2 = c;
                    } else if (header_nbr_byte == 3) {
                        gs2_lidar_frame_buf[cpt_frame].syncByte3 = c;
                    }
                    header_nbr_byte++;
                    if (header_nbr_byte == 4) {
                        lidar_gs2_state++;
                        header_nbr_byte = 0;
                        #ifdef DEBUG_LIDAR
                            // printf("header received\n");
                        #endif
                    }
                } else {
                    lidar_error_nbr++;
                    #ifdef DEBUG_LIDAR
                        // printf("error on header byte %d \n", header_nbr_byte);
                    #endif
                    if (lidar_error_nbr > MAX_NBR_ERROR) {
                        lidar_error_nbr = 0;
                        // lidar_init = 0;
                    }
                }
                break;

            // ##################################################################
            // #                                                                #
            // #                  Address reception                             #
            // #                                                                #
            // ##################################################################
            case 1: 
                gs2_lidar_frame_buf[cpt_frame].address = c;
                #ifdef DEBUG_LIDAR
                    printf("address received: %d\n", gs2_lidar_frame_buf[cpt_frame].address);                
                #endif
                lidar_gs2_state++;
                break;


            // ##################################################################
            // #                                                                #
            // #                  Command reception                             #
            // #                                                                #
            // ##################################################################
            case 2:
                gs2_lidar_frame_buf[cpt_frame].cmd = c;
                #ifdef DEBUG_LIDAR
                    printf("cmd: %d\n", gs2_lidar_frame_buf[cpt_frame].cmd);
                #endif
                lidar_gs2_state++;
                data_length = 0;
                break;


            // ##################################################################
            // #                                                                #
            // #                  Size reception                                #   
            // #                                                                #
            // ##################################################################
            case 3:
                if (data_length == 0) { //LSB
                    gs2_lidar_frame_buf[cpt_frame].size = c;
                    data_length++;
                } else { // MSB
                    gs2_lidar_frame_buf[cpt_frame].size = gs2_lidar_frame_buf[cpt_frame].size | (c << 8);
                    #ifdef DEBUG_LIDAR
                        printf("size: %d\n", gs2_lidar_frame_buf[cpt_frame].size);
                    #endif
                    data_length = 0;
                    if(gs2_lidar_frame_buf[cpt_frame].size == 0){
                        lidar_gs2_state = 5;
                        cpt_frame++;
                    }else{
                        lidar_gs2_state++;
                    }
                }
                break;

            // ##################################################################
            // #                                                                #
            // #                  Data reception                                #
            // #                                                                #
            // ##################################################################
            case 4:
                gs2_lidar_frame_buf[cpt_frame].data[data_length] = c;
                if (data_length == gs2_lidar_frame_buf[cpt_frame].size - 1) {
                    // #ifdef DEBUG_LIDAR
                        printf("data received :");
                        for (uint16_t i = 0; i < gs2_lidar_frame_buf[cpt_frame].size; i++) {
                            printf("%d ", gs2_lidar_frame_buf[cpt_frame].data[i]);
                        }
                        printf("\n");
                    // #endif
                    data_length = 0;
                    lidar_gs2_state++;
                }else{
                    data_length++;
                }
                break;

            // ##################################################################
            // #                                                                #
            // #                  Checksum reception                            #
            // #                                                                #
            // ##################################################################
            case 5:
                #ifdef DEBUG_LIDAR
                    printf("checksum: %d\n", c);
                #endif
                lidar_gs2_state = 0;
                i_Lidar_In_Buff_TODO++;
                if (i_Lidar_In_Buff_TODO >= LIDAR_BUFF_SIZE){
                    i_Lidar_In_Buff_TODO = 0;
                }
                cpt_frame++;
                if(cpt_frame >= GS2_LIDAR_FRAME_BUFF_SIZE){
                    cpt_frame = 0;
                }
                break;
        }
    }
}

void Lidar_Calculation_loop() {
    if(i_Lidar_In_Buff_DONE != i_Lidar_In_Buff_TODO){
        
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

uint8_t Lidar_Get_Version_Cmd(void) {
    send_lidar_cmd(GS_LIDAR_CMD_GET_VERSION, GS2_DEVICE1_ADDRESS);
    return 0;
}

uint8_t Lidar_Get_Parameter_Cmd(void) {
    send_lidar_cmd(GS_LIDAR_CMD_GET_PARAMETER, GS2_DEVICE1_ADDRESS);
    return 0;
}
