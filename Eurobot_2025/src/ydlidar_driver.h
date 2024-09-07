#define LIDAR_BUFF_SIZE 1000

void create_frame(uint8_t *frame, uint8_t device_id, uint8_t cmd);
uint8_t Compute_Checksum(uint8_t *frame, uint8_t size);
void send_lidar_cmd(uint8_t cmd, uint8_t device_id);

void init_lidar_loop();
void Lidar_Loop();
void Lidar_Frame_Receive (uint8_t Buff[], uint8_t Len);


uint8_t Lidar_Start_Cmd(void);
uint8_t Lidar_Stop_Cmd(void);