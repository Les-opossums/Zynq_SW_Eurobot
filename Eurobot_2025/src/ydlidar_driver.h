void create_frame(uint8_t *frame, uint8_t device_id, uint8_t cmd);
uint8_t Compute_Checksum(uint8_t *frame, uint8_t size);
void send_lidar_cmd(uint8_t cmd);