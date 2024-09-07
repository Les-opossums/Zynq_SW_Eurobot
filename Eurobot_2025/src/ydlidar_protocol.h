
//gs2
#define GS_LIDAR_CMD_GET_ADDRESS               0x60
#define GS_LIDAR_CMD_GET_PARAMETER             0x61
#define GS_LIDAR_CMD_GET_VERSION               0x62
#define GS_LIDAR_CMD_SCAN                      0x63
#define GS_LIDAR_CMD_STOP                      0x64
#define GS_LIDAR_CMD_RESTART                   0x67
#define GS_LIDAR_CMD_SET_BAUDRATE              0x68
#define GS_LIDAR_CMD_SET_EDGE_MODE             0x69

#define PACKET_HEADER                          0xA5A5A5A5
#define GS2_DEVICE1_ADDRESS                    0x01
#define GS2_DEVICE2_ADDRESS                    0x02
#define GS2_DEVICE3_ADDRESS                    0x04

struct gs2_lidar_header {
    uint8_t syncByte0;
    uint8_t syncByte1;
    uint8_t syncByte2;
    uint8_t syncByte3;
    uint8_t address;
    uint8_t type;
    uint16_t size;
};
struct cmd_packet_gs {
    uint8_t syncByte0;
    uint8_t syncByte1;
    uint8_t syncByte2;
    uint8_t syncByte3;
    uint8_t address;
    uint8_t cmd_flag;
    uint16_t size;
};

