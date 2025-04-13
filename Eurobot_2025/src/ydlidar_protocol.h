
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
#define GS2_ALL_DEVICE_ADDRESS                 0x00
#define GS2_DEVICE1_ADDRESS                    0x01
#define GS2_DEVICE2_ADDRESS                    0x02
#define GS2_DEVICE3_ADDRESS                    0x04

#define Angle_Px 1.22
#define Angle_Py 5.315
#define Angle_PAngle 22.5 //GS2

typedef struct {
    uint8_t syncByte0;
    uint8_t syncByte1;
    uint8_t syncByte2;
    uint8_t syncByte3;
    uint8_t address;
    uint8_t cmd;
    uint16_t size;
    uint16_t env;
    uint8_t data[320];
}gs2_lidar_frame;

