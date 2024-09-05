
#define SUNNOISEINTENSITY 0xff
#define GLASSNOISEINTENSITY 0xfe

#define LIDAR_CMD_STOP                      0x65
#define LIDAR_CMD_SCAN                      0x60
#define LIDAR_CMD_FORCE_SCAN                0x61
#define LIDAR_CMD_RESET                     0x80
#define LIDAR_CMD_FORCE_STOP                0x00
#define LIDAR_CMD_GET_EAI                   0x55
#define LIDAR_CMD_GET_DEVICE_INFO           0x90
#define LIDAR_CMD_GET_DEVICE_HEALTH         0x92
#define LIDAR_ANS_TYPE_DEVINFO              0x4
#define LIDAR_ANS_TYPE_DEVHEALTH            0x6
#define LIDAR_CMD_SYNC_BYTE                 0xA5
#define LIDAR_CMDFLAG_HAS_PAYLOAD           0x80
#define LIDAR_ANS_SYNC_BYTE1                0xA5
#define LIDAR_ANS_SYNC_BYTE2                0x5A
#define LIDAR_ANS_TYPE_MEASUREMENT          0x81
#define LIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define LIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1
#define LIDAR_RESP_MEASUREMENT_DISTANCE_SHIFT  2
#define LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT 8

#define LIDAR_CMD_RUN_POSITIVE             0x06
#define LIDAR_CMD_RUN_INVERSION            0x07
#define LIDAR_CMD_SET_AIMSPEED_ADDMIC      0x09
#define LIDAR_CMD_SET_AIMSPEED_DISMIC      0x0A
#define LIDAR_CMD_SET_AIMSPEED_ADD         0x0B
#define LIDAR_CMD_SET_AIMSPEED_DIS         0x0C
#define LIDAR_CMD_GET_AIMSPEED             0x0D

#define LIDAR_CMD_SET_SAMPLING_RATE        0xD0
#define LIDAR_CMD_GET_SAMPLING_RATE        0xD1
#define LIDAR_STATUS_OK                    0x0
#define LIDAR_STATUS_WARNING               0x1
#define LIDAR_STATUS_ERROR                 0x2

#define LIDAR_CMD_ENABLE_LOW_POWER         0x01
#define LIDAR_CMD_DISABLE_LOW_POWER        0x02
#define LIDAR_CMD_STATE_MODEL_MOTOR        0x05
#define LIDAR_CMD_ENABLE_CONST_FREQ        0x0E
#define LIDAR_CMD_DISABLE_CONST_FREQ       0x0F

#define LIDAR_CMD_GET_OFFSET_ANGLE          0x93
#define LIDAR_CMD_SAVE_SET_EXPOSURE         0x94
#define LIDAR_CMD_SET_LOW_EXPOSURE          0x95
#define LIDAR_CMD_ADD_EXPOSURE       	    0x96
#define LIDAR_CMD_DIS_EXPOSURE       	    0x97

#define Angle_Px   1.22
#define Angle_Py   5.315
#define Angle_PAngle   22.5

//gs2
#define GS_LIDAR_CMD_GET_ADDRESS               0x60
#define GS_LIDAR_CMD_GET_PARAMETER             0x61
#define GS_LIDAR_CMD_GET_VERSION               0x62
#define GS_LIDAR_CMD_SCAN                      0x63
#define GS_LIDAR_ANS_SCAN                      0x63
#define GS_LIDAR_CMD_STOP                      0x64
#define GS_LIDAR_CMD_RESET                     0x67
#define GS_LIDAR_CMD_SET_BIAS                  0xD9
#define GS_LIDAR_CMD_SET_DEBUG_MODE            0xF0

#define PackageMaxModuleNums  0x03
#define PackageSampleMaxLngth_GS 0xA0  //160*2


#define MaximumNumberOfPackages  765
#define PackageSampleMaxLngth 0x100


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