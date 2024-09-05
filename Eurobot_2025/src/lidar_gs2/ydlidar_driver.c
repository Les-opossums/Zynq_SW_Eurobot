#include "ydlidar.h"

uint8_t isConnected = 0;


uint32_t sendCommand(uint8_t cmd,
                        const void *payload,
                        size_t payloadsize)
{
    uint8_t addr = 0x00;
    uint8_t pkt_header[12];
    cmd_packet_gs *header = reinterpret_cast<cmd_packet_gs * >(pkt_header);
    uint8_t checksum = 0;

    if (!isConnected) {
        return RESULT_FAIL;
    }

    header->syncByte0 = LIDAR_CMD_SYNC_BYTE;
    header->syncByte1 = LIDAR_CMD_SYNC_BYTE;
    header->syncByte2 = LIDAR_CMD_SYNC_BYTE;
    header->syncByte3 = LIDAR_CMD_SYNC_BYTE;
    header->address = addr;
    header->cmd_flag = cmd;
    header->size = 0xffff&payloadsize;
    sendData(pkt_header, 8) ;
    checksum += cmd;
    checksum += 0xff&header->size;
    checksum += 0xff&(header->size>>8);

    if (payloadsize && payload) {
      for (size_t pos = 0; pos < payloadsize; ++pos) {
        checksum += ((uint8_t *)payload)[pos];
      }
      uint8_t sizebyte = (uint8_t)(payloadsize);
      sendData((const uint8_t *)payload, sizebyte);
    }

    sendData(&checksum, 1);

    return RESULT_OK;
}
