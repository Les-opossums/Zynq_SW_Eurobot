#define RESULT_FAIL 0
#define RESULT_OK 1

uint32_t sendCommand(uint8_t cmd, 
                        const void *payload, 
                        size_t payloadsize);