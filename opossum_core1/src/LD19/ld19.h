#ifndef LD19_H
#define LD19_H

#define LD19_HEADER 0x54
#define LD19_VER_SIZE 0x2C

#define LD19_PTS_PER_PACKETS 12
#define LD19_PACKET_SIZE 47 // 1(header) + 1(ver_len) + 2(speed) + 2(start_angle) + 12*3(point) + 2(end_angle) + 2(timestamp) + 1(crc8)
#define LD19_DATA_SIZE 3

#define LD19_MAX_PTS_SCAN 1200 

#define LD19_ANGLE_STEP_MAX 5


#define M_PI 3.14159265358979323846

typedef struct {
    union {
        struct {
            uint16_t distance;  // mm
            uint8_t intensity;  // 0-255
        } __attribute__((packed));
        uint8_t bytes[LD19_DATA_SIZE];
    };
} __attribute__((packed)) LD19Measure;

typedef union {
    struct {
        uint8_t header;
        uint8_t verLen;
        uint16_t speed;
        uint16_t startAngle;
        LD19Measure measures[LD19_PTS_PER_PACKETS];
        uint16_t endAngle;
        uint16_t timestamp;
        uint8_t crc8;
    } __attribute__((packed));
    uint8_t bytes[LD19_PACKET_SIZE];
} LD19Packet;


typedef struct {
    float angle;
    uint16_t distance;
    uint8_t intensity;
    float x;
    float y;
} LD19DataPoint;

typedef struct {
    LD19DataPoint points[LD19_MAX_PTS_SCAN];
    uint16_t index;
} LD19DataPointHandler;

typedef struct {
    LD19Packet packet;
    uint16_t index;
    uint8_t computedCrc;
} LD19PacketHandler;

typedef struct {
    // Buffers
    LD19DataPointHandler scanA;
    LD19DataPointHandler scanB;
    LD19DataPointHandler *currentScan;
    LD19DataPointHandler *previousScan;
    uint8_t currentBuffer;

    // Reception
    LD19PacketHandler receivedData;
    float angles[LD19_PTS_PER_PACKETS];
    uint32_t checksumFailCount;

    // Settings
    uint8_t useCRC;
    uint8_t fullScan;
    uint8_t useFiltering;
    uint8_t upsideDown;

    uint8_t threshold;
    uint16_t minDist;
    uint16_t maxDist;
    int16_t minAngle;
    int16_t maxAngle;

    int16_t xOffset;
    int16_t yOffset;
    float angularOffset;
    int16_t xPosition;
    int16_t yPosition;
    float angularPosition;

    uint8_t newScan;
} LD19Instance;

// Init
void LD19_init(LD19Instance *self);

// Lecture brute
uint8_t LD19_readData(LD19Instance *self, XUartLite *UartLite);
uint8_t LD19_readDataCRC(LD19Instance *self, XUartLite *UartLite);
uint8_t LD19_readDataNoCRC(LD19Instance *self, XUartLite *UartLite);

// Scan complet
uint8_t LD19_readScan(LD19Instance *self, XUartLite *UartLite);

// Traitement
void LD19_computeData(LD19Instance *self);
void LD19_swapBuffers(LD19Instance *self);

// Configuration
void LD19_enableCRC(LD19Instance *self);
void LD19_disableCRC(LD19Instance *self);
void LD19_enableFullScan(LD19Instance *self);
void LD19_disableFullScan(LD19Instance *self);
void LD19_enableFiltering(LD19Instance *self);
void LD19_disableFiltering(LD19Instance *self);
void LD19_setIntensityThreshold(LD19Instance *self, uint8_t threshold);
void LD19_setDistanceRange(LD19Instance *self, uint16_t minDist, uint16_t maxDist);
void LD19_setAngleRange(LD19Instance *self, int16_t minAngle, int16_t maxAngle);
void LD19_setUpsideDown(LD19Instance *self, uint8_t upsideDown);
void LD19_setOffsetPosition(LD19Instance *self, int16_t xPos, int16_t yPos, float anglePos);
void LD19_setBasePosition(LD19Instance *self, float xBase, float yBase, float angleBase);
uint16_t LD19_getNbPointsInScan(LD19Instance *self);
uint16_t LD19_getSpeed(LD19Instance *self);

void LD19_printScanCSV(LD19Instance *self);
void LD19_printScanTeleplot(LD19Instance *self);

uint8_t LD19_isNewScan(LD19Instance *self);
LD19DataPoint *LD19_getPoint(LD19Instance *self, uint16_t n);
uint16_t LD19_getChecksumFailCount(LD19Instance *self);
uint8_t LD19_isChecksumFail(LD19Instance *self);


static inline float LD19_getAngleStep(LD19Instance *self){
    float fsa = (float)self->receivedData.packet.startAngle / 100.0;
    float lsa = (float)self->receivedData.packet.endAngle / 100.0;

    float range = lsa - fsa;
    if (range < 0)
        range += 360;

    float angleStep = range / (LD19_PTS_PER_PACKETS - 1);
    return angleStep;
}

static inline uint8_t LD19_filter(LD19Instance *self, LD19DataPoint *data) {
    uint8_t distanceFilter = data->distance <= self->maxDist && data->distance >= self->minDist;
    uint8_t intensityFilter = data->intensity >= self->threshold;
    uint8_t angularFilter;
    if (self->minAngle <= self->maxAngle) {
        angularFilter = data->angle <= self->maxAngle && data->angle >= self->minAngle;
    } else {
        angularFilter = data->angle <= self->maxAngle || data->angle >= self->minAngle;
    }
    return distanceFilter && intensityFilter && angularFilter;
}

#endif // LD19_H
