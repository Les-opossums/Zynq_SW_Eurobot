#ifndef LD19_H
#define LD19_H

#include "obstacle_extractor.h"

#define LD19_HEADER 0x54
#define LD19_VER_SIZE 0x2C

#define LD19_PTS_PER_PACKETS 12
#define LD19_PACKET_SIZE 47 // 1(header) + 1(ver_len) + 2(speed) + 2(start_angle) + 12*3(point) + 2(end_angle) + 2(timestamp) + 1(crc8)
#define LD19_DATA_SIZE 3

#define LD19_MAX_PTS_SCAN 1200 

#define LD19_ANGLE_STEP_MAX 5


#define M_PI 3.14159265358979323846

/**
 * @brief Structure representing a single measurement from the LD19 sensor.
 * 
 * @note This structure is packed to ensure proper alignment and size.
 */
typedef struct {
    union {
        struct {
            uint16_t distance;  // mm
            uint8_t intensity;  // 0-255
        } __attribute__((packed));
        uint8_t bytes[LD19_DATA_SIZE];
    };
} __attribute__((packed)) LD19Measure;

/**
 * @brief Union representing a single packet from the LD19 sensor.
 * This union is used to represent the different fields of a packet.
 * 
 * @note This union is packed to ensure proper alignment and size.
 */
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

/**
 * @brief Structure representing a single data point from the LD19 sensor. Each packet is composed of 12 data points.
 * 
 */
typedef struct {
    float angle;
    uint16_t distance;
    uint8_t intensity;
    float x;
    float y;
} LD19DataPoint;

/**
 * @brief Structure representing a collection of data points from the LD19 sensor.
 * 
 */
typedef struct {
    LD19DataPoint points[LD19_MAX_PTS_SCAN];
    uint16_t index;
} LD19DataPointHandler;

/**
 * @brief Structure representing a single packet from the LD19 sensor.
 * 
 */
typedef struct {
    LD19Packet packet;
    uint16_t index;
    uint8_t computedCrc;
} LD19PacketHandler;

/**
 * @brief Structure representing a single scan from the LD19 sensor.
 * 
 */
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

/** Initialize the LD19 instance with default settings.
 * 
 * @param self Pointer to the LD19Instance to initialize.
 * 
 * @note This function should be called before using the LD19 instance
 * It will initialize :
 * - Buffers
 * - Reception
 * - Settings
 */
void LD19_init(LD19Instance *self);

/** Read data from the LD19 sensor.
 * 
 * @param self Pointer to the LD19Instance.
 * @param UartLite Pointer to the XUartLite instance. each instance shall be unique and not shared between different LD19 instances.
 * @return 1 if data was read successfully, 0 otherwise.
 */
uint8_t LD19_readData(LD19Instance *self, XUartLite *UartLite);

/**
 * @brief Read data from the LD19 sensor with CRC check.
 * 
 * @param self Pointer to the LD19Instance.
 * @param UartLite Pointer to the XUartLite instance.
 * @return 1 if data was read successfully, 0 otherwise.
 */
uint8_t LD19_readDataCRC(LD19Instance *self, XUartLite *UartLite);

/**
 * @brief Read data from the LD19 sensor without CRC check.
 * 
 * @param self Pointer to the LD19Instance.
 * @param UartLite Pointer to the XUartLite instance.
 * @return 1 if data was read successfully, 0 otherwise.
 */
uint8_t LD19_readDataNoCRC(LD19Instance *self, XUartLite *UartLite);

// Scan complet
/**
 * @brief Read a complete scan from the LD19 sensor.
 * 
 * @param self Pointer to the LD19Instance.
 * @param UartLite Pointer to the XUartLite instance.
 * @return 1 if data was read successfully, 0 otherwise.
 * 
 * @note This function will read a complete scan from the LD19 sensor. 
 */
uint8_t LD19_readScan(LD19Instance *self, XUartLite *UartLite);

// Traitement
/**
 * @brief Compute the data from the LD19 sensor.
 * 
 * @param self Pointer to the LD19Instance.
 * 
 * @note This function will compute the angles for each data point in the current scan.
 */
void LD19_computeData(LD19Instance *self);

/**
 * @brief Swap the buffers used for scanning.
 * 
 * @param self Pointer to the LD19Instance.
 * 
 * @note This function will swap betweenthe current and previous scan buffers.
 */
void LD19_swapBuffers(LD19Instance *self);

// Configuration
/**
 * @brief Enable CRC checking for the LD19 sensor.
 * 
 * @param self Pointer to the LD19Instance.
 */
void LD19_enableCRC(LD19Instance *self);

/**
 * @brief Disable CRC checking for the LD19 sensor.
 * 
 * @param self Pointer to the LD19Instance.
 */
void LD19_disableCRC(LD19Instance *self);

/**
 * @brief Enable full scan mode for the LD19 sensor.
 * 
 * @param self Pointer to the LD19Instance.
 */
void LD19_enableFullScan(LD19Instance *self);

/**
 * @brief Disable full scan mode for the LD19 sensor.
 * 
 * @param self Pointer to the LD19Instance.
 */
void LD19_disableFullScan(LD19Instance *self);

/**
 * @brief Enable filtering for the LD19 sensor.
 * 
 * @param self Pointer to the LD19Instance.
 */
void LD19_enableFiltering(LD19Instance *self);

/**
 * @brief Disable filtering for the LD19 sensor.
 * 
 * @param self Pointer to the LD19Instance.
 */
void LD19_disableFiltering(LD19Instance *self);

/**
 * @brief Set the intensity threshold for the LD19 sensor.
 * 
 * @param self Pointer to the LD19Instance.
 * @param threshold The intensity threshold value in arbitrary units.
 * 
 * @note Points with intensity below this threshold will be ignored if filtering is enabled.
 */
void LD19_setIntensityThreshold(LD19Instance *self, uint8_t threshold);

/**
 * @brief Set the distance range for the LD19 sensor.
 * 
 * @param self Pointer to the LD19Instance.
 * @param minDist The minimum distance value in millimeters.
 * @param maxDist The maximum distance value in millimeters.
 * 
 * @note Points with distance outside this range will be ignored if filtering is enabled.
 */
void LD19_setDistanceRange(LD19Instance *self, uint16_t minDist, uint16_t maxDist);

/**
 * @brief Set the angle range for the LD19 sensor.
 * 
 * @param self Pointer to the LD19Instance.
 * @param minAngle The minimum angle value in degrees.
 * @param maxAngle The maximum angle value in degrees.
 * 
 * @note Points with angle outside this range will be ignored if filtering is enabled.
 */
void LD19_setAngleRange(LD19Instance *self, int16_t minAngle, int16_t maxAngle);

/**
 * @brief Set the upside-down mode for the LD19 sensor.
 * 
 * @param self Pointer to the LD19Instance.
 * @param upsideDown The upside-down mode (1 to enable, 0 to disable).
 * 
 * @note This function will set the upside-down mode for the LD19 sensor.
 */
void LD19_setUpsideDown(LD19Instance *self, uint8_t upsideDown);

/**
 * @brief Set the offset position for the LD19 sensor.
 * 
 * @param self Pointer to the LD19Instance.
 * @param xPos The X position offset in millimeters.
 * @param yPos The Y position offset in millimeters.
 * @param anglePos The angle position offset in degrees.
 */
void LD19_setOffsetPosition(LD19Instance *self, int16_t xPos, int16_t yPos, float anglePos);

/**
 * @brief Set the base position for the LD19 sensor.
 * 
 * @param self Pointer to the LD19Instance.
 * @param xBase The X base position in millimeters.
 * @param yBase The Y base position in millimeters.
 * @param angleBase The angle base position in degrees.
 *
 * @note This function will set the base position for the LD19 sensor that depends on the mechanical setup.
 */
void LD19_setBasePosition(LD19Instance *self, float xBase, float yBase, float angleBase);

/**
 * @brief Get the number of points in the current scan.
 * 
 * @param self Pointer to the LD19Instance.
 * @return uint16_t The number of points in the current scan.
 */
uint16_t LD19_getNbPointsInScan(LD19Instance *self);

/**
 * @brief Get the speed of the LD19 sensor.
 * 
 * @param self Pointer to the LD19Instance.
 * @return uint16_t The speed of the LD19 sensor.
 * 
 * @note This function will return the speed of the LD19 sensor in millimeters per second, and can be used for a controlled loop on lidar speed.
 */
uint16_t LD19_getSpeed(LD19Instance *self);

/**
 * @brief Print the current scan data in CSV format.
 * 
 * @param self Pointer to the LD19Instance.
 */
void LD19_printScanCSV(LD19Instance *self);

/**
 * @brief Print the current scan data in Teleplot format.
 * 
 * @param self Pointer to the LD19Instance.
 */
void LD19_printScanTeleplot(LD19Instance *self);

/**
 * @brief Check if a new scan is available.
 * 
 * @param self Pointer to the LD19Instance.
 * @return uint8_t 1 if a new scan is available, 0 otherwise.
 */
uint8_t LD19_isNewScan(LD19Instance *self);

/**
 * @brief Get a data point from the current scan.
 * 
 * @param self Pointer to the LD19Instance.
 * @param n The index of the data point to retrieve.
 * @return LD19DataPoint* Pointer to the requested data point, or NULL if not available.
 */
LD19DataPoint *LD19_getPoint(LD19Instance *self, uint16_t n);

/**
 * @brief Get the number of checksum failures.
 * 
 * @param self Pointer to the LD19Instance.
 * @return uint16_t The number of checksum failures.
 */
uint16_t LD19_getChecksumFailCount(LD19Instance *self);

/**
 * @brief Check if a checksum failure occurred.
 * 
 * @param self Pointer to the LD19Instance.
 * @return uint8_t 1 if a checksum failure occurred, 0 otherwise.
 */
uint8_t LD19_isChecksumFail(LD19Instance *self);

/**
 * @brief Get the angle step between data points.
 * 
 * @param self Pointer to the LD19Instance.
 * @return float The angle step in degrees.
 *
 * @note This function will return the angle step between data points in degrees. angles are calculated from the start and end angles of the scan and the number of points in the scan. (12 points)
 */
static inline float LD19_getAngleStep(LD19Instance *self){
    float fsa = (float)self->receivedData.packet.startAngle / 100.0;
    float lsa = (float)self->receivedData.packet.endAngle / 100.0;

    float range = lsa - fsa;
    if (range < 0)
        range += 360;

    float angleStep = range / (LD19_PTS_PER_PACKETS - 1);
    return angleStep;
}

/**
 * @brief Get the angle step between data points.
 *
 * @param self Pointer to the LD19Instance.
 * @param data Pointer to the LD19DataPoint.
 * @return uint8_t The angle step in degrees.
 */
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

/**
 * @brief Convert LD19 data points to OE points. OE stands for Obstacle Extractor.
 *
 * @param ld Pointer to the LD19Instance.
 * @param out_pts Pointer to the output array of OE_Point.
 * @param out_n Pointer to the output number of points.
 */
void LD19_to_oe_points(const LD19Instance* ld, OE_Point* out_pts,int* out_n);

/**
 * @brief Print the detected obstacles.
 *
 * @param ld Pointer to the LD19Instance.
 *
 * @note This function will print the detected obstacles in a teleplot format.
 */
void LD19_print_obstacle(const LD19Instance* ld);


#endif // LD19_H
