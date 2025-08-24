#include "../main.h"
#include "ld19crc.h"


void LD19_init(LD19Instance *self) {
    // Tout mettre à zéro par défaut
    memset(self, 0, sizeof(LD19Instance));

    // Initialisation des pointeurs de scan
    self->currentScan = &self->scanA;
    self->previousScan = &self->scanB;
    // Valeurs par défaut des settings
    self->useCRC = 1;         // activer CRC par défaut
    self->fullScan = 1;       // utiliser un scan complet
    self->useFiltering = 0;   // filtrage activé
    self->upsideDown = 0;     // pas inversé

    self->threshold = 0;      // seuil 0
    self->minDist = 0;       // distance min 0 mm
    self->maxDist = 12000;    // distance max 12 m
    self->minAngle = 0;       // angle min 0°
    self->maxAngle = 360;   // angle max 360°

    self->xOffset = 0;
    self->yOffset = 0;
    self->angularOffset = 0.0f;

    self->xPosition = 0;    
    self->yPosition = 0;
    self->angularPosition = 0.0f;

    self->newScan = 0;
    self->currentBuffer = 0;
    self->checksumFailCount = 0;

    // initialiser les tableaux à 0
    for (int i = 0; i < LD19_PTS_PER_PACKETS; i++) {
        self->angles[i] = 0.0f;
    }

    self->currentScan->index = 0;
    self->previousScan->index = 0;

    self->receivedData.index = 0;
    self->receivedData.computedCrc = 0;

}


uint8_t LD19_readData(LD19Instance *self, XUartLite *UartLite) {
    // printf("Reading data from LD19...\n");
    return self->useCRC ? LD19_readDataCRC(self, UartLite) : LD19_readDataNoCRC(self, UartLite);
}

uint8_t LD19_readDataCRC(LD19Instance *self, XUartLite *UartLite) {
    uint8_t result = 0;
    uint8_t current = 0;
    while (XUartLite_Recv(UartLite, &current, 1)) {
        if (self->receivedData.index > 1 ||
            (self->receivedData.index == 0 && current == LD19_HEADER) ||
            (self->receivedData.index == 1 && current == LD19_VER_SIZE)) {

            self->receivedData.packet.bytes[self->receivedData.index] = current;
            if (self->receivedData.index < LD19_PACKET_SIZE - 1) {
                self->receivedData.computedCrc = CrcTable[self->receivedData.computedCrc ^ current];
                self->receivedData.index++;
            } else {
                if (self->receivedData.computedCrc == current) {
                    LD19_computeData(self);
                    result = 1;
                } else {
                    self->checksumFailCount++;
                }
                self->receivedData.index = 0;
                self->receivedData.computedCrc = 0;
            }
        } else {
            self->receivedData.index = 0;
            self->receivedData.computedCrc = 0;
        }
    }
    return result;
}

uint8_t LD19_readDataNoCRC(LD19Instance *self, XUartLite *UartLite) {
    uint8_t current = 0;
    while (XUartLite_Recv(UartLite, &current, 1)) {
        // printf("Received byte: 0x%02X\n", current);
        if (self->receivedData.index > 1 ||
            (self->receivedData.index == 0 && current == LD19_HEADER) ||
            (self->receivedData.index == 1 && current == LD19_VER_SIZE)) {
                self->receivedData.packet.bytes[self->receivedData.index] = current;
                self->receivedData.index++;
                if (self->receivedData.index == LD19_PACKET_SIZE - 1) {
                    LD19_computeData(self);
                    self->receivedData.index = 0;
                    return 1;
                }
        } else {
            self->receivedData.index = 0;
        }
    }
    return 0;
}

// ====================== SCAN ======================
uint8_t LD19_readScan(LD19Instance *self, XUartLite *UartLite) {
    static uint8_t isInit = 0;
    static float lastAngle = 0;
    static float startAngle = 0;

    self->newScan = 0;
    uint8_t result = 0;
    LD19DataPoint data;

    if (LD19_readData(self, UartLite)) {
        // printf("LD19 read scan data\n");
        for (int i = 0; i < LD19_PTS_PER_PACKETS; i++) {
            if (self->angles[i] < lastAngle) {
                if (!isInit) {
                    isInit = 1;
                } else {
                    if (lastAngle - startAngle > 340) {
                        self->newScan = 1;
                        result = 1;
                    }
                    startAngle = self->angles[i];
                    if (self->fullScan) {
                        LD19_swapBuffers(self);
                    }
                }
            }
            lastAngle = self->angles[i];

            if (self->currentScan->index < LD19_MAX_PTS_SCAN) {
                data.angle = self->angles[i];
                data.distance = self->receivedData.packet.measures[i].distance;
                data.intensity = self->receivedData.packet.measures[i].intensity;

                if (!self->useFiltering || LD19_filter(self, &data)) {
                    data.x = self->xPosition + self->xOffset * cos(self->angularPosition)
                             - self->yOffset * sin(self->angularPosition)
                             + data.distance * cos((data.angle + self->angularPosition + self->angularOffset) * M_PI / 180.0);

                    data.y = self->yPosition + self->xOffset * sin(self->angularPosition)
                             + self->yOffset * cos(self->angularPosition)
                             - data.distance * sin((data.angle + self->angularPosition + self->angularOffset) * M_PI / 180.0);
                    self->currentScan->points[self->currentScan->index] = data;
                    self->currentScan->index++;
                }
            }
        }
        if (!self->fullScan) {
            LD19_swapBuffers(self);
        }
    }
    return result;
}

// ====================== DATA PROCESSING ======================
void LD19_computeData(LD19Instance *self) {
    float angleStep = LD19_getAngleStep(self);
    if (angleStep > LD19_ANGLE_STEP_MAX)
        return;

    int8_t reverse = (self->upsideDown ? -1 : 1);
    float fsa = self->receivedData.packet.startAngle / 100.0;
    for (uint16_t i = 0; i < LD19_PTS_PER_PACKETS; i++) {
        float raw_deg = fsa + i * angleStep;
        self->angles[i] = (raw_deg <= 360 ? raw_deg : raw_deg - 360) * reverse;
    }
}

// ====================== BUFFER SWAP ======================
void LD19_swapBuffers(LD19Instance *self) {
    if (self->currentBuffer) {
        self->currentScan = &self->scanB;
        self->previousScan = &self->scanA;
    } else {
        self->currentScan = &self->scanA;
        self->previousScan = &self->scanB;
    }
    self->currentBuffer = !self->currentBuffer;
    self->currentScan->index = 0;
}

// ====================== SETTINGS ======================
void LD19_enableCRC(LD19Instance *self) { 
    self->useCRC = 1; 
}
void LD19_disableCRC(LD19Instance *self) { 
    self->useCRC = 0; 
}
void LD19_enableFullScan(LD19Instance *self) { 
    self->fullScan = 1; 
}
void LD19_disableFullScan(LD19Instance *self) { 
    self->fullScan = 0; 
}
void LD19_enableFiltering(LD19Instance *self) { 
    self->useFiltering = 1; 
}
void LD19_disableFiltering(LD19Instance *self) { 
    self->useFiltering = 0; 
}
void LD19_setIntensityThreshold(LD19Instance *self, uint8_t threshold) { 
    self->threshold = threshold; 
}
void LD19_setMaxDistance(LD19Instance *self, uint16_t maxDist) { 
    self->maxDist = maxDist; 
}

void LD19_setDistanceRange(LD19Instance *self, uint16_t minDist, uint16_t maxDist) {
    self->minDist = minDist;
    self->maxDist = maxDist;
}

int16_t LD19_rescaleAngle(int16_t angle) {
    if (angle > 360) angle %= 360;
    else while (angle < 0) angle += 360;
    return angle;
}

void LD19_setMinDistance(LD19Instance *self, uint16_t minDist) {
    self->minDist = LD19_rescaleAngle(minDist);
}

void LD19_setMaxAngle(LD19Instance *self, int16_t maxAngle) { 
    self->maxAngle = LD19_rescaleAngle(maxAngle); 
}
void LD19_setMinAngle(LD19Instance *self, int16_t minAngle) { 
    self->minAngle = minAngle; 
}
void LD19_setAngleRange(LD19Instance *self, int16_t minAngle, int16_t maxAngle) {
    self->minAngle = LD19_rescaleAngle(minAngle);
    self->maxAngle = LD19_rescaleAngle(maxAngle);
}
void LD19_setUpsideDown(LD19Instance *self, uint8_t upsideDown) { 
    self->upsideDown = upsideDown; 
}
void LD19_setOffsetPosition(LD19Instance *self, int16_t xPos, int16_t yPos, float anglePos) {
    self->xOffset = xPos;
    self->yOffset = yPos;
    self->angularOffset = anglePos;
}

void LD19_setBasePosition(LD19Instance *self, float xBase, float yBase, float angleBase) {
    self->xPosition = xBase;
    self->yPosition = yBase;
    self->angularPosition = angleBase;
}

uint16_t LD19_getNbPointsInScan(LD19Instance *self) {
    return self->previousScan->index;
}

uint16_t LD19_getSpeed(LD19Instance *self) {
    return self->receivedData.packet.speed;
}

void LD19_printScanCSV(LD19Instance *self) {
    for (uint16_t i = 0; i < self->currentScan->index; i++) {
        LD19DataPoint *p = &self->currentScan->points[i];
        printf("%.2f, %u, %u\n", p->angle, p->distance, p->intensity);
    }
}


void LD19_printScanTeleplot(LD19Instance *self) {
    if (self->previousScan->index) {
        printf("Lidar Scan: %d points\n", self->previousScan->index);
        printf(">lidar:");
        for (uint16_t i = 0; i < self->previousScan->index; i++) {
            printf("%d:%d;", (int)self->previousScan->points[i].x, (int)self->previousScan->points[i].y);
        }
        printf("|xy,clr\n");
    }
  }

uint8_t LD19_isNewScan(LD19Instance *self) {
    return self->newScan;
}

LD19DataPoint *LD19_getPoint(LD19Instance *self, uint16_t n) {
    static LD19DataPoint empty;
    LD19DataPoint *result = &empty;
    if (n < self->previousScan->index) {
        result = &self->previousScan->points[n];
    }
    return result;
}

uint16_t LD19_getChecksumFailCount(LD19Instance *self) {
    return self->checksumFailCount;
}

uint8_t LD19_isChecksumFail(LD19Instance *self) {
    static uint8_t previousChecksumFailCount = 0;
    uint8_t currentChecksumFailCount = LD19_getChecksumFailCount(self);
    uint8_t isFail = (currentChecksumFailCount > previousChecksumFailCount);
    previousChecksumFailCount = currentChecksumFailCount;
    return isFail;
}

void LD19_to_oe_points(const LD19Instance* ld, OE_Point* out_pts,int* out_n) {
    const LD19DataPointHandler* scan = ld->previousScan;
    int n = scan->index;  // nombre de points valides
    if (n > LD19_MAX_PTS_SCAN) n = LD19_MAX_PTS_SCAN;

    for (int i = 0; i < n; i++) {
        const LD19DataPoint* p = &scan->points[i];

        OE_Point q;
        q.x = p->x;  // mm
        q.y = p->y;  // mm

        out_pts[i] = q;
    }
    *out_n = n;
}


void LD19_print_obstacle(const LD19Instance* ld) {
    OE_Point pts[LD19_MAX_PTS_SCAN];
    int n = 0;
    LD19_to_oe_points(ld, pts, &n);

    if (n == 0) {
        printf("Pas de points dans le scan.\n");
        return;
    } else {
        printf("Nbrpoints: %d\n", n);
    }

    OE_Params p = oe_default_params();
    OE_Output out;
    oe_reset_output(&out);

    OE_Transform tf = { .enabled=1 }; // pas de transform pour l’instant
    oe_process_points(pts, n, &p, &tf, &out);

    // Segments détectés
    // printf("==== Segments (%d) ====\n", out.num_segments);
    // for (int i = 0; i < out.num_segments; i++) {
    //     OE_Segment* s = &out.segments[i];
    //     // convert to mm
    //     s->first_point.x *= 1000.0f;
    //     s->first_point.y *= 1000.0f;
    //     s->last_point.x *= 1000.0f;
    //     s->last_point.y *= 1000.0f;
    //     printf(">seg_start:%d:%d|xy,clr\n",
    //            (int)s->first_point.x, (int)s->first_point.y);
    //     printf(">seg_end:%d:%d|xy,clr\n",
    //            (int)s->last_point.x, (int)s->last_point.y);
    // }

    // Cercles détectés
    // printf("==== Cercles (%d) ====\n", out.num_circles);
    for (int i = 0; i < out.num_circles; i++) {
        OE_Circle* c = &out.circles[i];
        // convert to mm
        c->center.x *= 1000.0f;
        c->center.y *= 1000.0f;
        printf(">circle:%d:%d|xy,clr\n", (int)c->center.x, (int)c->center.y);
    }
}