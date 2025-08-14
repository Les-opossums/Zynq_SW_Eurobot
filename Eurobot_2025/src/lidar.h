// ---------------------
// buffer and dma parameters
// ---------------------
#define DMA_DEV_ID      XPAR_AXIDMA_0_DEVICE_ID
#define RX_BUFFER_SIZE  4096 // Taille en bytes (ex : 1024 points * 4 bytes)

#define LIDAR_REG_BASE   XPAR_OPOSSUM_LIDAR_0_BASEADDR

#define NUM_BUFFERS        2 // 
#define DMA_ALIGN          64

// offset param register
#define REG_DIST_MIN       0x00
#define REG_DIST_MAX       0x04
#define REG_ANGLE_MIN      0x08
#define REG_ANGLE_MAX      0x0C
#define REG_INTENSITY_MIN  0x10
#define REG_CTRL           0x14
#define REG_FRAME_COUNT    0x18
#define REG_ERROR_COUNT    0x1C

// ---------- Format des points ----------
#define BYTES_PER_POINT    4    // [31:16]=dist, [15:8]=angle8, [7:0]=intensité
#define POINTS_PER_FRAME   12   // 12 points per frame
#define FRAME_BYTES        (POINTS_PER_FRAME * BYTES_PER_POINT)


typedef struct {
    u32 dist_min;
    u32 dist_max;
    u32 angle_min;
    u32 angle_max;
    u32 intensity_min;
    u32 ctrl;
    u32 frame_count;
    u32 error_count;
} LD19Register;

typedef struct {
    uint16_t dist_mm; // [31:16]
    uint8_t angle_deg; // [15:8]
    uint8_t intensity; // [7:0]
} LidarPoint;

int init_dma(void);

int lidar_read_block(u32 nb_bytes);

void parse_lidar_data(u32 nb_words);