// ---------------------
// buffer and dma parameters
// ---------------------
#define DMA_DEV_ID      XPAR_AXIDMA_0_DEVICE_ID
#define RX_BUFFER_SIZE  48 // Taille en bytes (ex : 12 points * 4 bytes)

#define LIDAR_REG_BASE   XPAR_OPOSSUM_LIDAR_0_BASEADDR

#define NUM_BUFFERS        2 // 
#define DMA_ALIGN          32

#define DMA_TIMEOUT        100 // Timeout en ms

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

extern u8 RxBuf[NUM_BUFFERS][FRAME_BYTES] __attribute__ ((aligned(DMA_ALIGN)));


typedef struct {
    uint32_t dist_min;
    uint32_t dist_max;
    uint32_t angle_min;
    uint32_t angle_max;
    uint32_t intensity_min;
    uint32_t ctrl;
    uint32_t frame_count;
    uint32_t error_count;
} LD19Register;

extern LD19Register lidar_1_reg;

typedef struct {
    uint16_t dist_mm; // [31:16]
    uint8_t angle_deg; // [15:8]
    uint8_t intensity; // [7:0]
} LidarPoint;

int init_dma(void);

void init_lidar(LD19Register *reg);

int dma_recv_frame_blocking(u8 *dst, u32 len_bytes);


void dump_frame(const u8 *buf, u32 len);

void lidar_wr32(u32 off, u32 val);
u32  lidar_rd32(u32 off);

int lidar_dma_recv(void *dst, u32 len_bytes);