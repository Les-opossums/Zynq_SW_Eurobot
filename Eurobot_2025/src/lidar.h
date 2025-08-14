// ---------------------
// Paramètres DMA et buffer
// ---------------------
#define DMA_DEV_ID      XPAR_AXIDMA_0_DEVICE_ID
#define RX_BUFFER_SIZE  4096 // Taille en bytes (ex : 1024 points * 4 bytes)

#define LIDAR_REG_BASE   XPAR_LIDAR_FILTER_REGS_0_S_AXI_BASEADDR

#define REG_DIST_MIN       0x00
#define REG_DIST_MAX       0x04
#define REG_ANGLE_MIN      0x08
#define REG_ANGLE_MAX      0x0C
#define REG_INTENSITY_MIN  0x10
#define REG_CTRL           0x14
#define REG_FRAME_COUNT    0x18
#define REG_ERROR_COUNT    0x1C

int init_dma(void);

int lidar_read_block(u32 nb_bytes);

void parse_lidar_data(u32 nb_words);