#include "driver_bno085_io.h"
#include "../../IPC_MANAGER/IPC_manager.h"
#include "xstatus.h"

// Un seul capteur IMU dans ce projet — flag global suffit,
// posé en ISR (pin 60, EDGE_FALLING) et consommé dans BNO085_IO_Update.
static volatile u8 Bno085_DataReadyFlag = 0;

void BNO085_INT_Callback(void *callback_ref) {
    (void)callback_ref;
    // ISR : aucune opération SPI ici, on se contente de lever le flag.
    Bno085_DataReadyFlag = 1;
}

int BNO085_IO_Init(void *instance) {
    bno085_io_context_t *ctx = (bno085_io_context_t *)instance;

    int ret = BNO085_Init(&ctx->dev, ctx->gpio_ctx, ctx->pin_cs, ctx->pin_rst, ctx->pin_int);
    if (ret != BNO085_OK) {
        BNO085_LOG("[IMU] Echec initialisation (%d)\n", ret);
        return XST_FAILURE;
    }

    for (u32 i = 0; i < ctx->num_reports; i++) {
        BNO085_EnableReport(&ctx->dev, ctx->report_table[i].report_id, ctx->report_table[i].interval_us);
    }

    return XST_SUCCESS;
}

void BNO085_IO_Update(void *instance) {
    bno085_io_context_t *ctx = (bno085_io_context_t *)instance;

    if (!Bno085_DataReadyFlag) return;
    Bno085_DataReadyFlag = 0;

    if (BNO085_Poll(&ctx->dev) != BNO085_OK) return;
    if (!BNO085_DataReady(&ctx->dev)) return;

    // Republication en mémoire partagée pour CORE1
    BNO085_Data *data = BNO085_GetData(&ctx->dev);
    IPC_DATA->imu_gyro_x = data->gyro.x;
    IPC_DATA->imu_gyro_y = data->gyro.y;
    IPC_DATA->imu_gyro_z = data->gyro.z;
    IPC_DATA->imu_accel_x = data->accel.x;
    IPC_DATA->imu_accel_y = data->accel.y;
    IPC_DATA->imu_accel_z = data->accel.z;
    IPC_DATA->imu_calib_status = data->calib_status;
    IPC_DATA->imu_seq++;
}