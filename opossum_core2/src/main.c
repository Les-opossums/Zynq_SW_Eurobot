#include "main.h"
#include "lib_asserv/Lib_Asserv.h"

extern volatile uint32_t sgi_debug_count;
// #define COMM_VAL (*(volatile unsigned long *)(0xFFFF0000))
extern u32 MMUTable;

int old_timer_us1 = 0;
int old_timer_ms1 = 0;

int old_timer_AU = 0;

/**
 * @brief declaration pour l'imu
 * 
 */
BNO085_Dev imu;                /* Handle IMU — global ou static main */
int        imu_ok     = 0;     /* 1 si init réussie                  */
int        imu_last_print_ms = 0;

void IMU_Init(void)
{
    int ret = BNO085_Init(&imu);
    if (ret != BNO085_OK) {
        xil_printf("[IMU] Echec initialisation (%d) — IMU desactive\r\n", ret);
        return;
    }
 
    /*
     * Activation des rapports souhaités.
     * Intervalles recommandés pour la localisation robot :
     *   Gyroscope           → 1 ms   (1000 Hz) — critique pour intégration
     *   Accélération linéaire → 5 ms  (200 Hz)
     *   Rotation Vector     → 10 ms  (200 Hz) — orientation absolue
     *   Game Rotation Vector → 10 ms (100 Hz) — orientation sans magnéto
     */
    xil_printf("[IMU] Activation des rapports...\r\n");
    BNO085_EnableReport(&imu, SH2_GYROSCOPE_CALIBRATED,  1000U);
    // BNO085_EnableReport(&imu, SH2_LINEAR_ACCELERATION,   10000U);  
    // BNO085_EnableReport(&imu, SH2_GAME_ROTATION_VECTOR,  10000U); 
    // BNO085_EnableReport(&imu, SH2_ROTATION_VECTOR,       10000U);

    imu_ok = 1;
    xil_printf("[IMU] Initialisation OK\r\n");
}
 
int main()
{
    Xil_SetTlbAttributes(0xFFFF0000,0x14de2); 

    init_platform();
    print("CPU1: init_platform\n\r");


    int Status = SetupInterruptSystem(&InterruptController);

    if (Status != XST_SUCCESS) {
        xil_printf("Interrupt Setup Failed\r\n");
    } else {
        xil_printf("Interrupt Setup Done\r\n");
    }

    // initialise timer
    Status = Init_Timer_us1();
    if (Status != XST_SUCCESS) {
        xil_printf("CPU1: Timer initialization failed\n\r");
        return XST_FAILURE;
    }
    
    // initialise shared memory
    init_shared_memory();


    Init_CAN();
    Init_AU();
    Init_Asserv();
    
    IMU_Init();

    while(1){ 
        if(Timer_ms1 - old_timer_ms1 > 100) {
            printf("X: %.4f | Y: %.4f | Z: %.4f\n", imu.data.gyro.x, imu.data.gyro.y, imu.data.gyro.z);
            old_timer_ms1 = Timer_ms1;
        }
        AU_Loop();
        if(AU_state == 1){
            if(Timer_ms1 - old_timer_AU >= 100) {
                // xil_printf("AU activated, stopping asserv\n\r");
                old_timer_AU = Timer_ms1;
            }
            motion_free();
            Init_CAN_MOTOR_variables();
            Init_Asserv();
        }else{
            Asserv_Loop();
        }
    }

    cleanup_platform();
    return 0;
}
