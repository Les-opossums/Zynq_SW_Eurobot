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
static BNO085_Dev imu;                /* Handle IMU — global ou static main */
static int        imu_ok     = 0;     /* 1 si init réussie                  */
static int        imu_last_print_ms = 0;

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
     *   Rotation Vector     → 10 ms  (100 Hz) — orientation absolue
     *   Game Rotation Vector → 10 ms (100 Hz) — orientation sans magnéto
     *
     * Pour le debug initial, 10 ms sur tout est suffisant.
     */
    xil_printf("[IMU] Activation des rapports...\r\n");
    BNO085_EnableReport(&imu, SH2_GYROSCOPE_CALIBRATED,  1000U);   /* 1 kHz  */
    BNO085_EnableReport(&imu, SH2_LINEAR_ACCELERATION,   5000U);   /* 200 Hz */
    BNO085_EnableReport(&imu, SH2_GAME_ROTATION_VECTOR, 10000U);   /* 100 Hz */
    BNO085_EnableReport(&imu, SH2_ROTATION_VECTOR,      10000U);   /* 100 Hz */
 
    imu_ok = 1;
    xil_printf("[IMU] Initialisation OK\r\n");
}
 
/* ─── Boucle principale (à appeler dans while(1)) ───────────────────────── */
#define RAD_TO_DEG 57.29578f

void IMU_Loop(void)
{
    if (!imu_ok) return;
 
    /* --- Poll IMU --- */
    BNO085_Poll(&imu);
 
    /* --- Affichage debug toutes les 100 ms --- */
    if ((Timer_ms1 - imu_last_print_ms) >= 100) {
        imu_last_print_ms = Timer_ms1;
 
        BNO085_Data *d = BNO085_GetData(&imu);
 
        /*
         * Format CSV sur une ligne → facile à parser avec un script Python
         * ou à visualiser dans un terminal série.
         *
         * Champs :
         *   gyro_x/y/z          [rad/s]  — vt = gyro_z pour un robot plan
         *   lin_ax/ay/az        [m/s²]   — accélération sans gravité
         *   yaw / pitch / roll  [°]
         *   qw/qi/qj/qk                  — quaternion AHRS
         *   calib                        — 0=non calibré, 3=pleinement calibré
         */
        printf("IMU g=%d,%d,%d a=%d,%d,%d ypr=%d,%d,%d q=%d,%d,%d,%d cal=%d\r\n",
        (int)(d->gyro.x * 100.0f), 
        (int)(d->gyro.y * 100.0f), 
        (int)(d->gyro.z * 100.0f),
        (int)(d->accel.x * 100.0f), 
        (int)(d->accel.y * 100.0f), 
        (int)(d->accel.z * 100.0f),
        (int)(d->yaw * RAD_TO_DEG),      // Angle en degrés !
        (int)(d->pitch * RAD_TO_DEG),    // Angle en degrés !
        (int)(d->roll * RAD_TO_DEG),     // Angle en degrés !
        (int)(d->rotation.real * 1000.0f), 
        (int)(d->rotation.i * 1000.0f), 
        (int)(d->rotation.j * 1000.0f), 
        (int)(d->rotation.k * 1000.0f),
        (int)d->calib_status
    );
    }
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
        if(Timer_ms1 - old_timer_ms1 >= 1000) {
            old_timer_ms1 = Timer_ms1;
            // printf("SGI recues : %lu\r\n", sgi_debug_count);
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
        IMU_Loop();            // Poll IMU + print debug
    }

    cleanup_platform();
    return 0;
}
