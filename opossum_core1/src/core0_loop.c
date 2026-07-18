#include "../../opossum_common/app_interface.h"
#include "../../opossum_common/IPC_MANAGER/IPC_manager.h"
#include "../../opossum_common/TIMER_MANAGER/timer_manager.h"
#include "APP_COM/com_interpreter_loop.h"
#include "xil_printf.h"
// ... autres includes spécifiques ...

void App_Init(void) {
    // Initialisations spécifiques au CPU0
}

void App_Loop(void) {
    Com_Interpreter_Update();
    // Le reste du code de ton Core0_Loop

    static u32 print_timer = 0;
    if (Timer_ms1 - print_timer > 500) { // On affiche toutes les 500ms
        print_timer = Timer_ms1;
        
        xil_printf("[IMU] Accel: X=%.2f Y=%.2f Z=%.2f | Status: %d\n", 
            IPC_DATA->imu_accel_x, 
            IPC_DATA->imu_accel_y, 
            IPC_DATA->imu_accel_z, 
            IPC_DATA->imu_calib_status);
    }
}
