#include "CORE_ID/core_id.h"
#include "IO_config.h"
#include "IPC_MANAGER/IPC_manager.h"
#include "IRQ_MANAGER/IRQ_manager.h"
#include "TIMER_MANAGER/timer_manager.h"
#include "APP_COM/com_interpreter_loop.h"

// temporaire pour debug
#include "APP_TEST/led_au_test.h"

// definition du callback quand l'un ou l'autre des cores reçoit une interruption SGI de l'autre core
void On_IPC_Message_Received(void *callback_ref) {
    #if THIS_CORE_ID == CORE_ID_CPU0
        // On est sur le core 0, on a reçu une interruption du core 1
    #else
        // On est sur le core 1, on a reçu une interruption du core 0
    #endif
}

int main(void){
    IPC_Init();
    IRQ_Manager_Init();
    Timer_Manager_Init();

    IRQ_Manager_Connect(IPC_SGI_INT_ID, On_IPC_Message_Received, NULL);

    IO_Manager_Init();
    IRQ_Manager_Start();
    IPC_SyncCores();

    #if THIS_CORE_ID == CORE_ID_CPU0
        u32 imu_print_timer_old = 0;
    #else
    #endif

    xil_printf("Entree dans la boucle principale\n");

    while(1){
        IO_Manager_Update();

        #if THIS_CORE_ID == CORE_ID_CPU0
            // execution du code du core 0
            // LED_AU_Test_Update();
            Com_Interpreter_Update();

            // if (Timer_ms1 - imu_print_timer_old > 200) {
            //     imu_print_timer_old = Timer_ms1;
            //     BNO085_Data *imu_data = BNO085_GetData(&Imu_Ctx.dev);
            //     xil_printf("[IMU] gyro (mrad/s) x=%ld y=%ld z=%ld calib=%d\n",
            //         (long)(imu_data->gyro.x * 1000.0f),
            //         (long)(imu_data->gyro.y * 1000.0f),
            //         (long)(imu_data->gyro.z * 1000.0f),
            //         imu_data->calib_status);
            // }
        #else
            // execution du code du core 1
        #endif
    }
}