#include "CORE_ID/core_id.h"
#include "IO_config.h"
#include "IPC_MANAGER/IPC_manager.h"
#include "IRQ_MANAGER/IRQ_manager.h"
#include "TIMER_MANAGER/timer_manager.h"
#include "APP_COM/com_interpreter_loop.h"
#include "app_interface.h"
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

    App_Init(); // init spécifique à l'application du coeur en cours de compilation

    xil_printf("Entree dans la boucle principale\n");

    while(1){
        IO_Manager_Update();

        App_Loop(); // boucle spécifique à l'application du coeur en cours de compilation
    }
}