#include "../../opossum_common/app_interface.h"
#include "../../opossum_common/IPC_MANAGER/IPC_manager.h"
#include "../../opossum_common/TIMER_MANAGER/timer_manager.h"
#include "APP_ASSERV_BRIDGE/asserv_commands.h"
#include "APP_COM/com_interpreter_loop.h"
#include "xil_printf.h"
// ... autres includes spécifiques ...

void App_Init(void) {
    // Initialisations spécifiques au CPU0
}

void App_Loop(void) {
    Com_Interpreter_Update();
    Print_Position_loop();
}
