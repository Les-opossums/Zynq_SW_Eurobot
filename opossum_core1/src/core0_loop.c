#include "../../opossum_common/app_interface.h"
#include "APP_COM/com_interpreter_loop.h"
// ... autres includes spécifiques ...

void App_Init(void) {
    // Initialisations spécifiques au CPU0
}

void App_Loop(void) {
    Com_Interpreter_Update();
    // Le reste du code de ton Core0_Loop
}