#include "../../opossum_common/app_interface.h"
#include "ASSERV/asserv_loop.h"

void App_Init(void) {
    // Initialisations spécifiques au CPU1

    asserv_loop_init();
}

void App_Loop(void) {
    asserv_loop_update();
}
