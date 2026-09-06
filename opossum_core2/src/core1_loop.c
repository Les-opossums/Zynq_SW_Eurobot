#include "../../opossum_common/app_interface.h"
#include "../../opossum_common/board_config.h"
#include "ASSERV/asserv_loop.h"

void App_Init(void) {
    // Initialisations spécifiques au CPU1

#if USE_ASSERV
    asserv_loop_init();
#endif
}

void App_Loop(void) {
#if USE_ASSERV
    asserv_loop_update();
#endif
}
