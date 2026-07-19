#include "../../opossum_common/app_interface.h"
#include "../../opossum_common/IPC_MANAGER/IPC_manager.h"
#include "../../opossum_common/TIMER_MANAGER/timer_manager.h"
#include "../../opossum_common/TIMER_MANAGER/timing_stats.h"
#include "../../opossum_common/IO_config.h" // IO_Manager_PrintTiming
#include "APP_ASSERV_BRIDGE/asserv_commands.h"
#include "APP_COM/com_interpreter_loop.h"
#include "APP_COM/eth_interpreter_bridge.h"
#include "xil_printf.h"
// ... autres includes spécifiques ...

#if defined(TIMING_MEASURE)
static uint32_t last_timing_print_ms;
#endif

void App_Init(void) {
    // Initialisations spécifiques au CPU0

    // Enregistre le handler de commandes Ethernet aupres du driver ETH.
    // A appeler apres IO_Manager_Init() (qui a deja appele ETH_IO_Init()/
    // eth_driver_init()), mais avant que la boucle principale ne commence
    // a "poller" les trames entrantes (ETH_IO_Update()).
    ETH_Interpreter_Bridge_Init();
}

void App_Loop(void) {
    Com_Interpreter_Update();
    Print_Position_loop();

#if defined(TIMING_MEASURE)
    // Marge temps-reel CPU0 : peripheriques IO_Manager (dont le poll BNO085)
    // + reception/interpretation UART. Impression toutes les ~1s.
    if (ts_trigger_ms(1000U, &last_timing_print_ms)) {
        IO_Manager_PrintTiming();
        Com_Interpreter_PrintTiming();
    }
#endif
}
