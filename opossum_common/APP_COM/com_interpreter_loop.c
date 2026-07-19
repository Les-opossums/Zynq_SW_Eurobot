#include "com_interpreter_loop.h"
#include "interpreteur.h"
#include "../IO_config.h"
#include "../TIMER_MANAGER/timing_stats.h"

static interp_ctx_t Uart_Interp_Ctx = {0};

#if defined(TIMING_MEASURE)
static TimingStats ComTiming;
#endif

void Com_Interpreter_Update(void) {
#if defined(TIMING_MEASURE)
    T_START(com_update);
#endif

    u8 c;
    while (UART_PS_GetByte(&UartComm_Ctx, &c)) {
        Interp(&Uart_Interp_Ctx, (char)c);
    }

#if defined(TIMING_MEASURE)
    T_STOP(com_update, ComTiming);
#endif
}

void Com_Interpreter_PrintTiming(void) {
#if defined(TIMING_MEASURE)
    ts_print_one("com_uart", &ComTiming);
#endif
}