#include "com_interpreter_loop.h"
#include "interpreteur.h"
#include "../IO_config.h"

static interp_ctx_t Uart_Interp_Ctx = {0};

void Com_Interpreter_Update(void) {
    u8 c;
    while (UART_PS_GetByte(&UartComm_Ctx, &c)) {
        Interp(&Uart_Interp_Ctx, (char)c);
    }
}