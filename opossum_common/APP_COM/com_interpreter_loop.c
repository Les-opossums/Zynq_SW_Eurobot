#include "com_interpreter_loop.h"
#include "interpreteur.h"
#include "../IO_config.h"

void Com_Interpreter_Update(void) {
    u8 c;
    // On traite tous les octets disponibles à chaque cycle
    while (UART_PS_GetByte(&UartComm_Ctx, &c)) {
        Interp((char)c);
    }
}