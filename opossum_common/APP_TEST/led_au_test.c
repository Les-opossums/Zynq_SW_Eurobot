#include "led_au_test.h"
#include "../IO_config.h"
#include "../IO_MANAGER/DRIVER_WS2812B/driver_ws2812b.h"
#include "../TIMER_MANAGER/timer_manager.h"

// Animation simple : clignote rouge tant que l'AU est appuyé,
// vert fixe sinon. Test de bout en bout : GPIO PS -> AU_state -> bandeau LED.
static u32 blink_timer_old = 0;
static u8  blink_on = 0;

void LED_AU_Test_Update(void) {
    if (AU_state == 1) {
        // AU appuyé : clignotement rouge, période 200ms
        if (Timer_ms1 - blink_timer_old > 200) {
            blink_timer_old = Timer_ms1;
            blink_on = !blink_on;
        }

        if (blink_on) {
            WS2812B_SetAll(&Ws2812b_Ctx, 0xFF, 0x00, 0x00);
        } else {
            WS2812B_SetAll(&Ws2812b_Ctx, 0x00, 0x00, 0x00);
        }
    } else {
        // AU relâché : vert fixe
        WS2812B_SetAll(&Ws2812b_Ctx, 0x00, 0xFF, 0x00);
    }
}