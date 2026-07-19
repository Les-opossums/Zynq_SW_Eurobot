#include "driver_ws2812b.h"
#include "xil_io.h"

// Horloge milliseconde partagée, entretenue par la boucle 1ms (fast loop)
extern volatile u32 Timer_ms1;

// ==================================================================
// HELPER : écrit directement une couleur sur l'IP AXI
// ==================================================================
static void WS2812B_WritePixel_HW(ws2812b_context_t *ctx, u32 index, uint8_t r, uint8_t g, uint8_t b) {
    if (index < ctx->num_leds) {
        u32 addr = ctx->base_addr + (index * 4);
        // Ordre des couleurs de l'IP : Blue << 16 | Red << 8 | Green
        u32 color_data = (b << 16) | (r << 8) | g;
        Xil_Out32(addr, color_data);
    }
}

// ==================================================================
// HELPER : pousse tout le tampon logiciel vers l'IP AXI
// ==================================================================
static void WS2812B_Flush(ws2812b_context_t *ctx) {
    for (u32 i = 0; i < ctx->num_leds; i++) {
        WS2812B_WritePixel_HW(ctx, i, ctx->led_buffer[i].red, ctx->led_buffer[i].green, ctx->led_buffer[i].blue);
    }
    ctx->dirty = 0;
}

// ==================================================================
// 1. Initialisation du driver
// ==================================================================
int WS2812B_Init(void *instance) {
    ws2812b_context_t *ctx = (ws2812b_context_t *)instance;

    for (u32 i = 0; i < ctx->num_leds; i++) {
        ctx->led_buffer[i].red   = 0;
        ctx->led_buffer[i].green = 0;
        ctx->led_buffer[i].blue  = 0;
    }

    ctx->last_refresh_ms = 0;
    ctx->dirty = 1; // force un premier rafraîchissement au démarrage

    return XST_SUCCESS;
}

// ==================================================================
// 2. Mise à jour périodique (appelée par l'IO_Manager)
//    Se contente de pousser le tampon logiciel vers l'IP AXI au
//    rythme défini par refresh_period_ms. Aucune logique d'animation.
// ==================================================================
void WS2812B_Update(void *instance) {
    ws2812b_context_t *ctx = (ws2812b_context_t *)instance;

    if (Timer_ms1 - ctx->last_refresh_ms > ctx->refresh_period_ms) {
        ctx->last_refresh_ms = Timer_ms1;
        WS2812B_Flush(ctx);
    }
}

// ==================================================================
// 3. API bas niveau
// ==================================================================
void WS2812B_SetPixel(ws2812b_context_t *ctx, u32 index, uint8_t r, uint8_t g, uint8_t b) {
    if (index < ctx->num_leds) {
        ctx->led_buffer[index].red   = r;
        ctx->led_buffer[index].green = g;
        ctx->led_buffer[index].blue  = b;
        ctx->dirty = 1;
    }
}

void WS2812B_SetAll(ws2812b_context_t *ctx, uint8_t r, uint8_t g, uint8_t b) {
    for (u32 i = 0; i < ctx->num_leds; i++) {
        ctx->led_buffer[i].red   = r;
        ctx->led_buffer[i].green = g;
        ctx->led_buffer[i].blue  = b;
    }
    ctx->dirty = 1;
}

void WS2812B_Clear(ws2812b_context_t *ctx) {
    WS2812B_SetAll(ctx, 0, 0, 0);
}

void WS2812B_Force_Refresh(ws2812b_context_t *ctx) {
    WS2812B_Flush(ctx);
}