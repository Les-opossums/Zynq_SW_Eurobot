#ifndef DRIVER_WS2812B_H
#define DRIVER_WS2812B_H

#include "xil_types.h"

// --- Structure d'une couleur LED (tampon logiciel) ---
typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} led_color_t;

// --- Structure de Contexte du Driver WS2812B ---
typedef struct {
    u32 base_addr;              // Adresse de base AXI du périphérique WS2812B
    u32 num_leds;                // Nombre de LEDs sur le bandeau
    led_color_t *led_buffer;     // Tampon logiciel des couleurs (alloué par l'appelant)

    u32 refresh_period_ms;       // Période de rafraîchissement matériel (push vers l'AXI)
    u32 last_refresh_ms;         // Horodatage du dernier rafraîchissement matériel
    volatile u8 dirty;           // Flag : le tampon a été modifié depuis le dernier push (optionnel, optimisation)
} ws2812b_context_t;

// --- Prototypes standards pour l'IO_Manager ---
int  WS2812B_Init(void *instance);
void WS2812B_Update(void *instance);

// --- API bas niveau (utilisée par la couche applicative) ---
void WS2812B_SetPixel(ws2812b_context_t *ctx, u32 index, uint8_t r, uint8_t g, uint8_t b);
void WS2812B_SetAll(ws2812b_context_t *ctx, uint8_t r, uint8_t g, uint8_t b);
void WS2812B_Clear(ws2812b_context_t *ctx);
void WS2812B_Force_Refresh(ws2812b_context_t *ctx); // pousse immédiatement le tampon vers l'AXI, sans attendre le timer

#endif /* DRIVER_WS2812B_H */