#include "led_au_animation.h"
#include "IO_config.h"
#include "IO_MANAGER/DRIVER_WS2812B/driver_ws2812b.h"
#include "TIMER_MANAGER/timer_manager.h"

/*
 * Animation d'arret d'urgence : le ruban s'allume rouge LED par LED (une
 * barre qui se remplit du debut a la fin, chaque LED apparaissant avec un
 * fondu progressif), puis une fois arrivee au bout, s'eteint LED par LED
 * dans le meme ordre (chaque LED s'estompe progressivement vers le noir).
 * Le cycle (remplissage puis extinction) se repete en boucle tant que
 * l'arret d'urgence est actif.
 */

#define AU_ANIM_PEAK_RED       255    /* intensite d'une LED pleinement allumee (0-255) */
#define AU_ANIM_STEP_FILL_MS   20.0f  /* duree pour allumer une LED (fondu compris) */
#define AU_ANIM_STEP_EMPTY_MS  20.0f  /* duree pour eteindre une LED (fondu compris) */

static uint8_t  au_anim_was_active = 0;
static uint32_t au_anim_start_ms   = 0;

static uint8_t clamp_u8(int v) {
    if (v < 0)   return 0;
    if (v > 255) return 255;
    return (uint8_t)v;
}

void LED_AU_Animation_Update(void) {
    if (!AU_state) {
        if (au_anim_was_active) {
            /* Front descendant : arret d'urgence relache, on eteint proprement. */
            WS2812B_Clear(&Ws2812b_Ctx);
            au_anim_was_active = 0;
        }
        return;
    }

    if (!au_anim_was_active) {
        /* Front montant : on redemarre l'animation depuis une phase propre
         * (evite de reprendre en plein milieu d'un cycle si l'AU est
         * appuye/relache plusieurs fois de suite). */
        au_anim_start_ms   = (uint32_t)Timer_ms1;
        au_anim_was_active = 1;
    }

    const float elapsed_ms = (float)((uint32_t)Timer_ms1 - au_anim_start_ms);

    const float fill_total_ms  = (float)NBR_LED * AU_ANIM_STEP_FILL_MS;
    const float empty_total_ms = (float)NBR_LED * AU_ANIM_STEP_EMPTY_MS;
    const float cycle_total_ms = fill_total_ms + empty_total_ms;

    /* fmodf() maison (sans <math.h>) : reste de la division flottante. */
    float phase_ms = elapsed_ms - cycle_total_ms * (float)((uint32_t)(elapsed_ms / cycle_total_ms));

    if (phase_ms < fill_total_ms) {
        /* --- Phase 1 : remplissage, LED par LED, du debut vers la fin --- */
        const float progress   = phase_ms / AU_ANIM_STEP_FILL_MS; /* position flottante dans le ruban */
        const int   lit_count  = (int)progress;                   /* nombre de LED deja pleinement allumees */
        const float fade_frac  = progress - (float)lit_count;     /* 0..1 : fondu de la LED en cours d'allumage */

        for (int i = 0; i < NBR_LED; i++) {
            uint8_t red;
            if (i < lit_count) {
                red = AU_ANIM_PEAK_RED;
            } else if (i == lit_count) {
                red = clamp_u8((int)(fade_frac * (float)AU_ANIM_PEAK_RED));
            } else {
                red = 0;
            }
            WS2812B_SetPixel(&Ws2812b_Ctx, i, red, 0, 0);
        }
    } else {
        /* --- Phase 2 : extinction, LED par LED, dans le meme ordre --- */
        const float progress   = (phase_ms - fill_total_ms) / AU_ANIM_STEP_EMPTY_MS;
        const int   off_count  = (int)progress;                   /* nombre de LED deja eteintes */
        const float fade_frac  = progress - (float)off_count;     /* 0..1 : fondu de la LED en cours d'extinction */

        for (int i = 0; i < NBR_LED; i++) {
            uint8_t red;
            if (i < off_count) {
                red = 0;
            } else if (i == off_count) {
                red = clamp_u8((int)((1.0f - fade_frac) * (float)AU_ANIM_PEAK_RED));
            } else {
                red = AU_ANIM_PEAK_RED;
            }
            WS2812B_SetPixel(&Ws2812b_Ctx, i, red, 0, 0);
        }
    }
}
