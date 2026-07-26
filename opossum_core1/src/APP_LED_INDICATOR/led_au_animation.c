#include "led_au_animation.h"
#include "IO_config.h"
#include "IO_MANAGER/DRIVER_WS2812B/driver_ws2812b.h"
#include "TIMER_MANAGER/timer_manager.h"
#include "IPC_MANAGER/IPC_manager.h"   /* IPC_DATA, LOC_INIT_* */

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

/*
 * Indicateur de statut global du bandeau. Priorite :
 *   1. Arret d'urgence actif       -> animation rouge (LED_AU_Animation_Update).
 *   2. Init localisation en cours  -> gauge de chargement orange clignotante,
 *      remplie a hauteur de loc_init_progress (%).
 *   3. Robot pret (offset fige)    -> vert fixe, jusqu'a ce qu'on tire la laisse.
 *   4. Match lance / boot          -> bandeau eteint.
 * L'etat (loc_init_state / loc_init_progress) est calcule par CORE1 (fusion
 * heading dans asserv_loop.c) et lu ici via la memoire partagee.
 */
#define LED_INIT_BLINK_PERIOD_MS  500.0f  /* periode de clignotement de la gauge (~2 Hz) */
#define LED_ORANGE_R              255      /* teinte orange de la gauge */
#define LED_ORANGE_G              70
#define LED_READY_GREEN_G         180      /* intensite du vert "pret" */

void LED_Indicator_Update(void) {
    /* 1. Arret d'urgence prioritaire : animation rouge existante. */
    if (AU_state) {
        LED_AU_Animation_Update();
        return;
    }

    /* AU relache : on laisse l'animation AU faire son nettoyage de front
     * descendant (extinction propre + re-armement), puis on redessine l'etat
     * courant par-dessus (le push materiel n'a lieu qu'une fois, cf
     * WS2812B_Update). */
    LED_AU_Animation_Update();

    switch (IPC_DATA->loc_init_state) {

    case LOC_INIT_RUNNING: {
        /* Gauge de chargement orange clignotante. */
        float ph  = (float)((uint32_t)Timer_ms1 % (uint32_t)LED_INIT_BLINK_PERIOD_MS)
                    / LED_INIT_BLINK_PERIOD_MS;                 /* 0..1 */
        float tri = (ph < 0.5f) ? (ph * 2.0f) : (2.0f - ph * 2.0f); /* 0..1..0 */
        float b   = 0.25f + 0.75f * tri;                        /* luminosite 0.25..1.0 */

        uint32_t prog = IPC_DATA->loc_init_progress;            /* 0..100 */
        int lit = (int)((prog * (uint32_t)NBR_LED) / 100u);

        for (int i = 0; i < NBR_LED; i++) {
            if (i < lit) {
                WS2812B_SetPixel(&Ws2812b_Ctx, i,
                                 clamp_u8((int)((float)LED_ORANGE_R * b)),
                                 clamp_u8((int)((float)LED_ORANGE_G * b)),
                                 0);
            } else {
                WS2812B_SetPixel(&Ws2812b_Ctx, i, 0, 0, 0);
            }
        }
        break;
    }

    case LOC_INIT_READY:
        /* Robot pret : vert fixe jusqu'a ce qu'on tire la laisse. */
        WS2812B_SetAll(&Ws2812b_Ctx, 0, LED_READY_GREEN_G, 0);
        break;

    case LOC_INIT_IDLE:
    case LOC_INIT_DONE:
    default:
        /* Boot (IDLE) ou match lance apres laisse tiree (DONE) : bandeau eteint. */
        WS2812B_Clear(&Ws2812b_Ctx);
        break;
    }
}
