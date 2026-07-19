#ifndef LED_AU_ANIMATION_H
#define LED_AU_ANIMATION_H

/**
 * @brief A appeler a chaque tour de App_Loop() (CPU0). Anime le bandeau
 * WS2812B en fondu rouge + halo tournant tant que AU_state est actif, et
 * eteint proprement le ruban au relachement de l'arret d'urgence.
 * Purement visuel : ne pilote aucune securite (la coupure moteurs/pinces
 * se fait ailleurs, cf. asserv_loop.c et AU_pinces()).
 */
void LED_AU_Animation_Update(void);

#endif // LED_AU_ANIMATION_H
