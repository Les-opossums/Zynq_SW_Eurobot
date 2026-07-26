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

/**
 * @brief Indicateur de statut global du bandeau WS2812B, a appeler a chaque
 * tour de App_Loop() (CPU0) a la place de LED_AU_Animation_Update(). Gere par
 * ordre de priorite : arret d'urgence (rouge), init de localisation en cours
 * (gauge orange clignotante selon loc_init_progress), robot pret (vert fixe),
 * puis extinction du bandeau une fois la laisse tiree / match lance.
 * Purement visuel : ne pilote aucune securite.
 */
void LED_Indicator_Update(void);

#endif // LED_AU_ANIMATION_H
