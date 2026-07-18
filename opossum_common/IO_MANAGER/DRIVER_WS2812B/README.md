# Driver WS2812B (Bandeau LED)

Ce sous-dossier contient le pilote permettant de contrôler des bandeaux de LEDs adressables WS2812B (ou NeoPixel) depuis le processeur (PS), en s'appuyant sur un périphérique AXI personnalisé implémenté dans la logique programmable (PL).

## Architecture du Pilote

Le pilote WS2812B repose sur une logique de **tampon logiciel** (virtuellement assimilable à du double buffering) :
1.  **Tampon Logiciel (Software Buffer) :** Un tableau de structures `led_color_t` est alloué en RAM. La couche applicative modifie les couleurs de ce tampon de manière asynchrone. Cela permet de préparer une animation complexe (boucles, calculs) sans générer de trafic permanent et lent sur le bus AXI.
2.  **Rafraîchissement Matériel :** Périodiquement, la fonction de mise à jour lit l'intégralité du tampon logiciel et pousse les couleurs formatées vers les registres du composant matériel AXI.

## Interface Matérielle (AXI IP)

Le driver interagit avec l'IP AXI personnalisée en écrivant directement dans sa plage d'adressage mémoire. 
*   Chaque LED correspond à un registre de 32 bits (offset de 4 octets par LED).
*   Le format de couleur attendu par l'IP matérielle est un mot de 32 bits encodé sous l'ordre spécifique : `(Bleu << 16) | (Rouge << 8) | Vert`. Le pilote se charge automatiquement de ce décalage de bits lors du transfert physique.

## Interface Standard (`IO_MANAGER`)

Le driver s'insère nativement dans le flux de l'`IO_Manager` grâce à deux fonctions :
*   `int WS2812B_Init(void *instance)` : Initialise le tampon logiciel au noir complet (extinction des LEDs), force le flag `dirty` à 1 pour déclencher l'allumage physique, et remet à zéro l'horodatage.
*   `void WS2812B_Update(void *instance)` : Appelée par la boucle principale, elle compare l'horloge milliseconde globale (`Timer_ms1`) avec le dernier rafraîchissement. Si le délai configuré (`refresh_period_ms`) est écoulé, le tampon est poussé vers le FPGA.

## API Applicative (Dessin)

Ces fonctions permettent de dessiner sur le tampon depuis le code applicatif :
*   `void WS2812B_SetPixel(ctx, index, r, g, b)` : Modifie la couleur d'une LED spécifique (en vérifiant que l'index ne dépasse pas le nombre maximum de LEDs) et lève le flag `dirty`.
*   `void WS2812B_SetAll(ctx, r, g, b)` : Applique une couleur uniforme à l'ensemble du bandeau.
*   `void WS2812B_Clear(ctx)` : Raccourci pour éteindre toutes les LEDs (SetAll à 0, 0, 0).
*   `void WS2812B_Force_Refresh(ctx)` : Pousse immédiatement le tampon logiciel vers le bus AXI sans attendre la prochaine échéance du timer. Très utile pour des flashs très courts et réactifs.

## Dépendances Externes

Pour que la mise à jour périodique non-bloquante fonctionne, ce driver s'appuie sur une variable globale `extern volatile u32 Timer_ms1`. Le système doit garantir que cette variable est bien incrémentée toutes les millisecondes (via un timer matériel de base de temps, par exemple).