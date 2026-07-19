# APP_TEST — Tests de bout en bout

Ce dossier contient de petits modules de test manuels, non indispensables au fonctionnement du robot, utilisés pour valider rapidement une chaîne matérielle complète pendant le développement. Non appelés automatiquement : à brancher/débrancher explicitement dans `App_Loop()` (cf. [`core0_loop.c`](../../opossum_core1/README.md)) selon le besoin.

## Fichiers

* **`led_au_test.c` / `led_au_test.h`** — `LED_AU_Test_Update()` : test de bout en bout GPIO PS → `AU_state` → bandeau LED ([DRIVER_WS2812B](../IO_MANAGER/DRIVER_WS2812B/README.md)). Bandeau vert fixe quand l'arrêt d'urgence est relâché, clignotant rouge (période 200 ms) quand il est appuyé. Utile pour valider rapidement que la chaîne GPIO PS → variable partagée → pilote LED fonctionne sans avoir à instrumenter tout le reste du robot.

## Voir aussi

* [DRIVER_PS_GPIO](../IO_MANAGER/DRIVER_PS_GPIO/README.md) — source de `AU_state`
* [DRIVER_WS2812B](../IO_MANAGER/DRIVER_WS2812B/README.md) — pilote du bandeau LED
* [opossum_common](../README.md) — vue d'ensemble de l'architecture
