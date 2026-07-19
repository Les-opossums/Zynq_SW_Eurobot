# ASSERV — Odométrie, trajectoire, PID et pilotage moteur (CPU1)

Ce dossier contient toute la boucle de contrôle temps réel de CPU1 : lecture de l'odométrie, génération/suivi de trajectoire, contraintes physiques, PID de vitesse, et transmission des commandes moteur. Point d'entrée : `asserv_loop_init()` / `asserv_loop_update()` (`asserv_loop.c`), appelées depuis `App_Init()`/`App_Loop()` (cf. [`core1_loop.c`](../../README.md)).

## Fichiers

| Fichier | Rôle |
|---|---|
| `asserv.h` | En-tête agrégateur : inclut tous les modules ci-dessous + [KALMAN](../KALMAN/README.md) + [`c610_feedback.h`](../../../opossum_common/APP_MOTORS/README.md) + [`IO_config.h`](../../../opossum_common/README.md) + [`IPC_manager.h`](../../../opossum_common/IPC_MANAGER/README.md) |
| `asserv_default.h` | Toutes les constantes par défaut (gains PID, limites de vitesse/accélération, entraxe, réduction moteur...) |
| `asserv_loop.c` / `.h` | Séquencement des boucles rapide/lente + réception des commandes IPC (`receive_commands()`) |
| `odometry.c` / `.h` | Intégration de l'odométrie brute à partir des RPM moteurs |
| `motion.c` / `.h` | Génération de trajectoire (mode position / vitesse / vitesse absolue / bloqué), état `motion_done` remonté à CPU0 |
| `speed_constrainer.c` / `.h` | Limiteurs de vitesse et d'accélération (rampes) |
| `pid_speed.c` / `.h` | PID de vitesse par roue |
| `PWM_Calculator.c` / `.h` | Calcul final des commandes ESC (`Asserv_PWM_calculator`) |

## Séquencement (`asserv_loop_update`)

Deux boucles à fréquences différentes, cadencées par `Timer_ms1` (cf. [TIMER_MANAGER](../../../opossum_common/TIMER_MANAGER/README.md)) :

* **Boucle rapide** (`fast_loop`, tous les `ODO_EVERY_MS` = 1 ms) : lecture des RPM moteurs ([APP_MOTORS](../../../opossum_common/APP_MOTORS/README.md)), intégration odométrique, prédiction + correction Kalman (odométrie puis IMU), poussée dans la FIFO Kalman, avancement d'une éventuelle repropagation en tâche de fond.
* **Boucle lente** (`slow_loop`, tous les `ASSERV_EVERY` = 10 cycles rapides, soit 10 ms) : moyenne de l'odométrie rapide, génération de trajectoire (`motion_step`), contraintes vitesse/accélération, calcul PID → commandes ESC, **coupure matérielle sur `IPC_DATA->AU_state`** (arrêt d'urgence, remise à zéro de l'intégrale du PID), transmission CAN, remontée de télémétrie vers CPU0 (position Kalman, vitesse odométrie, vitesse contrainte).

`receive_commands()` est appelée à **chaque** tour (avant même l'attente d'échéance), pour ne jamais retarder l'application d'une commande reçue de CPU0 — cf. [APP_ASSERV_BRIDGE](../../../opossum_common/APP_ASSERV_BRIDGE/README.md) côté émetteur.

## Instrumentation (`TIMING_MEASURE`)

Chaque étape des deux boucles est chronométrée individuellement (`ts_fast_*`, `ts_slow_*`) et imprimée toutes les secondes aux côtés du timing de l'[IO_MANAGER](../../../opossum_common/IO_MANAGER/README.md) — cf. [TIMER_MANAGER / timing_stats.h](../../../opossum_common/TIMER_MANAGER/README.md) pour l'interrupteur commun aux deux cœurs.

## Voir aussi

* [KALMAN](../KALMAN/README.md) — filtre de fusion consommé par ce module
* [APP_MOTORS](../../../opossum_common/APP_MOTORS/README.md) — retour et commande des ESC C610
* [APP_ASSERV_BRIDGE](../../../opossum_common/APP_ASSERV_BRIDGE/README.md) — commandes envoyées depuis CPU0
* [IPC_MANAGER](../../../opossum_common/IPC_MANAGER/README.md) — mécanisme de réception/envoi utilisé par `receive_commands()`
* [opossum_core2](../../README.md) — vue d'ensemble du projet CPU1
