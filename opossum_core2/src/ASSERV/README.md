# ASSERV — Adaptateur Zynq de l'asservissement (CPU1)

L'odométrie, le Kalman, la génération de trajectoire, les contraintes et le PID sont dans la
bibliothèque C pur [opossum_control](../../../libs/opossum_control/README.md), commune au
firmware et à la simulation PC. Ce dossier ne contient que la traduction avec le matériel de
CPU1. Point d'entrée : `asserv_loop_init()` / `asserv_loop_update()`, appelées depuis
`App_Init()`/`App_Loop()` (cf. [`core1_loop.c`](../../README.md)).

## Fichiers

| Fichier | Rôle |
|---|---|
| `asserv.h` | En-tête agrégateur : [opossum_control](../../../libs/opossum_control/README.md) + [`c610_feedback.h`](../../../opossum_common/APP_MOTORS/README.md) + [`IO_config.h`](../../../opossum_common/README.md) + [`IPC_manager.h`](../../../opossum_common/IPC_MANAGER/README.md) + timers |
| `asserv_loop.c` / `.h` | Cadencement par `Timer_ms1`, lecture IPC (commandes, IMU, AU, laisse) et CAN (retours moteurs), envoi CAN, télémétrie IPC |
| `control_hal_zynq.c` | Services plateforme de la bibliothèque : `xil_printf`, remontée fiable de `motion_done` par IPC, chronométrage `TIMING_MEASURE` |

## Séquencement (`asserv_loop_update`)

* À **chaque** tour : `receive_commands()` lit les messages IPC de CORE0 et les passe à
  `control_apply_commands()` (jamais de retard sur une commande). Les observations caméra
  ne sont lues qu'une fois le Kalman initialisé.
* Tous les `CONTROL_FAST_PERIOD_MS` (1 ms) : lecture des RPM moteurs et de l'IMU, puis
  `control_fast_step()` (odométrie, prédiction + corrections Kalman, FIFO, fusion du cap IMU).
* Tous les `CONTROL_TICKS_PER_SLOW` pas rapides (10 ms) : `control_slow_step()` (trajectoire,
  contraintes, PID, **coupure sur `IPC_DATA->AU_state`**), envoi CAN, télémétrie vers CORE0
  (pose Kalman, vitesse odométrie, consigne contrainte).

## Instrumentation (`TIMING_MEASURE`)

Les étapes internes de la bibliothèque (`fast_*`, `slow_motion`, `slow_constr`, `slow_pid`)
sont chronométrées via `control_hal_zynq.c`, l'envoi CAN (`slow_can`), la télémétrie
(`slow_ipc`) et le pas lent complet (`slow_total`) ici ; impression toutes les secondes avec
le timing de l'[IO_MANAGER](../../../opossum_common/IO_MANAGER/README.md) — cf.
[TIMER_MANAGER / timing_stats.h](../../../opossum_common/TIMER_MANAGER/README.md).

## Voir aussi

* [opossum_control](../../../libs/opossum_control/README.md) — la logique d'asservissement et le [Kalman](../../../libs/opossum_control/KALMAN.md)
* [APP_MOTORS](../../../opossum_common/APP_MOTORS/README.md) — retour et commande des ESC C610
* [APP_ASSERV_BRIDGE](../../../opossum_common/APP_ASSERV_BRIDGE/README.md) — commandes envoyées depuis CPU0
* [IPC_MANAGER](../../../opossum_common/IPC_MANAGER/README.md) — mécanisme de réception/envoi
* [opossum_core2](../../README.md) — vue d'ensemble du projet CPU1
