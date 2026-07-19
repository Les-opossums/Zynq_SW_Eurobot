# APP_ASSERV_BRIDGE — Commandes d'asservissement (CPU0 → CPU1 via IPC)

Ce dossier contient le "pont" entre l'[interpréteur de commandes](../APP_COM/README.md) (UART/Ethernet, reçu côté CPU0) et la boucle d'asservissement/odométrie/Kalman qui tourne sur CPU1 (cf. [`opossum_core2`](../../opossum_core2/README.md)). Toutes les commandes ici sont exécutées sur CPU0 mais relaient leurs paramètres à CPU1 via l'[IPC_MANAGER](../IPC_MANAGER/README.md) — CPU0 ne fait jamais de calcul d'asservissement lui-même.

## Fichiers

* **`asserv_commands.c` / `asserv_commands.h`** — Les handlers de commandes (`uint8_t Xxx_Cmd(void)`), enregistrés dans [`command_list.c`](../APP_COM/README.md). Chaque commande lit ses paramètres via `Get_Param_*` (cf. [`interpreteur.h`](../APP_COM/README.md)) puis les envoie à CPU1 via `SEND_FIELD`/`IPC_SendToOtherCore` (macros de l'[IPC_MANAGER](../IPC_MANAGER/README.md)).
* **`robot_messages.h`** — Structures de payload Ethernet (`eth_payload_odom_t`, `eth_payload_imu_t`, `eth_payload_heartbeat_t`, `eth_payload_robot_state_t`) envoyées au Raspberry Pi. **Doit rester synchronisé** avec le code ROS 2 côté Pi, comme [`ETH_protocol.h`](../ETH_protocol.h).

## Commandes principales

| Commande | Rôle |
|---|---|
| `MOVE`, `MOVESEX` | Consigne de position (cible unique ou séquence de points) |
| `SPEED`, `ASPEED` | Consigne de vitesse (repère robot / absolu) |
| `FREE`, `BLOCK` | Libère / bloque l'asservissement moteur |
| `VMAX`, `VTMAX`, `AMAX` | Limites de vitesse/accélération |
| `SET`, `SET0` | Repositionnement (recalage de la position estimée) |
| `SETLIDAR`, `SETCAMERA1/2/3` | Observations externes injectées dans le filtre de Kalman |
| `LIDARNOISE` | Bruit de mesure du lidar (matrice R du Kalman) |
| `ENKALMAN` | Active/désactive les corrections lidar/caméra du Kalman |
| `ODOSPACING` | Entraxe des roues (calibration odométrie) |
| `GETPOS`, `GETODO`, `ASSERVDONE` | Lecture d'état (position estimée, odométrie brute, mouvement terminé) |
| `PDE` | Active l'envoi périodique de la position (télémétrie) |
| `PWM` | Commande manuelle directe des ESC (dérogation, debug) |
| `CALIBFF`, `SPEEDTEST` | Calibration feed-forward roues / test de vitesse minuté |

## Boucles à appeler depuis `App_Loop()` (CPU0)

* `Print_Position_loop()` — envoi périodique de la position/télémétrie si activé par `PDE`.
* `Speed_Timed_Loop()`, `Move_Seq_Loop()` — suivi des commandes temporisées / séquences de points.

## Voir aussi

* [ASSERV (CPU1)](../../opossum_core2/src/ASSERV/README.md) — logique de contrôle qui reçoit ces commandes
* [KALMAN (CPU1)](../../opossum_core2/src/KALMAN/README.md) — filtre alimenté par SETLIDAR/SETCAMERA*/LIDARNOISE/ENKALMAN
* [IPC_MANAGER](../IPC_MANAGER/README.md) — mécanisme de transfert CPU0 → CPU1
* [APP_COM](../APP_COM/README.md) — interpréteur et table des commandes
* [opossum_common](../README.md) — vue d'ensemble de l'architecture
