# Driver Ethernet (UDP bare-metal, lwIP)

Ce dossier contient le pilote Ethernet utilisé pour la communication entre le Zynq (CPU0) et le Raspberry Pi (ROS 2). Il s'appuie sur la pile **lwIP** (mode raw/NO_SYS) fournie par le BSP Xilinx pour l'EMAC du Processing System, avec un protocole applicatif maison au-dessus (framing + CRC16 + dispatch par type de message).

## Architecture du module

Deux couches, comme pour la plupart des drivers de ce projet :

* **Couche transport (`ETH_driver.c` / `ETH_driver.h`)** : initialise lwIP/l'EMAC (`eth_driver_init`), poll les trames entrantes et les timers internes de lwIP (`eth_driver_poll`), gère l'envoi de trames applicatives (`eth_send_frame`), un `printf`-like non bloquant par UDP (`eth_printf`), et expose les statistiques bas niveau (`eth_driver_get_stats`).
* **Surcouche `IO_MANAGER` (`driver_eth_io.c` / `driver_eth_io.h`)** : encapsule le tout sous la forme standard `eth_io_context_t` + `ETH_IO_Init`/`ETH_IO_Update` attendue par la table de périphériques (cf. [IO_MANAGER](../README.md)).

Le protocole applicatif (types de messages, ports UDP, format de trame) est défini dans [`opossum_common/ETH_protocol.h`](../../ETH_protocol.h) — **ce fichier doit rester synchronisé avec le code Raspberry Pi/ROS 2**, exactement comme [`robot_messages.h`](../../APP_ASSERV_BRIDGE/robot_messages.h).

## Canaux UDP

| Canal | Port | Sens | Framé | Usage |
|---|---|---|---|---|
| `DEBUG` | 5000 | Zynq → Pi | oui | `eth_printf`, texte libre |
| `TELEMETRY` | 5001 | Zynq → Pi | oui | Odométrie, IMU, état moteurs/robot, heartbeat |
| `CMD` | 5002 | Pi → Zynq | oui | Commandes structurées (position, lidar, caméras, block/free...) |
| `RAW_CMD` | 5003 | Pi → Zynq | non | Commandes texte brutes (mêmes commandes que l'UART, debug humain) |

Chaque trame framée a un en-tête `eth_frame_header_t` (14 octets : magic, version, type, séquence, timestamp µs, longueur payload, CRC16).

## Réception des commandes

Le dispatch des commandes entrantes ne vit pas dans ce dossier : voir [`APP_COM/eth_interpreter_bridge.c`](../../APP_COM/eth_interpreter_bridge.c), qui enregistre son handler via `eth_driver_set_cmd_handler()` et route les commandes texte vers l'interpréteur habituel, les commandes structurées directement vers CORE1 via l'[IPC_MANAGER](../../IPC_MANAGER/README.md).

## Debug

Les prints de diagnostic d'init (`ETH_IO_DEBUG`/`ETH_IO_LOG`) sont désactivés par défaut — décommenter `#define ETH_IO_DEBUG` dans `driver_eth_io.h` pour les réactiver. `eth_printf()` reste lui toujours actif (canal `DEBUG`), c'est l'outil de diagnostic "en vol" à privilégier plutôt qu'un print UART, puisqu'il est reçu côté Raspberry Pi.

## Régénération du BSP et patch PHY

Le fichier Xilinx BSP `xemacpsif_physpeed.c` (négociation de vitesse du PHY RTL8201F) est régénéré par Vitis à chaque "Re-generate BSP Sources" et écrase le patch forçant le lien à 100 Mbits (le PHY négocie sinon à tort 10 Mbits). Relancer `patch_phy_speed.sh` (à la racine de `Zynq_SW`) après chaque régénération.

## Voir aussi

* [IO_MANAGER](../README.md) — table des périphériques et cycle de vie générique
* [APP_COM](../../APP_COM/README.md) — interpréteur de commandes et pont Ethernet
* [IPC_MANAGER](../../IPC_MANAGER/README.md) — transfert des commandes structurées vers CORE1
* [opossum_common](../../README.md) — vue d'ensemble de l'architecture
