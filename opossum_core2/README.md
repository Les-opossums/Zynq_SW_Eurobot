# opossum_core2 — Projet applicatif CPU1

Ce projet Vitis est compilé pour **CPU1** (`ps7_cortexa9_1`, `THIS_CORE_ID=1`) et joue le rôle d'**esclave** au démarrage : il est réveillé par CPU0 (`opossum_core1`) après que celui-ci a terminé sa propre initialisation (cf. `main.c` dans [opossum_common](../opossum_common/README.md)).

## Rôle de CPU1

CPU1 est entièrement dédié au **temps réel de l'asservissement** : odométrie, fusion de capteurs (Kalman), génération de trajectoire, PID de vitesse et pilotage moteur — cf. [ASSERV](src/ASSERV/README.md) et [KALMAN](src/KALMAN/README.md). Il ne possède directement aucun périphérique de communication externe (pas d'UART console, pas d'Ethernet) : toutes ses consignes viennent de CPU0 via l'[IPC_MANAGER](../opossum_common/IPC_MANAGER/README.md) (cf. [APP_ASSERV_BRIDGE](../opossum_common/APP_ASSERV_BRIDGE/README.md)), et son seul périphérique matériel propre est le bus CAN moteurs (cf. [DRIVER_CAN](../opossum_common/IO_MANAGER/DRIVER_CAN/README.md), [APP_MOTORS](../opossum_common/APP_MOTORS/README.md)).

## Fichiers propres à ce projet

Le code partagé vit dans [`opossum_common`](../opossum_common/README.md). Seul le code spécifique à CPU1 vit ici :

* **`src/core1_loop.c`** — implémente `App_Init()`/`App_Loop()` (cf. [`app_interface.h`](../opossum_common/app_interface.h)), qui se contentent d'appeler `asserv_loop_init()`/`asserv_loop_update()` — cf. [ASSERV](src/ASSERV/README.md).
* **`src/ASSERV/`** — odométrie, génération de trajectoire, contraintes vitesse/accélération, PID, calcul PWM. Cf. [README dédié](src/ASSERV/README.md).
* **`src/KALMAN/`** — filtre de Kalman étendu (fusion odométrie/lidar/caméras/IMU) et sa FIFO de repropagation. Cf. [README dédié](src/KALMAN/README.md).

## Voir aussi

* [opossum_core1](../opossum_core1/README.md) — projet CPU0 (maître, communication, actionneurs)
* [opossum_common](../opossum_common/README.md) — vue d'ensemble de l'architecture partagée
* [README racine](../README.md) — mise en place du projet sous Vitis
