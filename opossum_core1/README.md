# opossum_core1 — Projet applicatif CPU0

Malgré son nom (hérité d'une numérotation historique), ce projet Vitis est compilé pour **CPU0** (`ps7_cortexa9_0`, `THIS_CORE_ID=0`) et joue le rôle de **maître** de la séquence de démarrage (cf. `main.c` dans [opossum_common](../opossum_common/README.md)) : il initialise ses propres périphériques en premier, puis réveille CPU1 (`opossum_core2`).

## Rôle de CPU0

* **Entrées/Sorties "IHM et capteurs"** : GPIO PS (arrêt d'urgence, laisse, switches), bandeau LED, IMU BNO085 — cf. [IO_MANAGER](../opossum_common/IO_MANAGER/README.md).
* **Communication avec l'extérieur** : console UART, liaison Ethernet UDP avec le Raspberry Pi — cf. [DRIVER_ETH](../opossum_common/IO_MANAGER/DRIVER_ETH/README.md), [APP_COM](../opossum_common/APP_COM/README.md).
* **Actionneurs** : bus FEETECH (servos + pompes des pinces) — cf. [DRIVER_FEETECH](../opossum_common/IO_MANAGER/DRIVER_FEETECH/README.md) et [APP_ACTIONNEURS](src/APP_ACTIONNEURS/README.md) ci-dessous.
* CPU0 **ne fait pas** de calcul d'asservissement/odométrie/Kalman — cela vit entièrement côté CPU1, cf. [opossum_core2](../opossum_core2/README.md). Les commandes de mouvement reçues par CPU0 (UART/Ethernet) sont relayées à CPU1 via l'[IPC_MANAGER](../opossum_common/IPC_MANAGER/README.md) (cf. [APP_ASSERV_BRIDGE](../opossum_common/APP_ASSERV_BRIDGE/README.md)).

## Fichiers propres à ce projet

Le code partagé entre les deux cœurs vit dans [`opossum_common`](../opossum_common/README.md) (dossier lié dans le projet Vitis, cf. étape 7 du [README racine](../README.md)). Seul le code **strictement spécifique à CPU0** vit ici :

* **`src/core0_loop.c`** — implémente `App_Init()`/`App_Loop()` (cf. [`app_interface.h`](../opossum_common/app_interface.h)) :
  * `App_Init()` : enregistre le handler de commandes Ethernet.
  * `App_Loop()` : boucle de l'interpréteur UART, télémétrie, mise à jour des pinces (`Init_Pinces_Loop()`/`pince_loop()`), coupure de sécurité des pinces sur front montant de l'arrêt d'urgence, impression périodique des temps d'exécution (si `TIMING_MEASURE` actif, cf. [TIMER_MANAGER](../opossum_common/TIMER_MANAGER/README.md)).
* **`src/APP_ACTIONNEURS/`** — logique applicative des pinces (machine à états), cf. [README dédié](src/APP_ACTIONNEURS/README.md).

## Voir aussi

* [opossum_core2](../opossum_core2/README.md) — projet CPU1 (asservissement, odométrie, Kalman)
* [opossum_common](../opossum_common/README.md) — vue d'ensemble de l'architecture partagée
* [README racine](../README.md) — mise en place du projet sous Vitis
