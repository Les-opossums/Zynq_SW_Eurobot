# PLATFORM — Initialisation bas niveau (BSP standalone)

Ce dossier contient la couche d'initialisation "plateforme" générée/adaptée du template standalone Xilinx, appelée en tout premier dans `main()` (cf. [`main.c`](../README.md)), avant même l'[IPC_MANAGER](../IPC_MANAGER/README.md) et l'[IRQ_MANAGER](../IRQ_MANAGER/README.md).

## Fichiers

* **`platform.c` / `platform.h`** — `init_platform()` / `cleanup_platform()` : initialisation minimale du BSP standalone (caches, etc.), boilerplate Xilinx standard.
* **`platform_config.h`** — configuration de plateforme (générée par Vitis selon le hardware exporté).

## Voir aussi

* [opossum_common](../README.md) — vue d'ensemble de l'architecture, séquence de démarrage complète dans `main.c`
* [IRQ_MANAGER](../IRQ_MANAGER/README.md) — initialisé juste après
* [IPC_MANAGER](../IPC_MANAGER/README.md) — initialisé juste après
