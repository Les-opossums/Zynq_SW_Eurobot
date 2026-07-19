# SYSTEM_MANAGER — Redémarrage logiciel

Ce dossier contient les fonctions de contrôle système "globales" (par opposition aux périphériques individuels gérés par l'[IO_MANAGER](../IO_MANAGER/README.md)). Pour l'instant : un seul mécanisme, le redémarrage logiciel complet du Zynq.

## Fichiers

* **`system_reset.c` / `system_reset.h`**

## `System_Reboot()`

Déclenche un reset "chaud" complet du Processing System via le registre matériel **SLCR `PSS_RST_CTRL`** (Zynq-7000 TRM, UG585 §B.28, adresse fixe `0xF8000000 + 0x200`, indépendante du block design donc stable entre régénérations de BSP). Équivalent logiciel d'un appui sur le bouton reset `PS_SRST_B` :

* Les **deux cœurs** (CPU0 + CPU1) redémarrent, ainsi que le cache, la MMU et tous les périphériques PS.
* Le BootROM puis le FSBL s'exécutent à nouveau comme à la mise sous tension.
* La fonction ne retourne jamais.

Utilisable depuis n'importe quel cœur (le registre SLCR est une ressource matérielle partagée, pas un état propre à un cœur), mais en pratique déclenché depuis CPU0 via la commande `REBOOT` (cf. [APP_COM](../APP_COM/README.md)).

## Commande associée

```
REBOOT   -> reset PS complet, ne revient jamais
```

Enregistrée dans [`command_list.c`](../APP_COM/README.md) via `Reboot_Cmd()`.

## Voir aussi

* [APP_COM](../APP_COM/README.md) — table des commandes (`REBOOT`)
* [APP_DRIVER_BRIDGE](../APP_DRIVER_BRIDGE/README.md) — pilotage générique des drivers (même esprit, granularité périphérique plutôt que système entier)
* [opossum_common](../README.md) — vue d'ensemble de l'architecture
