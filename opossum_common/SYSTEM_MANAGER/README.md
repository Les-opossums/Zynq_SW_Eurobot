# SYSTEM_MANAGER — Redémarrage logiciel

Ce dossier contient les fonctions de contrôle système "globales" (par opposition aux périphériques individuels gérés par l'[IO_MANAGER](../IO_MANAGER/README.md)). Pour l'instant : un seul mécanisme, le redémarrage logiciel complet du Zynq — notamment utilisé pour booter la nouvelle image après un update OTA (cf. [APP_FWUPDATE](../APP_FWUPDATE/README.md)).

## Fichiers

* **`system_reset.c` / `system_reset.h`**

## `System_Reboot()`

Déclenche un reset "chaud" complet du Processing System via le registre matériel **SLCR `PSS_RST_CTRL`** (Zynq-7000 TRM, UG585 §B.28, adresse fixe `0xF8000000 + 0x200`, indépendante du block design donc stable entre régénérations de BSP). Équivalent logiciel d'un appui sur le bouton reset `PS_SRST_B`. Séquence :

1. Déverrouillage des registres SLCR (clé `0xDF0D` dans `SLCR_UNLOCK`).
2. **Reset du contrôleur QSPI** (`LQSPI_RST_CTRL`, `0x230`) avant le reset PS. Indispensable après un `FWUPDATE` : l'appli a laissé le contrôleur QSPI en mode manuel (cf. [APP_FWUPDATE/qspi_flash.c](../APP_FWUPDATE/README.md)) ; sans ça, le BootROM peut ne pas relire proprement la flash au redémarrage.
3. `PSS_RST_CTRL` : reset PS complet — les **deux cœurs** (CPU0 + CPU1), le cache, la MMU et tous les périphériques PS redémarrent. La fonction ne retourne jamais.

Utilisable depuis n'importe quel cœur (registre SLCR = ressource matérielle partagée), en pratique déclenché depuis CPU0 via la commande `REBOOT`.

### ⚠️ Boot mode et soft reset (important)

Sur Zynq-7000, le **mode de boot (JTAG / QSPI) n'est échantillonné qu'au POR** (power-on reset). Un soft reset `PSS_RST_CTRL` **ne relit pas les straps** : le BootROM re-tourne dans le mode latché au dernier POR.

* Carte strappée **QSPI** → `System_Reboot()` recharge l'image depuis la flash (c'est ce qu'attend l'OTA).
* Carte strappée **JTAG** → `System_Reboot()` repart en attente JTAG (ne boote pas la QSPI).

Si après un `REBOOT` la carte ne redémarre que par un vrai **power-cycle (POR)** alors qu'elle est bien strappée QSPI, c'est un blocage connu du warm reset (état DDR/PL non réinitialisé par le soft reset). Le seul redémarrage 100 % fiable équivalent à un power-cycle est un vrai POR — déclenchable à distance via un GPIO (ex. du Raspberry) câblé sur `PS_POR_B`.

## Commande associée

```
REBOOT   -> reset PS complet, ne revient jamais
```

Enregistrée dans [`command_list.c`](../APP_COM/README.md) via `Reboot_Cmd()`. Envoyée automatiquement par [`zynq_ota.sh`](../../zynq_ota.sh) après un `+DONE`.

## Voir aussi

* [APP_FWUPDATE](../APP_FWUPDATE/README.md) — mise à jour OTA de la QSPI (utilise `REBOOT`)
* [APP_COM](../APP_COM/README.md) — table des commandes (`REBOOT`)
* [APP_DRIVER_BRIDGE](../APP_DRIVER_BRIDGE/README.md) — pilotage générique des drivers (granularité périphérique)
* [opossum_common](../README.md) — vue d'ensemble de l'architecture
