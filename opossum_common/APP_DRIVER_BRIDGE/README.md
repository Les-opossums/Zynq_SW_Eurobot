# APP_DRIVER_BRIDGE — Pilotage générique des drivers IO_Manager

Ce dossier contient le pont entre l'[interpréteur de commandes](../APP_COM/README.md) et l'API dynamique de l'[IO_MANAGER](../IO_MANAGER/README.md) : activer, désactiver, ou lister l'état de n'importe quel périphérique de `IO_DEVICE_TABLE`, par son nom, sans avoir à écrire de commande dédiée par driver.

## Fichiers

* **`driver_commands.c` / `driver_commands.h`** — Trois commandes :
  * `Driver_Enable_Cmd()` → commande `DRVEN "NOM"`
  * `Driver_Disable_Cmd()` → commande `DRVDIS "NOM"`
  * `Driver_List_Cmd()` → commande `DRVLIST` (aucun paramètre)

Le nom (`Get_Param_String`, cf. [`interpreteur.h`](../APP_COM/README.md)) doit être passé entre guillemets et correspond au champ `.name` de `IO_DEVICE_TABLE` (cf. [`IO_config.h`](../IO_config.h)) : `GPIO_PS`, `WS2812B`, `UART_COMM`, `CAN_MOTORS`, `ETHERNET`, `IMU_BNO085`, `UART_FEETECH`, `FEETECH`. La comparaison est insensible à la casse.

## Exemples

```
DRVLIST              -> imprime l'etat ON/OFF de tous les drivers de ce coeur
DRVDIS "FEETECH"     -> coupe le protocole FEETECH (arret propre : deinit())
DRVEN  "FEETECH"     -> le reactive (reinit() du hardware)
DRVDIS "WS2812B"     -> eteint le bandeau LED
```

## Pourquoi par nom et pas par type ?

`IO_Manager_SetDeviceState(dev_type_t type, ...)` (l'API historique de l'IO_Manager) agit sur **tous** les périphériques partageant un même `dev_type_t` — or plusieurs périphériques peuvent partager un type générique (ex. `UART_COMM` et `UART_FEETECH` partagent tous les deux `DEV_TYPE_UART_PS`). `IO_Manager_SetDeviceStateByName()` cible un seul périphérique précis, sans ambiguïté, et vérifie en plus qu'il est bien géré par le cœur courant avant d'agir (pas d'appel `init()`/`deinit()` sur une instance jamais initialisée côté de l'autre cœur).

## Voir aussi

* [IO_MANAGER](../IO_MANAGER/README.md) — `IO_Manager_SetDeviceStateByName()` / `IO_Manager_PrintDeviceList()`
* [APP_COM](../APP_COM/README.md) — interpréteur et table des commandes
* [SYSTEM_MANAGER](../SYSTEM_MANAGER/README.md) — commande REBOOT (même famille : pilotage système générique)
* [opossum_common](../README.md) — vue d'ensemble de l'architecture
