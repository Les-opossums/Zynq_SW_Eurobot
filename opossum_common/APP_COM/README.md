# APP_COM — Interpréteur de commandes

Ce dossier contient l'interpréteur de commandes texte partagé par les deux cœurs, ainsi que les deux "boucles" qui l'alimentent en octets (UART et Ethernet). C'est le point d'entrée unique de toutes les commandes pilotées depuis l'extérieur (console série, Raspberry Pi).

## Fichiers

* **`interpreteur.c` / `interpreteur.h`** — Le cœur du parseur. `Interp(ctx, c)` consomme les caractères un par un dans un contexte de parsing (`interp_ctx_t`) et déclenche la commande correspondante une fois la ligne terminée (`\n`). Fournit aussi les fonctions de lecture de paramètres utilisées par toutes les commandes applicatives :
  * `Get_Param_u32(u32 *out)` / `Get_Param_x32` (hexa) / `Get_Param_Float(float *out)` — paramètres numériques espacés.
  * `Get_Param_String(char dest[], u8 max_len)` — paramètre texte, **entre guillemets** (`"..."`).
  * Chaque fonction retourne un code d'erreur (`PARAM_ERROR_CODE`, etc. — cf. `interpreteur.h`) que les commandes remontent directement.
* **`command_list.c` / `command_list.h`** — La table `Command_List[]` associant un nom de commande (chaîne) à sa fonction (`uint8_t (*)(void)`). **C'est le fichier à modifier pour enregistrer une nouvelle commande.** Compilé pour les deux cœurs (CPU0 et CPU1) : les commandes qui n'existent que sur un seul cœur (ex. les actions FEETECH, propres à CPU0) doivent être encadrées par `#if THIS_CORE_ID == CORE_ID_CPU0 ... #endif` pour ne pas casser la compilation de l'autre projet.
* **`com_interpreter_loop.c` / `com_interpreter_loop.h`** — Boucle CPU0 : draine les octets reçus sur `UART_COMM` (cf. [DRIVER_UART_PS](../IO_MANAGER/DRIVER_UART_PS/README.md)) et les pousse dans `Interp()`. Appelée depuis `App_Loop()` (cf. [`core0_loop.c`](../../opossum_core1/README.md)). Expose aussi `Com_Interpreter_PrintTiming()` (instrumentation, cf. [TIMER_MANAGER](../TIMER_MANAGER/README.md)).
* **`eth_interpreter_bridge.c` / `eth_interpreter_bridge.h`** — Pont entre le driver [Ethernet](../IO_MANAGER/DRIVER_ETH/README.md) et l'interpréteur : les commandes texte reçues sur le canal `RAW_CMD`/`CMD_GENERIC` repassent par `Interp()` (même logique que l'UART, avec son propre `interp_ctx_t` pour ne pas mélanger une commande partielle UART avec une commande partielle Ethernet) ; les commandes structurées (binaires — position, lidar, caméras, block/free) sont envoyées directement à CORE1 via l'[IPC_MANAGER](../IPC_MANAGER/README.md), sans repasser par le parseur ASCII.

## Ajouter une nouvelle commande

1. Écrire la fonction `uint8_t Ma_Commande_Cmd(void)` quelque part (dans un fichier applicatif dédié — voir [APP_ASSERV_BRIDGE](../APP_ASSERV_BRIDGE/README.md), [APP_DRIVER_BRIDGE](../APP_DRIVER_BRIDGE/README.md) ou [APP_ACTIONNEURS](../../opossum_core1/src/APP_ACTIONNEURS/README.md) pour des exemples), en lisant ses paramètres via `Get_Param_*` et en retournant `0` en cas de succès.
2. L'inclure et l'ajouter dans `Command_List[]` (`command_list.c`), avec le nom qui sera tapé sur la console (ex. `{ "MACOMMANDE", Ma_Commande_Cmd }`).
3. Si la commande n'a de sens que sur un seul cœur, l'encadrer par `#if THIS_CORE_ID == CORE_ID_CPU0` (ou `CPU1`) — cf. [CORE_ID](../CORE_ID/README.md).

## Voir aussi

* [APP_ASSERV_BRIDGE](../APP_ASSERV_BRIDGE/README.md) — commandes de mouvement/asservissement (MOVE, SPEED, VMAX...)
* [APP_DRIVER_BRIDGE](../APP_DRIVER_BRIDGE/README.md) — commandes génériques de pilotage des drivers (DRVEN/DRVDIS/DRVLIST)
* [APP_ACTIONNEURS](../../opossum_core1/src/APP_ACTIONNEURS/README.md) — commandes des pinces FEETECH (STSSEND, PINCE...)
* [SYSTEM_MANAGER](../SYSTEM_MANAGER/README.md) — commande REBOOT
* [IO_MANAGER / DRIVER_ETH](../IO_MANAGER/DRIVER_ETH/README.md) — source des commandes reçues par Ethernet
* [IO_MANAGER / DRIVER_UART_PS](../IO_MANAGER/DRIVER_UART_PS/README.md) — source des commandes reçues par UART
* [opossum_common](../README.md) — vue d'ensemble de l'architecture
