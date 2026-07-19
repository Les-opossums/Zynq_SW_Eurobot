# Driver FEETECH (servos + pompes des pinces)

Ce dossier contient le pilote du bus série half-duplex FEETECH utilisé pour piloter les servomoteurs (STS3215, SCS0009) et les cartes pompes/électrovannes des pinces du robot. Il est le pendant "bas niveau" de la logique applicative des pinces, qui vit elle dans [`opossum_core1/src/APP_ACTIONNEURS`](../../../opossum_core1/src/APP_ACTIONNEURS/README.md) (CPU0 uniquement).

## Architecture du module

Contrairement aux autres drivers, celui-ci se décompose en **deux devices `IO_MANAGER` séparés** :

1. **Transport UART1** : ne réinvente rien, réutilise directement le driver générique [`DRIVER_UART_PS`](../DRIVER_UART_PS/README.md) (`UART_PS_Init`/`UART_PS_Update`) sur une seconde instance UART PS (`UartFeetech_Ctx`, `XPAR_XUARTPS_1_DEVICE_ID`), distincte de `UART_COMM`.
2. **Protocole FEETECH (`feetech_io.c` / `feetech_io.h`)** : machine à états (`FEETECH_IO_Update`) qui construit/envoie les trames (checksum, endianness STS vs SCS), pilote la broche de direction du bus half-duplex (AXI GPIO dédiée), et distribue les réponses aux commandes en attente.

Ces deux devices apparaissent comme deux entrées distinctes dans `IO_DEVICE_TABLE` (cf. [IO_config.h](../../IO_config.h)), toutes deux `CORE_CPU0` : `UART_FEETECH` (transport) puis `FEETECH` (protocole).

## Matériel

* **UART1** (`XPAR_XUARTPS_1_DEVICE_ID`), débit configuré via `UART_FEETECH_BAUDRATE` dans `IO_config.h` (1 000 000 bauds par défaut, baud usine des STS3215/SCS0009 — à ajuster si les servos sont reconfigurés).
* **Direction du bus** : AXI GPIO dédiée (`XPAR_AXI_GPIO_6_DEVICE_ID`, canal 1) — `FEETECH_DIR_TX`/`FEETECH_DIR_RX` dans `feetech_io.h` si jamais le sens du buffer matériel doit être inversé.

## Deux protocoles supportés

| Protocole | Endianness | Servos typiques | Suffixe API |
|---|---|---|---|
| STS | petit-boutiste | STS3215 | *(aucun)* — `PutFEETECH`, `GetFEETECH`... |
| SCS | grand-boutiste (2 octets) | SCS0009 | `_SCS` — `PutFEETECH_SCS`, `GetFEETECH_SCS`... |

## API applicative

* `PutFEETECH(id, reg, data)` / `PutFEETECH_SCS(...)` : écriture non bloquante (mise en file, exécutée en tâche de fond par `FEETECH_IO_Update`).
* `PutFEETECH_Ext_Done(id, reg, data, &done)` / `..._SCS` : comme ci-dessus, avec un flag `done` levé quand la commande est traitée (permet d'enchaîner des étapes dans une machine à états sans bloquer).
* `PutFEETECH_Wait(id, reg, data)` / `GetFEETECH_Wait(id, reg)` : variantes **bloquantes** (boucle sur `FEETECH_IO_Update` en interne) — **interdites en fonctionnement normal du robot**, réservées au debug (cf. commentaires dans `feetech_io.h`).
* `GetFEETECH(id, reg, &out)` / `GetFEETECH_Ext_Done(...)` / `GetFEETECH_Ext_Done_With_Status(...)` : lecture d'un registre, avec ou sans code de statut détaillé (`FEETECH_STATUS_OK/TIMEOUT/CHKSUM_ERROR/UNSUPORTED_CMD`).
* `FEETECH_All_Cmd_Done()` : indique si la file de commandes est entièrement vidée.
* `RegisterLenFEETECH(addr)` : donne la largeur (1 ou 2 octets) d'un registre de la table mémoire du servo/carte pompe.

La table des registres (position, vitesse, couple, courant des pompes `PUMP_CMD_1/2`, électrovannes `VALVE_CMD_1/2`...) est définie en tête de `feetech_io.h`.

## Fiabilité

Chaque commande est réessayée jusqu'à `FEETECH_CMD_NB_MAX_TRY_SEND` fois (timeout ou erreur de checksum) avant d'abandonner. La file de commandes (`Liste_Command_FEETECH`, `FEETECH_CMD_LIST_SIZE` entrées) est un anneau FIFO classique.

## Debug

Prints de diagnostic (timeouts, erreurs de checksum, retries) désactivés par défaut — décommenter `#define FEETECH_DEBUG` dans `feetech_io.h`.

## Voir aussi

* [Actions pinces (APP_ACTIONNEURS)](../../../opossum_core1/src/APP_ACTIONNEURS/README.md) — machine à états des pinces, seule consommatrice de cette API
* [DRIVER_UART_PS](../DRIVER_UART_PS/README.md) — driver de transport réutilisé
* [IO_MANAGER](../README.md) — table des périphériques et cycle de vie générique
* [APP_DRIVER_BRIDGE](../../APP_DRIVER_BRIDGE/README.md) — commandes `DRVEN "FEETECH"` / `DRVDIS "FEETECH"` / `DRVLIST`
* [opossum_common](../../README.md) — vue d'ensemble de l'architecture
