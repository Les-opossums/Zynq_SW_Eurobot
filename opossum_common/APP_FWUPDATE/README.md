# APP_FWUPDATE — Mise à jour du firmware par liaison série (OTA)

Ce module permet de **reflasher la QSPI du Zynq à distance**, par la liaison série de commandes ([UART_COMM](../IO_MANAGER/DRIVER_UART_PS/README.md)), sans JTAG ni ouverture du robot. Le host (PC de dev ou **Raspberry Pi**) envoie un `BOOT.bin`, le Zynq l'écrit en QSPI, le vérifie, puis redémarre dessus.

Côté host : le script [`zynq_ota.sh`](../../zynq_ota.sh) (racine du dépôt). Côté firmware : ce module + la commande `FWUPDATE` enregistrée dans [`command_list.c`](../APP_COM/README.md) (**CPU0 uniquement** : console + QSPI côté CPU0).

## Fichiers

* **`qspi_flash.c` / `qspi_flash.h`** — accès bas niveau à la flash QSPI via `XQspiPs` (single-lane, adressage 3 octets). Commandes NOR génériques (Winbond/Micron/Spansion) : `WREN` (0x06), `SECTOR_ERASE` 64 Ko (0xD8), `PAGE_PROGRAM` 256 o (0x02), `READ` (0x03), polling `WIP`. API : `QspiFlash_Init / EraseRange / Write / Read`.
* **`fw_update.c` / `fw_update.h`** — la commande `FW_Update_Cmd()` : parsing, effacement, réception du flux, écriture QSPI, double vérification CRC32, sécurité anti-brick.

## Prérequis matériel : boot QSPI

Le tout n'a de sens que si la carte **boote depuis la QSPI** (straps de boot mode en QSPI, échantillonnés au POR). Sur Zynq-7000 le mode de boot n'est lu **qu'au power-on reset** : un reset logiciel ne le rechange pas. Une fois strappée QSPI, la carte démarre seule sur la flash au power-up — plus besoin de JTAG à chaque démarrage.

## Layout QSPI et sécurité anti-brick

Défini dans [`board_config.h`](../board_config.h) (source unique, à garder synchro avec `zynq_ota.sh`) :

| Offset | Contenu | Écrit par |
|---|---|---|
| `0x000000` | **Image primaire** (celle qu'on met à jour) | OTA (`send`) ou `jtag` |
| `QSPI_GOLDEN_OFFSET` (`0x400000`) | **Image golden** (secours, connue-bonne) | `golden` (JTAG, une fois) |

Trois couches protègent contre un update interrompu (coupure, câble, crash) :

1. **Golden Image Search du BootROM.** Si l'en-tête à l'offset 0 est invalide, le BootROM Zynq-7000 cherche automatiquement un en-tête valide **tous les 32 Ko en remontant** et boote la golden. La carte n'est donc jamais muette.
2. **Écriture atomique (en-tête en dernier).** Le **premier bloc** (en-tête de boot + début FSBL) est gardé en RAM et écrit **en tout dernier**, une fois tout le corps en flash et le CRC des données reçues vérifié. Tant que l'OTA n'est pas complet, l'offset 0 reste effacé → un plantage laisse le BootROM retomber sur la golden.
3. **Garde de taille.** L'OTA refuse toute image qui atteindrait la zone golden (`QSPI_UPDATE_MAX_SIZE`).

Comme le registre MultiBoot repart à 0 au POR, dès que l'image primaire est de nouveau valide la carte reboote dessus toute seule : **ça s'auto-répare**.

> La golden est ton filet : flashe-la **une seule fois** avec une version bien testée (via `./zynq_ota.sh golden`), et ne la touche plus. L'OTA normal ne l'écrase jamais.

## Protocole série

Transporté sur la même UART que l'interpréteur ([APP_COM](../APP_COM/README.md)). Pendant `FWUPDATE`, la boucle principale est bloquée : le firmware vide lui-même le buffer TX (`UART_PS_Update`) pour que les réponses partent tout de suite, et la réception s'appuie sur l'ISR RX. CRC32 = `zlib.crc32` (polynôme réfléchi `0xEDB88320`), transmis en **décimal**.

```
host -> "FWUPDATE <taille_dec> <crc32_dec>\r"
zynq -> "+READY <bloc>\n"          (après effacement de la zone primaire)
   [pour chaque bloc de <bloc> octets]
host -> <octets bruts>
zynq -> "+ACK <total_recu>\n"
zynq -> "+DONE\n"                  (corps écrit, en-tête écrit, CRC relu OK)
   sinon -> "-ERR <raison>\n"      (params, size, qspi_init, erase, timeout, write, crc_rx, crc_flash, ...)
host -> "REBOOT\r"                 (commande existante -> reboot QSPI)
```

Bloc = **8192 o**, débit **921600 bauds** (cf. `UART_COMM_BAUDRATE` dans [`IO_config.h`](../IO_config.h) — pense à mettre ton terminal série à 921600). ACK bloc-à-bloc = flow-control indispensable au débit du ring buffer RX bare-metal.

## Utilisation (host)

```bash
# Génère BOOT.bin (machine avec Vitis)
./zynq_ota.sh build

# Une fois : installe l'image de secours par JTAG (BOOT.bin connu-bon)
./zynq_ota.sh golden

# Mise à jour série (auto-détection du port, + REBOOT)
./zynq_ota.sh send            # ou 'all' = build + send
```

Détail des rôles et options : voir l'en-tête de [`zynq_ota.sh`](../../zynq_ota.sh) et le [README racine](../../README.md#mise-à-jour-du-firmware-par-liaison-série-ota).

## Redémarrage

Le `REBOOT` final passe par [`System_Reboot()`](../SYSTEM_MANAGER/README.md), qui remet le contrôleur QSPI à zéro avant le soft reset PS. Le soft reset ne rejoue le boot QSPI que si la carte est strappée QSPI ; si tu observes que seul un vrai power-cycle (POR) relance la carte, c'est le point à vérifier (cf. [SYSTEM_MANAGER](../SYSTEM_MANAGER/README.md)).

## Ce qui n'est pas couvert

Le fallback golden se déclenche sur une **image invalide**, pas sur un firmware qui boote mais se **fige** (bug logique). Pour ce cas, le complément est un **watchdog (SWDT)** qui reset si la boucle principale ne le rafraîchit plus (non implémenté à ce jour).

## Voir aussi

* [SYSTEM_MANAGER](../SYSTEM_MANAGER/README.md) — `System_Reboot()` (reset QSPI + soft reset, notes boot mode)
* [DRIVER_UART_PS](../IO_MANAGER/DRIVER_UART_PS/README.md) — transport série
* [APP_COM](../APP_COM/README.md) — interpréteur et table des commandes (`FWUPDATE`, `REBOOT`)
* [board_config.h](../board_config.h) — layout QSPI + flags carte nue
* [README racine](../../README.md) — vue d'ensemble et setup
