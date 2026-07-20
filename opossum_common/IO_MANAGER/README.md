# IO Manager

Ce dossier contient le cœur du système de gestion des périphériques (Entrées/Sorties) pour l'architecture multi-cœurs du robot. Il permet de centraliser l'initialisation, la mise à jour, et la **désactivation dynamique** de tous les drivers afin d'éviter la duplication de code entre les cœurs.

## Description

L'`IO_Manager` est un gestionnaire de haut niveau qui instancie, initialise et met à jour les différents périphériques (GPIO, UART, LEDs, IMU, CAN, etc.) en fonction du cœur CPU sur lequel le code est en cours d'exécution. Il s'appuie sur un fichier de configuration externe (`IO_config.h`) qui définit une table globale de périphériques.

## Fonctionnalités Principales

*   **Architecture Multi-Cœurs :** Chaque périphérique déclaré se voit attribuer un "propriétaire" via le type `core_owner_t` (`CORE_CPU0`, `CORE_CPU1`, ou `CORE_BOTH`). Les fonctions d'initialisation et de mise à jour vérifient si le périphérique appartient au cœur appelant (`THIS_CORE`) avant d'exécuter la moindre action.
*   **Structure Universelle (`io_device_t`) :** L'ensemble des périphériques est uniformisé à travers une structure commune contenant :
    *   Le type de périphérique (`dev_type_t`).
    *   Le propriétaire du périphérique.
    *   Un pointeur d'instance générique (`void *driver_instance`).
    *   Les paramètres de gestion d'interruption (`irq_id` et `irq_handler`).
    *   **L'état d'activation (`is_active`).**
    *   Les pointeurs vers les méthodes spécifiques du driver (`init`, `update`, et **`deinit`**).
*   **Gestion Dynamique et Arrêt d'Urgence (AU) :** Le gestionnaire permet de désactiver logiciellement et matériellement (via `deinit`) ou de réactiver des périphériques à la volée. Cela est indispensable lors d'un arrêt d'urgence pour couper la communication avec des modules non alimentés et éviter les blocages de bus ou les timeouts.
*   **Enregistrement automatique des Interruptions :** Lors de l'initialisation, si un ID d'interruption et un gestionnaire sont fournis, l'`IO_Manager` connecte automatiquement l'interruption matérielle via l'`IRQ_Manager`.

## Utilisation et Flux d'Exécution

1.  **Déclaration :** Les périphériques sont déclarés dans la macro `IO_DEVICE_TABLE` du fichier `IO_config.h`. Ils doivent inclure leur état initial (`.is_active = 1` ou `0`) et leur éventuelle fonction de coupure (`.deinit`).
2.  **Initialisation :** L'appel à `IO_Manager_Init()` parcourt la `DeviceTable`. Si le périphérique appartient au cœur actif, sa méthode `init()` est appelée et ses interruptions sont connectées.
3.  **Boucle de contrôle :** L'appel régulier à `IO_Manager_Update()` dans la boucle applicative parcourt la `DeviceTable`. **Si un composant est désactivé (`is_active == 0`), sa mise à jour est ignorée.**
4.  **Changement d'État Dynamique :** Deux fonctions permettent d'allumer ou d'éteindre un périphérique :
    *   `IO_Manager_SetDeviceState(dev_type_t type, u8 active)` : agit sur **tous** les périphériques d'un même `dev_type_t` (attention, plusieurs devices peuvent partager un type générique, ex. `DEV_TYPE_UART_PS` pour `UART_COMM` et `UART_FEETECH`).
    *   `IO_Manager_SetDeviceStateByName(const char *name, u8 active)` : cible **un seul** périphérique par son nom (`.name` dans `IO_DEVICE_TABLE`, insensible à la casse), sans ambiguïté même en cas de type partagé ; refuse silencieusement (avec message) d'agir sur un périphérique non géré par le cœur courant. C'est cette variante qu'exploitent les commandes `DRVEN`/`DRVDIS` (cf. [APP_DRIVER_BRIDGE](../APP_DRIVER_BRIDGE/README.md)).
    *   Dans les deux cas : *Désactivation (`active = 0`)* appelle `deinit()` (hardware en état sûr, coupure des interruptions), *Réactivation (`active = 1`)* appelle `init()` (reconfiguration matérielle) puis relance le polling logiciel.
5.  **Liste d'état (`IO_Manager_PrintDeviceList()`)** : imprime l'état ON/OFF de tous les périphériques du cœur courant — utilisé par la commande `DRVLIST` (cf. [APP_DRIVER_BRIDGE](../APP_DRIVER_BRIDGE/README.md)).
6.  **Rapport d'initialisation combiné** : `IO_Manager_Init()` ne print rien directement (pour éviter tout entrelacement sur l'UART partagé entre les deux cœurs) ; chaque cœur stocke silencieusement l'état de ses périphériques (`IO_Manager_ExportStatus()`), et CPU0 imprime un rapport **unique** pour les deux cœurs une fois la synchronisation IPC terminée (`IO_Manager_PrintCombinedInitReport()`, cf. `main.c` et l'[IPC_MANAGER](../IPC_MANAGER/README.md)).
7.  **Instrumentation temporelle (`TIMING_MEASURE`)** : si activé (cf. [TIMER_MANAGER](../TIMER_MANAGER/README.md)), `IO_Manager_Update()` chronomètre automatiquement chaque périphérique de la table (générique — couvre donc tout nouveau driver sans instrumentation manuelle) ; `IO_Manager_PrintTiming()` imprime les résultats.

## Dépendances

L'`IO_Manager` interagit avec les modules suivants :
*   `CORE_ID` : Pour identifier le cœur en cours d'exécution (`THIS_CORE_ID`, `THIS_CORE`).
*   `IRQ_MANAGER` : Pour l'enregistrement des interruptions centralisées (`IRQ_Manager_Connect`).
*   Les sous-dossiers spécifiques aux drivers (ex: `DRIVER_PS_GPIO`, `DRIVER_WS2812B`, etc.).

## Drivers disponibles

* [DRIVER_PS_GPIO](DRIVER_PS_GPIO/README.md) — GPIO du Processing System (AU, laisse, switches, broches BNO085...)
* [DRIVER_WS2812B](DRIVER_WS2812B/README.md) — bandeau LED adressable (IP AXI custom)
* [DRIVER_BNO085](DRIVER_BNO085/README.md) — IMU (SPI, protocole SHTP/SH-2)
* [DRIVER_UART_PS](DRIVER_UART_PS/README.md) — UART du Processing System (console + bus FEETECH)
* [DRIVER_CAN](DRIVER_CAN/README.md) — bus CAN moteurs (ESC C610)
* [DRIVER_ETH](DRIVER_ETH/README.md) — liaison Ethernet UDP avec le Raspberry Pi
* [DRIVER_FEETECH](DRIVER_FEETECH/README.md) — bus servos/pompes des pinces
* [DRIVER_LD19](DRIVER_LD19/README.md) — lidar LD19/LD06 (IP VHDL maison + AXI DMA, interruption)

## Voir aussi

* [CORE_ID](../CORE_ID/README.md) — `core_owner_t`/`THIS_CORE`, utilisés par le filtrage `owner`
* [IRQ_MANAGER](../IRQ_MANAGER/README.md) — connexion automatique des interruptions
* [APP_DRIVER_BRIDGE](../APP_DRIVER_BRIDGE/README.md) — commandes `DRVEN`/`DRVDIS`/`DRVLIST`
* [TIMER_MANAGER](../TIMER_MANAGER/README.md) — instrumentation temporelle (`TIMING_MEASURE`)
* [opossum_common](../README.md) — vue d'ensemble de l'architecture