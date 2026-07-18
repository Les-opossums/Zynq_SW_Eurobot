# IO Manager

Ce dossier contient le cœur du système de gestion des périphériques (Entrées/Sorties) pour l'architecture multi-cœurs du robot. Il permet de centraliser l'initialisation et la mise à jour de tous les drivers afin d'éviter la duplication de code entre les cœurs.

## Description

L'`IO_Manager` est un gestionnaire de haut niveau qui instancie, initialise et met à jour les différents périphériques (GPIO, UART, LEDs, IMU, etc.) en fonction du cœur CPU sur lequel le code est en cours d'exécution. Il s'appuie sur un fichier de configuration externe (`IO_config.h`) qui définit une table globale de périphériques.

## Fonctionnalités Principales

*   **Architecture Multi-Cœurs :** Chaque périphérique déclaré se voit attribuer un "propriétaire" via le type `core_owner_t` (`CORE_CPU0`, `CORE_CPU1`, ou `CORE_BOTH`). Les fonctions d'initialisation et de mise à jour vérifient si le périphérique appartient au cœur appelant (`THIS_CORE`) avant d'exécuter la moindre action.
*   **Structure Universelle (`io_device_t`) :** L'ensemble des périphériques est uniformisé à travers une structure commune contenant :
    *   Le type de périphérique (`dev_type_t`).
    *   Le propriétaire du périphérique.
    *   Un pointeur d'instance générique (`void *driver_instance`).
    *   Les paramètres de gestion d'interruption (`irq_id` et `irq_handler`).
    *   Les pointeurs vers les méthodes spécifiques du driver (`init` et `update`).
*   **Enregistrement automatique des Interruptions :** Lors de l'initialisation, si un ID d'interruption et un gestionnaire sont fournis, l'`IO_Manager` connecte automatiquement l'interruption matérielle via l'`IRQ_Manager`.

## Utilisation et Flux d'Exécution

1.  **Déclaration :** Les périphériques sont déclarés dans la macro `IO_DEVICE_TABLE` du fichier `IO_config.h`[cite: 3].
2.  **Initialisation :** L'appel à `IO_Manager_Init()` parcourt la `DeviceTable`. Si le périphérique appartient au cœur actif, sa méthode `init()` est appelée et ses interruptions sont connectées. Un compte-rendu est affiché sur la console via `xil_printf`.
3.  **Boucle de contrôle :** L'appel régulier à `IO_Manager_Update()` dans la boucle applicative parcourt la `DeviceTable` et invoque la méthode `update()` des périphériques appartenant au cœur actif.

## Dépendances

L'`IO_Manager` interagit avec les modules suivants :
*   `CORE_ID` : Pour identifier le cœur en cours d'exécution (`THIS_CORE_ID`, `THIS_CORE`).
*   `IRQ_MANAGER` : Pour l'enregistrement des interruptions centralisées (`IRQ_Manager_Connect`).
*   Les sous-dossiers spécifiques aux drivers (ex: `DRIVER_PS_GPIO`, `DRIVER_WS2812B`, etc.).