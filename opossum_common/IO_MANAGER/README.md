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
4.  **Changement d'État Dynamique :** L'utilisation de la fonction `IO_Manager_SetDeviceState(dev_type_t type, u8 active)` permet d'allumer ou d'éteindre un périphérique :
    *   *Désactivation (`active = 0`) :* Appelle la fonction `deinit()` pour mettre le hardware dans un état sûr (coupure des interruptions, vidage des buffers), puis passe l'état inactif pour stopper les appels à `update()`.
    *   *Réactivation (`active = 1`) :* Appelle la fonction `init()` pour reconfigurer le matériel, puis relance le polling logiciel.

## Dépendances

L'`IO_Manager` interagit avec les modules suivants :
*   `CORE_ID` : Pour identifier le cœur en cours d'exécution (`THIS_CORE_ID`, `THIS_CORE`).
*   `IRQ_MANAGER` : Pour l'enregistrement des interruptions centralisées (`IRQ_Manager_Connect`).
*   Les sous-dossiers spécifiques aux drivers (ex: `DRIVER_PS_GPIO`, `DRIVER_WS2812B`, etc.).