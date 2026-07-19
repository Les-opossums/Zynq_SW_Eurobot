# Driver CAN PS (Controller Area Network)

Ce dossier contient le pilote permettant la gestion du contrôleur CAN intégré au Processing System (PS) du Zynq-7000. Il est conçu pour s'intégrer nativement à l'architecture de l'`IO_MANAGER` et supporte la désactivation dynamique requise lors des arrêts d'urgence.

## Architecture du Pilote

Le driver CAN PS s'appuie sur la bibliothèque matérielle `xcanps` de Xilinx. Contrairement à une simple scrutation (polling), la réception (RX) est **intégralement gérée sous interruption**, garantissant qu'aucune trame prioritaire ne soit manquée.

Le pilote implémente le paradigme de "Publication/Abonnement" (Publish/Subscribe) pour simplifier le traitement applicatif :
*   Le matériel filtre en amont les trames indésirables.
*   Seuls les messages dont l'ID correspond à un abonnement déclenchent un callback spécifique vers la couche application.

## Fonctionnalités Principales

*   **Filtrage Matériel Strict :** Le Zynq-7000 disposant de 4 filtres d'acceptance (UAF1 à UAF4), le pilote supporte jusqu'à 4 "abonnés" (subscribers) simultanés (`CAN_IO_MAX_SUBSCRIBERS`). Les masques sont configurés pour garantir une correspondance exacte de l'ID, ignorant tout trafic non sollicité.
*   **Réception sous Interruption (ISR) :** À chaque trame reçue correspondant à un filtre, le gestionnaire `CAN_IO_RecvHandler` extrait les données et invoque immédiatement la fonction de callback associée à l'ID reçu.
*   **Envoi Bloquant :** La fonction d'envoi (`CAN_IO_Send`) insère la trame dans la FIFO matérielle. Si celle-ci est pleine, la fonction bloque (attente active) jusqu'à ce qu'une place se libère.
*   **Désactivation Dynamique (Arrêt d'Urgence) :** En conformité avec l'architecture `IO_MANAGER`, le driver implémente une fonction `Deinit` (`CAN_IO_Disable`). Appelée lors d'un arrêt d'urgence (câble débranché, perte de puissance d'un module), elle masque les interruptions et passe le contrôleur en mode configuration, évitant ainsi un emballement des compteurs d'erreurs d'acquittement (Bus-Off).
*   **Statistiques et Diagnostics :** Le pilote enregistre en continu tous les types d'erreurs bus (ACK, Stuff, Form, Bit, CRC) ainsi que les débordements de FIFO et les pertes d'arbitrage. Ces compteurs (`CAN_ErrorStats`) sont consultables via `CAN_IO_PrintErrorStats`.
*   **Debug gérable à part :** Les prints de diagnostic à l'init/activation/désactivation (`CAN_IO_DEBUG`/`CAN_IO_LOG`, à décommenter dans `driver_can_io.h`) sont désactivés par défaut — le rapport d'init unique de l'[IO_MANAGER](../README.md) suffit en usage normal. `CAN_IO_PrintErrorStats` reste lui toujours actif : c'est un diagnostic explicite à la demande, pas du bruit de boot.

## Structure de Données

L'application doit instancier un `can_io_context_t` pour chaque bus physique utilisé (CAN0 ou CAN1). Ce contexte regroupe :
*   L'instance matérielle `XCanPs`.
*   Les paramètres temporels (Prescaler, TS1, TS2, SJW) définissant le baudrate.
*   Le tableau de configuration des abonnés (`subscriber_table`).
*   L'état d'activation du bus et l'historique des erreurs matérielles.

## Interface API Standard (`IO_MANAGER`)

*   `int CAN_IO_Init(void *instance)` : Initialise le contrôleur, configure le timing, active les filtres d'acceptance selon le tableau des abonnés, lie les 4 gestionnaires d'interruption (Send, Recv, Error, Event), et bascule en mode Normal.
*   `void CAN_IO_Update(void *instance)` : Fonction vide, présente uniquement pour la conformité avec `io_device_t`. Le traitement RX est asynchrone (ISR) et le TX est à la demande.
*   `void CAN_IO_Deinit(void *instance)` : Désactive les interruptions et suspend le contrôleur.

## API Applicative

*   `int CAN_IO_Send(ctx, id, payload, len)` : Transmet une trame de données avec l'identifiant 11-bits spécifié.
*   `void CAN_IO_Disable(ctx)` / `CAN_IO_Enable(ctx)` : Bascule logiciellement l'état de la liaison CAN (désactive/réactive la participation au trafic sur le bus).
*   `void CAN_IO_PrintErrorStats(ctx)` : Affiche le bilan de santé du bus sur la console série (utile pour détecter un problème de câblage ou de résistance de terminaison).

## Voir aussi

* [IO_MANAGER](../README.md) — table des périphériques et cycle de vie générique
* [APP_MOTORS](../../APP_MOTORS/README.md) — couche applicative (retour/commande des ESC C610) au-dessus de ce driver
* [ASSERV (CPU1)](../../../opossum_core2/src/ASSERV/README.md) — seul consommateur de ce bus
* [APP_DRIVER_BRIDGE](../../APP_DRIVER_BRIDGE/README.md) — commandes `DRVEN "CAN_MOTORS"` / `DRVDIS "CAN_MOTORS"` / `DRVLIST`
* [opossum_common](../../README.md) — vue d'ensemble de l'architecture