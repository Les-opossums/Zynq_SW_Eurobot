# IPC Manager (Inter-Processor Communication)

Ce dossier contient le module dédié à la communication inter-cœurs (CPU0 et CPU1) sur le Zynq-7000. Il permet d'échanger des données en toute sécurité et de synchroniser l'exécution des deux processeurs de manière asynchrone.

## Architecture et Mémoire Partagée

*   **Localisation :** La mémoire partagée est située dans l'OCM (On-Chip Memory) à l'adresse fixe `0xFFFF0000`. Cette zone mémoire globale est définie par le pointeur `IPC_DATA`.
*   **Gestion du Cache (MMU) :** Pour éviter les incohérences de données (problèmes de *cache coherency*) inhérentes à l'architecture multi-cœurs, l'adresse de l'OCM est configurée via le gestionnaire MMU pour être non-cachée, fortement ordonnée (*Strongly Ordered*) et partagée entre les processeurs (attribut `0x14de2`).
*   **Initialisation (`IPC_Init`) :** Au démarrage, le CPU0 est seul responsable de remettre entièrement à zéro (via un `memset`) la structure de données partagée pour garantir un état initial propre.

## Synchronisation au Démarrage

Le module propose la fonction `IPC_SyncCores()` permettant de s'assurer que les deux processeurs entament leur boucle applicative simultanément.
*   Chaque cœur met à `1` son propre drapeau d'état (`core0_ready` ou `core1_ready`).
*   Il entre ensuite dans une boucle d'attente active jusqu'à ce que l'autre cœur ait également levé son drapeau. Des barrières mémoire de données (`dmb()`) sont utilisées pour forcer la synchronisation physique des écritures/lectures.

## Protocole de Transfert (Handshake & Interruptions)

L'échange de données repose sur un mécanisme de drapeaux de validation/acquittement (`flag_valid` / `flag_ack`) couplé aux Interruptions Logicielles (SGI) :
*   **Interruption SGI :** Le module utilise l'identifiant matériel d'interruption logicielle 14 (`IPC_SGI_INT_ID`). La fonction interne `trigger_other_core_interrupt()` exploite le GIC (`XScuGic_SoftwareIntr`) pour envoyer dynamiquement cette interruption au cœur cible (bit `0x01` pour CPU0, bit `0x02` pour CPU1).
*   **Envoi (`IPC_SendToOtherCore`) :** La fonction vérifie d'abord que le récepteur a bien consommé la donnée précédente (`flag_valid` à 1 et `flag_ack` à 0 provoque un retour d'échec `0`). Si l'envoi est possible, les données sont copiées par `memcpy`, les drapeaux sont mis à jour, puis une barrière matérielle critique `dsb sy` force l'écriture mémoire avant de déclencher l'interruption SGI.
*   **Réception (`IPC_CheckFromOtherCore`) :** La fonction vérifie la présence d'une nouvelle information. Si elle existe, les données sont copiées localement, une barrière `dmb sy` est appliquée, puis la réception est acquittée (`flag_ack = 1`, `flag_valid = 0`).

## Macros d'Aide (API Applicative)

Afin de simplifier drastiquement le code au niveau de l'application, l'en-tête `IPC_manager.h` fournit deux macros puissantes qui abstraient la gestion des drapeaux :
*   `SEND_FIELD(data_ptr, field_name)` : Envoie automatiquement un champ spécifique de votre structure `data_ptr` vers `IPC_DATA` en gérant implicitement sa taille et ses pointeurs de drapeaux de validation associés (`flag_##field_name##_valid` et `_ack`).
*   `CHECK_FIELD(data_ptr, field_name)` : Lit une donnée spécifique reçue de l'autre processeur, l'écrit dans la structure pointée par `data_ptr`, et acquitte la transaction.