# Driver BNO085 (IMU)

Ce sous-dossier contient le pilote de la centrale inertielle BNO085 pour l'architecture Zynq-7000. Il implémente les communications via le bus SPI en exploitant le protocole SHTP (Sensor Hub Transport Protocol) et le parsing des rapports SH-2.

## Architecture du Module

Le pilote est scindé en deux couches distinctes pour séparer la logique bas-niveau de l'intégration système :

*   **Couche Cœur SHTP/SH-2 (`BNO085.c` / `BNO085.h`) :** Gère la machine d'état du capteur, l'envoi/réception des paquets SHTP sur le bus SPI, et le décodage des trames binaires complexes (Feature Reports) en valeurs physiques (flottants).
*   **Surcouche `IO_MANAGER` (`driver_bno085_io.c` / `driver_bno085_io.h`) :** Encapsule le driver sous une forme standardisée (structure `bno085_io_context_t`) disposant de fonctions `Init` et `Update` compatibles avec la table de l'`IO_Manager`.

## Gestion Matérielle (Broches & SPI)

Ce driver interagit avec le matériel de manière spécifique :

*   **SPI (Mode 3) :** Les échanges utilisent le périphérique PS SPI0 configuré en Mode 3 (CPOL=1, CPHA=1), avec le signal Chip Select (CS) matériel forcé à l'état désactivé.
*   **GPIO PS Partagé :** Le driver ne possède pas de contexte GPIO propre ; il reçoit un pointeur vers le contexte partagé `PsGpio_Ctx` pour manipuler ses trois broches.
    *   **CS (Sortie) :** Le Chip Select est piloté manuellement (actif bas) car le séquencement doit se faire *à l'intérieur* même de la transaction SHTP.
    *   **RST (Sortie) :** Broche de reset matériel (actif bas)[cite: 13, 15].
    *   **INT (Entrée) :** Ligne d'interruption du capteur, configurée sur un front descendant pour signaler la disponibilité des données.

## Fonctionnement et Flux d'Exécution

*   **Initialisation (`BNO085_IO_Init`) :** La fonction configure l'interface SPI, procède à un reset matériel du capteur, puis active automatiquement la liste des rapports demandés dans la configuration (`report_table`).
*   **Interruption ISR (`BNO085_INT_Callback`) :** La routine de service d'interruption associée à la broche INT est extrêmement courte et non bloquante ; elle se contente de lever un drapeau logiciel (`Bno085_DataReadyFlag`).
*   **Polling et Partage IPC (`BNO085_IO_Update`) :** 
    *   Appelée à chaque itération de l'application principale, cette fonction vérifie le drapeau d'interruption.
    *   Si des données sont prêtes, elle déclenche la lecture SPI (`BNO085_Poll`).
    *   Les nouvelles données (accéléromètre, gyroscope, statut de calibration, etc.) sont immédiatement publiées dans la mémoire partagée (`IPC_DATA`) pour être rendues accessibles au second cœur (`CORE1`).

## Types de Données Supportées (SH-2)

Le cœur du driver est capable de parser dynamiquement différents types de données (convertis depuis leur format "Q-point" interne vers des variables de type *float*) :

*   **Accéléromètres :** Accélération brute et accélération linéaire (gravité soustraite).
*   **Gyroscope :** Vitesse angulaire calibrée.
*   **Magnétomètre :** Champ magnétique en µT.
*   **Vecteurs de Rotation :** Vecteur AHRS classique (avec référence Nord) et "Game Rotation Vector" (sans magnétomètre pour éviter les perturbations).
*   **Angles d'Euler :** Les quaternions reçus sont automatiquement convertis en angles ZYX (Yaw, Pitch, Roll) en degrés.