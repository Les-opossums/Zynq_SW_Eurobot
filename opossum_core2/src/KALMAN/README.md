# KALMAN — Filtre de fusion de capteurs (CPU1)

Ce dossier contient le filtre de Kalman étendu (EKF) qui fusionne l'odométrie, le lidar, jusqu'à 3 caméras et l'IMU pour estimer la position/vitesse du robot, ainsi qu'une FIFO permettant de corriger le passé (les mesures externes arrivent toujours en retard) et de "repropager" l'effet de la correction jusqu'au présent.

## Fichiers

* **`kalman.c` / `kalman.h`** — L'état du filtre (`KalmanState` : vecteur d'état `x[6]` = x, y, theta, vx, vy, vtheta + covariance `P[6][6]`) et les opérations de base :
  * `kalman_predict(state, dt)` — prédiction (modèle cinématique)
  * `kalman_update_odo(state, speed)` — correction par les vitesses odométriques (encodeurs, cf. [ASSERV](../ASSERV/README.md))
  * `kalman_update_imu(state, vtheta)` — correction par la vitesse angulaire IMU
  * `kalman_update(state, z, R_diag, bypass_outlier)` — correction générique par une mesure de position absolue (lidar/caméra), avec rejet d'outliers par distance de Mahalanobis
  * Les constantes de bruit de processus (`PROCESS_NOISE_*`) et de mesure (`OBS_NOISE_*`) sont en tête du fichier, à ajuster selon la confiance accordée à chaque capteur.
* **`kalman_FIFO.c` / `kalman_FIFO.h`** — `KalmanFIFO` (200 emplacements, un par cycle de boucle rapide) : historise l'état du filtre et les observations disponibles à chaque instant, pour permettre :
  * `kalman_fifo_insert_lidar` / `kalman_fifo_insert_camera` — insertion d'une mesure retardée (délai fourni par le capteur) à son index temporel réel dans la FIFO.
  * `kalman_fifo_repropagate_start` / `..._tick` — relance la prédiction+correction à partir du point corrigé jusqu'au présent, étalée sur plusieurs cycles de la boucle rapide (`REPROPAGATE_STEPS_PER_TICK` slots par tick) pour ne jamais bloquer le temps réel.

## Pourquoi une FIFO de repropagation ?

Le lidar et les caméras (traités côté Raspberry Pi) arrivent avec un délai de plusieurs dizaines de millisecondes. Corriger uniquement l'état **présent** avec une mesure du **passé** introduirait une erreur systématique. La FIFO permet de : retrouver l'état tel qu'il était au moment réel de la mesure, y appliquer la correction, puis rejouer (repropager) la prédiction pas à pas jusqu'à l'instant présent avec les vitesses odométrie/IMU historisées — sans jamais geler la boucle rapide (le travail est étalé sur plusieurs cycles via `repropagate_job`).

## Intégration avec ASSERV

Toute la logique d'insertion/déclenchement de repropagation est pilotée depuis [`asserv_loop.c`](../ASSERV/README.md) (`receive_commands()` pour l'insertion, `fast_loop()` pour la prédiction/repropagation, cf. [ASSERV](../ASSERV/README.md)). Ce dossier ne contient que le filtre lui-même, sans dépendance au séquencement temps réel.

## Voir aussi

* [ASSERV](../ASSERV/README.md) — boucle qui pilote ce filtre (prédiction, insertion des mesures, repropagation)
* [APP_ASSERV_BRIDGE](../../../opossum_common/APP_ASSERV_BRIDGE/README.md) — commandes `SETLIDAR`/`SETCAMERA*`/`LIDARNOISE`/`ENKALMAN` qui alimentent ce filtre depuis CPU0
* [opossum_core2](../../README.md) — vue d'ensemble du projet CPU1
