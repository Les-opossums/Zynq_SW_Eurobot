# APP_MOTORS — Retour et commande des ESC C610 (moteurs M2006)

Ce dossier contient la couche applicative au-dessus du [driver CAN](../IO_MANAGER/DRIVER_CAN/README.md) spécifique aux 4 ESC C610 (moteurs M2006) du robot. Utilisé uniquement sur CPU1 (cf. [`opossum_core2` / ASSERV](../../opossum_core2/src/ASSERV/README.md)), dont dépend directement l'odométrie et le calcul PID.

## Fichiers

* **`c610_feedback.c` / `c610_feedback.h`**

## Fonctionnement

* `C610_Motor_Callback(app_ctx, frame_words, dlc)` — callback CAN (abonnement, cf. [DRIVER_CAN](../IO_MANAGER/DRIVER_CAN/README.md)) qui décode une trame de retour C610 (angle, vitesse, couple) et met à jour l'instance `C610_MotorFeedback_t` correspondante (un abonnement par moteur, IDs `CAN_MOTOR_1_ID`.. `CAN_MOTOR_4_ID` = `0x201`..`0x204`).
* `motor_feedback[4]` — tableau des 4 retours moteurs, lu directement par l'odométrie (cf. [ASSERV](../../opossum_core2/src/ASSERV/README.md)) à chaque cycle de la boucle rapide.
* `Init_CAN_MOTOR_variables()` — remet à zéro les 4 structures de retour (appelé par `asserv_loop_init()`).
* `CAN_transmit_motor(ctx, motors, nb_motors)` — envoie les 4 consignes de courant aux ESC en une seule trame CAN (`ESC_TX_MESSAGE_ID = 0x200`, 2 octets big-endian par moteur).

## Voir aussi

* [DRIVER_CAN](../IO_MANAGER/DRIVER_CAN/README.md) — driver bas niveau (filtrage matériel, publish/subscribe)
* [ASSERV (CPU1)](../../opossum_core2/src/ASSERV/README.md) — consommateur de `motor_feedback[]` et de `CAN_transmit_motor()`
* [opossum_common](../README.md) — vue d'ensemble de l'architecture
