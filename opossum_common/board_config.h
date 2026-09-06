#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* =========================================================================
 * Configuration "carte nue" — active/desactive les drivers a la COMPILATION
 * pour accelerer l'init (on n'attend plus les peripheriques absents : IMU,
 * servos FEETECH, LIDAR, CAN...).
 *
 * Ces flags pilotent EN UN SEUL ENDROIT :
 *   - les entrees de la table des drivers (DeviceTable[] dans io_manager.c),
 *   - les appels directs correspondants (core0_loop.c, core1_loop.c).
 *
 * Pour reactiver un peripherique quand le hardware est rebranche : repasser
 * son flag a 1 ici, recompiler. Rien d'autre a toucher.
 *
 * Toujours actifs (non conditionnes) : UART_COMM (console + interpreteur,
 * necessaire a l'OTA serie) et GPIO_PS (entrees AU/laisse + LED de vie MIO0).
 * ========================================================================= */

#define USE_ETHERNET   0   /* EMAC/PHY + telemetrie AU/laisse/pose vers le Pi */
#define USE_WS2812B    1   /* bandeau LED d'etat (init rapide, PL)            */
#define USE_IMU        0   /* BNO085 (SPI) — init lente si absent            */
#define USE_FEETECH    0   /* bus servos/pompes des pinces (UART1 + servos)  */
#define USE_CAN        0   /* bus moteurs C610 (CAN)                          */
#define USE_LIDAR      0   /* LD19 (UART + DMA PL)                            */

#define USE_ASSERV     0   /* boucle d'asserv CPU1 (depend de CAN + IMU)      */


/* =========================================================================
 * Layout QSPI + securite OTA (image golden / fallback MultiBoot)
 * ========================================================================= *
 * L'OTA reecrit UNIQUEMENT l'image primaire a l'offset 0. Une image "golden"
 * (secours, connue-bonne, avec la commande FWUPDATE) est flashee UNE FOIS par
 * JTAG a QSPI_GOLDEN_OFFSET et n'est jamais touchee par l'OTA. Si l'image
 * primaire est corrompue (OTA interrompu), le BootROM fait sa "Golden Image
 * Search" (recherche d'un en-tete valide tous les 32 Ko en remontant) et
 * boote la golden -> la carte n'est jamais bricke.
 *
 * QSPI_GOLDEN_OFFSET doit etre aligne sur 32 Ko, > taille max de l'image
 * primaire, et rentrer dans la flash (verifier la taille de ta puce : 4 Mo
 * ici convient pour une flash >= 8 Mo et une image < 4 Mo ; monte-le si ton
 * image grossit et que la flash est plus grande). Cote host, GOLDEN_OFFSET
 * dans zynq_ota.sh doit valoir la MEME chose.
 */
#define QSPI_GOLDEN_OFFSET    0x00400000U        /* 4 Mo : offset de l'image golden */
#define QSPI_UPDATE_MAX_SIZE  QSPI_GOLDEN_OFFSET /* l'OTA (offset 0) ne doit jamais l'atteindre */

#endif /* BOARD_CONFIG_H */
