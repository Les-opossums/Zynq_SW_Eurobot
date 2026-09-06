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

#endif /* BOARD_CONFIG_H */
