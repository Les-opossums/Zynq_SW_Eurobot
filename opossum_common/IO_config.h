#ifndef IO_CONFIG_H
#define IO_CONFIG_H

#include "xparameters.h"
#include "board_config.h"   // flags USE_xxx (config carte nue)

/* ================================================================= *
 * Include vers l'IO Manager
 * ================================================================= */
#include "IO_MANAGER/io_manager.h"

/* ================================================================= *
 * Include vers les drivers spécifiques
 * ================================================================= */
#include "IO_MANAGER/DRIVER_PS_GPIO/driver_ps_gpio.h"
#include "IO_MANAGER/DRIVER_WS2812B/driver_ws2812b.h"
#include "IO_MANAGER/DRIVER_BNO085/driver_bno085_io.h"
#include "IO_MANAGER/DRIVER_UART_PS/driver_uart_ps.h"
#include "IO_MANAGER/DRIVER_CAN/driver_can_io.h"
#include "IO_MANAGER/DRIVER_ETH/driver_eth_io.h"
#include "IO_MANAGER/DRIVER_FEETECH/feetech_io.h"
#include "IO_MANAGER/DRIVER_LD19/lidar_ld19.h"
#include "APP_MOTORS/c610_feedback.h"

/* ================================================================= *
 * Variables globales des états
 * ================================================================= */
extern volatile int AU_state;        // Etat de l'AU (0 = relâché, 1 = appuyé)
extern volatile int leash_state;     // Etat de la laisse (0 = relâchée, 1 = attachée)
extern volatile int team_state;      // Etat de l'équipe (0 = équipe 1, 1 = équipe 2)
extern volatile int IO_1_state;      // switch IHM
extern volatile int IO_2_state;      // switch IHM
extern volatile int IO_3_state;      // switch IHM
extern volatile int bno_cs_state;
extern volatile int bno_rst_state;
extern volatile int bno_int_state;
extern volatile int bno_wake_state;
extern volatile int alive_led_state; // LED de vie (heartbeat 1 Hz) sur MIO0

/* ================================================================= *
 * Fonctions de callback pour les interruptions (Optionnel)
 * ================================================================= */
extern void leash_Callback(void *callback_ref);
extern void AU_Callback(void *callback_ref);

#define PS_GPIO_PINS { \
    /* PIN,     DIRECTION,      INTERRUPTION,           VARIABLE LIEE,      CALLBACK SPECIFIQUE */\
    { 55,       PS_GPIO_DIR_INPUT,   PIN_IRQ_EDGE_BOTH,      &AU_state,          AU_Callback}, \
    { 54,       PS_GPIO_DIR_INPUT,   PIN_IRQ_EDGE_BOTH,      &leash_state,       leash_Callback}, \
    { 56,       PS_GPIO_DIR_INPUT,   PIN_IRQ_NONE,           &team_state,        NULL}, \
    { 57,       PS_GPIO_DIR_INPUT,   PIN_IRQ_NONE,           &IO_1_state,        NULL}, \
    { 58,       PS_GPIO_DIR_INPUT,   PIN_IRQ_NONE,           &IO_2_state,        NULL}, \
    { 59,       PS_GPIO_DIR_INPUT,   PIN_IRQ_NONE,           &IO_3_state,        NULL}, \
    { 61,       PS_GPIO_DIR_OUTPUT,  PIN_IRQ_NONE,           NULL,     NULL}, /* bno_rst — piloté directement par le driver */ \
    { 62,       PS_GPIO_DIR_OUTPUT,  PIN_IRQ_NONE,           &bno_wake_state,    NULL}, /* bno_cs  — piloté directement par le driver */ \
    { 63,       PS_GPIO_DIR_OUTPUT,  PIN_IRQ_NONE,           NULL,      NULL}, \
    { 60,       PS_GPIO_DIR_INPUT ,  PIN_IRQ_EDGE_FALLING,   &bno_int_state,     BNO085_INT_Callback}, \
    /* LED de vie (heartbeat 1 Hz) cablee sur MIO0, banque 500 : pilotee en
     * sortie, l'etat suit alive_led_state (toggle a 500 ms dans core0_loop.c). */\
    { 0,        PS_GPIO_DIR_OUTPUT,  PIN_IRQ_NONE,           &alive_led_state,   NULL} \
}

/* ================================================================= *
 * Configuration du driver GPIO PS
 * ================================================================= */
extern ps_gpio_context_t PsGpio_Ctx; // Contexte du driver GPIO PS

/* ================================================================= *
 * Configuration du bandeau LED WS2812B
 * ================================================================= */
#define WS2812B_BASEADDR    XPAR_AXI_WS2812B_RAM_0_BASEADDR 
#define NBR_LED             44

extern led_color_t Led_Buffer[NBR_LED];
extern ws2812b_context_t Ws2812b_Ctx; // Contexte du driver WS2812B

/* ================================================================= *
 * Configuration du driver BNO085
 * =================================================================
 */

/* --- Choix de la source d'orientation absolue de l'IMU -----------------
 * IMU_USE_MAGNETO :
 *   1 -> SH2_ROTATION_VECTOR      : fusion 9 axes AVEC magnetometre
 *                                   (cap reference sur le Nord magnetique,
 *                                    pas de derive long terme mais sensible
 *                                    aux perturbations magnetiques : moteurs,
 *                                    structures metalliques de la table...).
 *   0 -> SH2_GAME_ROTATION_VECTOR : fusion 6 axes SANS magnetometre
 *                                   (origine de cap arbitraire au demarrage,
 *                                    immunise aux perturbations magnetiques,
 *                                    tres faible derive sur la duree d'un match).
 * Dans les deux cas, la fusion heading de CORE1 recale ce cap sur le repere
 * monde via un offset moyenne sur les mesures lidar : l'origine arbitraire du
 * Game Rotation Vector n'est donc pas un probleme. Le Game Rotation Vector est
 * le choix par defaut (robuste aux moteurs), passer a 1 pour tester le magneto. */
#ifndef IMU_USE_MAGNETO
#define IMU_USE_MAGNETO 0
#endif

#if IMU_USE_MAGNETO
#define IMU_ORIENTATION_REPORT  SH2_ROTATION_VECTOR
#else
#define IMU_ORIENTATION_REPORT  SH2_GAME_ROTATION_VECTOR
#endif

/* Gyroscope calibre (vitesse angulaire, update Kalman vtheta) a ~500 Hz, plus
 * le rapport d'orientation absolue (update Kalman theta) a 100 Hz. */
#define BNO085_REPORTS { \
    { SH2_GYROSCOPE_CALIBRATED, 2000U }, \
    { IMU_ORIENTATION_REPORT,  10000U }, \
}

extern bno085_io_context_t Imu_Ctx;

/* ================================================================= *
 * Configuration UART PS (communication avec les autres cartes)
 * ================================================================= */
#define UART_COMM_DEVICE_ID   XPAR_XUARTPS_0_DEVICE_ID
#define UART_COMM_BAUDRATE    115200 //921600

extern uart_ps_context_t UartComm_Ctx;

/* ================================================================= *
 * CAN0 — bus moteurs (ESC C610), actif sur CORE1
 * ================================================================= */
#define CAN0_DEVICE_ID       XPAR_XCANPS_0_DEVICE_ID
// ATTENTION : XPAR_XCANPS_0_INTR (alias "canonique" genere par les outils
// Xilinx) vaut XPS_CAN0_INT_ID (60), l'ID d'interruption du controleur CAN0
// PHYSIQUE. Mais XPAR_XCANPS_0_BASEADDR pointe en realite vers 0xE0009000,
// c-a-d le controleur CAN1 physique (le seul instancie dans ce hardware
// design) -- bug connu de generation des parametres canoniques Xilinx
// quand seule la 2e instance d'un peripherique est activee. On utilise donc
// directement XPAR_PS7_CAN_1_INTR (= XPS_CAN1_INT_ID = 83), le vrai ID
// d'interruption du CAN1, pour que le GIC ecoute la bonne ligne.
#define CAN0_INTR_ID         XPAR_PS7_CAN_1_INTR

#define CAN0_BAUD_PRESCALER  9
#define CAN0_BTR_SJW         0
#define CAN0_BTR_TS2         0
#define CAN0_BTR_TS1         7

#define CAN0_SUBSCRIBERS { \
    { CAN_MOTOR_1_ID, C610_Motor_Callback, &motor_feedback[0] }, \
    { CAN_MOTOR_2_ID, C610_Motor_Callback, &motor_feedback[1] }, \
    { CAN_MOTOR_3_ID, C610_Motor_Callback, &motor_feedback[2] }, \
    { CAN_MOTOR_4_ID, C610_Motor_Callback, &motor_feedback[3] }, \
}

extern can_io_context_t Can0_Ctx;

/* ================================================================= *
 * Configuration Ethernet (communication avec le Raspberry Pi)
 * ================================================================= */
extern eth_io_context_t Eth_Ctx;

/* ================================================================= *
 * Configuration FEETECH — bus servos/pompes des pinces, sur CORE0
 * ================================================================= *
 * Transport : UART1 (deja compile en dur dans le BSP standalone), dedie
 * au bus half-duplex FEETECH, distinct de UART0 utilisee par UART_COMM.
 * Direction du bus (buffer half-duplex) : AXI GPIO dediee (pas de PS GPIO
 * libre pour ce role sur ce hardware).
 */
#define UART_FEETECH_DEVICE_ID   XPAR_XUARTPS_1_DEVICE_ID
#define UART_FEETECH_BAUDRATE    1000000 // baud usine des servos STS3215/SCS0009 ; a ajuster si reconfigures

#define FEETECH_DIR_GPIO_DEVICE_ID  XPAR_AXI_GPIO_6_DEVICE_ID
#define FEETECH_DIR_GPIO_CHANNEL    1

extern uart_ps_context_t UartFeetech_Ctx;
extern feetech_io_context_t Feetech_Ctx;

/* ================================================================= *
 * Configuration LIDAR LD19 — IP maison lidar_top_for_dma + AXI DMA
 * (S2MM uniquement), sur CORE0.
 * ================================================================= *
 * Chaine : UART LD19 -> parseur -> filtre -> clustering (VHDL, PL) ->
 * AXI-Stream -> AXI DMA -> tampon memoire (cf opossum_hw pour le detail).
 * Config/statut du filtre/clustering exposes en AXI4-Lite sur
 * XPAR_LIDAR_TOP_FOR_DMA_0_BASEADDR (cf lidar_ld19.h pour la carte des
 * registres). Reception DMA en mode Direct Register (pas de Scatter-
 * Gather), pilotee par interruption (S2MM, cf LIDAR_LD19_IntrHandler).
 *
 * XPAR_FABRIC_AXI_DMA_0_S2MM_INTROUT_INTR est cablee directement sur le
 * GIC (ID 62, distincte de l'IRQ AXI_UARTLITE_0 = ID 61) dans le hardware
 * actuellement exporte -- pas de xlconcat partage entre les deux a ce
 * jour, malgre ce qui avait ete mentionne verbalement ; a revisiter si le
 * block design est modifie en ce sens et re-exporte.
 */
#define LIDAR_LD19_DMA_DEVICE_ID  XPAR_AXI_DMA_0_DEVICE_ID
#define LIDAR_LD19_REGS_BASEADDR  XPAR_LIDAR_TOP_FOR_DMA_0_BASEADDR
#define LIDAR_LD19_INTR_ID        XPAR_FABRIC_AXI_DMA_0_S2MM_INTROUT_INTR

extern lidar_ld19_context_t Lidar_Ld19_Ctx;

/* ================================================================= *
 * Table des peripheriques (DeviceTable[])
 * ================================================================= *
 * Instanciee dans io_manager.c avec des gardes #if USE_xxx (cf
 * board_config.h) : un #if etant impossible dans une macro, la table
 * n'est plus un #define ici. Pour activer/desactiver un driver, editer
 * board_config.h (un seul endroit pour la table ET les appels directs).
 * ================================================================= */

#endif /* IO_CONFIG_H */