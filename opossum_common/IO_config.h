#ifndef IO_CONFIG_H
#define IO_CONFIG_H

#include "xparameters.h"

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

/* ================================================================= *
 * Fonctions de callback pour les interruptions (Optionnel)
 * ================================================================= */
extern void leash_Callback(void *callback_ref);
extern void AU_Callback(void *callback_ref);

#define PS_GPIO_PINS { \
    /* PIN,     DIRECTION,      INTERRUPTION,           VARIABLE LIEE,      CALLBACK SPECIFIQUE */\
    { 55,       PS_GPIO_DIR_INPUT,   PIN_IRQ_EDGE_BOTH,      &AU_state,          AU_Callback}, \
    { 54,       PS_GPIO_DIR_INPUT,   PIN_IRQ_EDGE_RISING,    &leash_state,       leash_Callback}, \
    { 56,       PS_GPIO_DIR_INPUT,   PIN_IRQ_NONE,           &team_state,        NULL}, \
    { 57,       PS_GPIO_DIR_INPUT,   PIN_IRQ_NONE,           &IO_1_state,        NULL}, \
    { 58,       PS_GPIO_DIR_INPUT,   PIN_IRQ_NONE,           &IO_2_state,        NULL}, \
    { 59,       PS_GPIO_DIR_INPUT,   PIN_IRQ_NONE,           &IO_3_state,        NULL}, \
    { 61,       PS_GPIO_DIR_OUTPUT,  PIN_IRQ_NONE,           NULL,     NULL}, /* bno_rst — piloté directement par le driver */ \
    { 62,       PS_GPIO_DIR_OUTPUT,  PIN_IRQ_NONE,           &bno_wake_state,    NULL}, /* bno_cs  — piloté directement par le driver */ \
    { 63,       PS_GPIO_DIR_OUTPUT,  PIN_IRQ_NONE,           NULL,      NULL}, \
    { 60,       PS_GPIO_DIR_INPUT ,  PIN_IRQ_EDGE_FALLING,   &bno_int_state,     BNO085_INT_Callback} \
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
#define BNO085_REPORTS { \
    { SH2_GYROSCOPE_CALIBRATED, 2000U }, \
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
 * Table de l'io manager
 * ================================================================= */
#define IO_DEVICE_TABLE { \
    { \
        .name = "GPIO_PS", \
        .type = DEV_TYPE_GPIO_PS, \
        .owner = CORE_CPU0, \
        .driver_instance = &PsGpio_Ctx, \
        .irq_id = XPAR_XGPIOPS_0_INTR, \
        .irq_handler = (Xil_InterruptHandler)XGpioPs_IntrHandler, \
        .init = PS_GPIO_Init, \
        .update = PS_GPIO_Update, \
        .deinit = NULL \
    }, \
    { \
        .name = "WS2812B", \
        .type = DEV_TYPE_WS2812B, \
        .owner = CORE_CPU0, \
        .driver_instance = &Ws2812b_Ctx, \
        .irq_id = 0, \
        .irq_handler = NULL, \
        .init = WS2812B_Init, \
        .update = WS2812B_Update, \
        .deinit = NULL \
    }, \
    {  \
        .name = "IMU_BNO085", \
        .type = DEV_TYPE_IMU_BNO085, \
        .owner = CORE_CPU0, \
        .driver_instance = &Imu_Ctx, \
        .irq_id = 0, \
        .irq_handler = NULL, \
        .init = BNO085_IO_Init, \
        .update = BNO085_IO_Update, \
        .deinit = NULL \
    }, \
    { \
        .name = "UART_COMM", \
        .type = DEV_TYPE_UART_PS, \
        .owner = CORE_CPU0, \
        .driver_instance = &UartComm_Ctx, \
        .irq_id = XPAR_XUARTPS_0_INTR, \
        .irq_handler = (Xil_InterruptHandler)XUartPs_InterruptHandler, \
        .init = UART_PS_Init, \
        .update = UART_PS_Update, \
        .deinit = NULL \
    }, \
    { \
        .name = "CAN_MOTORS", \
        .type = DEV_TYPE_CAN_MOTORS, \
        .owner = CORE_CPU1, \
        .driver_instance = &Can0_Ctx, \
        .irq_id = CAN0_INTR_ID, \
        .irq_handler = (Xil_InterruptHandler)XCanPs_IntrHandler, \
        .init = CAN_IO_Init, \
        .update = CAN_IO_Update, \
        .deinit = CAN_IO_Deinit \
    }\
}

    

    //}, \
    // { \
    //     .type = DEV_TYPE_ETHERNET, \
    //     .owner = CORE_CPU0, \
    //     .driver_instance = &Eth_Ctx, \
    //     .irq_id = 0, \
    //     .irq_handler = NULL, \
    //     .init = ETH_IO_Init, \
    //     .update = ETH_IO_Update, \
    //     .deinit = NULL \
    // } \
}

#endif /* IO_CONFIG_H */