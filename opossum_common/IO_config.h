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

#define PS_GPIO_PINS { \
    /* PIN,     DIRECTION,      INTERRUPTION,           VARIABLE LIEE,      CALLBACK SPECIFIQUE */\
    { 55,       PS_GPIO_DIR_INPUT,   PIN_IRQ_NONE,           &AU_state,          NULL}, \
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
 * Table de l'io manager
 * ================================================================= */
#define IO_DEVICE_TABLE { \
    { \
        .type = DEV_TYPE_GPIO_PS, \
        .owner = CORE_CPU0, \
        .driver_instance = &PsGpio_Ctx, \
        .irq_id = XPAR_XGPIOPS_0_INTR, \
        .irq_handler = (Xil_InterruptHandler)XGpioPs_IntrHandler, \
        .init = PS_GPIO_Init, \
        .update = PS_GPIO_Update \
    }, \
    { \
        .type = DEV_TYPE_WS2812B, \
        .owner = CORE_CPU0, \
        .driver_instance = &Ws2812b_Ctx, \
        .irq_id = 0, \
        .irq_handler = NULL, \
        .init = WS2812B_Init, \
        .update = WS2812B_Update \
    }, \
    {   .type = DEV_TYPE_IMU_BNO085, \
        .owner = CORE_CPU0, \
        .driver_instance = &Imu_Ctx, \
        .irq_id = 0, \
        .irq_handler = NULL, \
        .init = BNO085_IO_Init, \
        .update = BNO085_IO_Update \
    }, \
    { \
        .type = DEV_TYPE_UART_PS, \
        .owner = CORE_CPU0, \
        .driver_instance = &UartComm_Ctx, \
        .irq_id = XPAR_XUARTPS_0_INTR, \
        .irq_handler = (Xil_InterruptHandler)XUartPs_InterruptHandler, \
        .init = UART_PS_Init, \
        .update = UART_PS_Update \
    } \
}

#endif /* IO_CONFIG_H */