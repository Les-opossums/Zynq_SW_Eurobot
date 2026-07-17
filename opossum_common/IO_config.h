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
    { 55,       IO_DIR_INPUT,   PIN_IRQ_EDGE_BOTH,      &AU_state,          NULL}, \
    { 54,       IO_DIR_INPUT,   PIN_IRQ_EDGE_RISING,    &leash_state,       leash_Callback}, \
    { 56,       IO_DIR_INPUT,   PIN_IRQ_NONE,           &team_state,        NULL}, \
    { 57,       IO_DIR_INPUT,   PIN_IRQ_NONE,           &IO_1_state,        NULL}, \
    { 58,       IO_DIR_INPUT,   PIN_IRQ_NONE,           &IO_2_state,        NULL}, \
    { 59,       IO_DIR_INPUT,   PIN_IRQ_NONE,           &IO_3_state,        NULL}, \
    { 61,       IO_DIR_OUTPUT,  PIN_IRQ_NONE,           &bno_rst_state,     NULL}, \
    { 62,       IO_DIR_OUTPUT,  PIN_IRQ_NONE,           &bno_wake_state,    NULL}, \
    { 63,       IO_DIR_OUTPUT,  PIN_IRQ_NONE,           &bno_cs_state,      NULL}, \
    { 60,       IO_DIR_INPUT ,  PIN_IRQ_EDGE_FALLING,   &bno_int_state,     NULL} \
}

extern ps_gpio_context_t PsGpio_Ctx; // Contexte du driver GPIO PS

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
    } \
}

#endif /* IO_CONFIG_H */