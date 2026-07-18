#ifndef DRIVER_BNO085_IO_H
#define DRIVER_BNO085_IO_H

#include "../DRIVER_PS_GPIO/driver_ps_gpio.h"
#include "BNO085.h"

typedef struct {
    u8  report_id;
    u32 interval_us;
} bno085_report_config_t;

typedef struct {
    BNO085_Dev dev;
    ps_gpio_context_t *gpio_ctx;
    u32 pin_cs, pin_rst, pin_int;
    const bno085_report_config_t *report_table;
    u32 num_reports;
} bno085_io_context_t;

int  BNO085_IO_Init(void *instance);
void BNO085_IO_Update(void *instance);

// Callback appelé depuis PS_GPIO_Callback (contexte ISR) — doit rester très court
void BNO085_INT_Callback(void *callback_ref);

#endif // DRIVER_BNO085_IO_H