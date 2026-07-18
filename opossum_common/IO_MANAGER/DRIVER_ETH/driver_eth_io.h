#ifndef DRIVER_ETH_IO_H
#define DRIVER_ETH_IO_H

#include "ETH_driver.h"

typedef struct {
    eth_driver_config_t config;
} eth_io_context_t;

int  ETH_IO_Init(void *instance);
void ETH_IO_Update(void *instance);

#endif /* DRIVER_ETH_IO_H */