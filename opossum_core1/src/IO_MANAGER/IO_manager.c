#include "IO_manager.h"
#include "../IO_config.h"

// Instanciation du tableau
static const io_pin_config_t io_table[] = IO_CONFIG_TABLE;

// Calcul automatique du nombre d'IO en fonction des macros activées !
static const int ACTUAL_IO_COUNT = (sizeof(io_table) / sizeof(io_table[0]));

static XGpioPs Gpio;

int IO_Manager_Init(void) {
    XGpioPs_Config *ConfigPtr = XGpioPs_LookupConfig(XPAR_PS7_GPIO_0_DEVICE_ID);
    if (ConfigPtr == NULL) return -1;

    if (THIS_CORE == GPIO_MASTER_CORE) {
        int Status = XGpioPs_CfgInitialize(&Gpio, ConfigPtr, ConfigPtr->BaseAddr);
        if (Status != XST_SUCCESS) return Status;
    } else {
        Gpio.GpioConfig.BaseAddr = ConfigPtr->BaseAddr;
        Gpio.IsReady = XIL_COMPONENT_IS_READY;
    }

    // On utilise ACTUAL_IO_COUNT au lieu du vieux define
    for (int i = 0; i < ACTUAL_IO_COUNT; i++) {
        
        if (io_table[i].owner == THIS_CORE || (io_table[i].owner == CORE_BOTH && THIS_CORE == GPIO_MASTER_CORE)) {
            XGpioPs_SetDirectionPin(&Gpio, io_table[i].pin_number, io_table[i].direction);
            
            if (io_table[i].direction == IO_DIR_OUTPUT) {
                XGpioPs_SetOutputEnablePin(&Gpio, io_table[i].pin_number, 1);
                XGpioPs_WritePin(&Gpio, io_table[i].pin_number, *(io_table[i].data_ptr));
            }
        }
    }
    return 0;
}

void IO_Manager_Update(void) {
    // On utilise ACTUAL_IO_COUNT ici aussi
    for (int i = 0; i < ACTUAL_IO_COUNT; i++) {
        
        if (io_table[i].owner == THIS_CORE || io_table[i].owner == CORE_BOTH) {
            if (io_table[i].direction == IO_DIR_INPUT) {
                *(io_table[i].data_ptr) = XGpioPs_ReadPin(&Gpio, io_table[i].pin_number);
            } else { 
                XGpioPs_WritePin(&Gpio, io_table[i].pin_number, *(io_table[i].data_ptr));
            }
        }
    }
}