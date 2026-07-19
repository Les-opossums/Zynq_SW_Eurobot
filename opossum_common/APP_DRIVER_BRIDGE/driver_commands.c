#include "driver_commands.h"
#include "../APP_COM/interpreteur.h"
#include "../IO_config.h" // IO_Manager_SetDeviceStateByName / IO_Manager_PrintDeviceList

#define DRIVER_NAME_MAX_LEN 32

uint8_t Driver_Enable_Cmd(void) {
    char name[DRIVER_NAME_MAX_LEN];
    if (Get_Param_String(name, sizeof(name)) == 0) {
        return PARAM_ERROR_CODE;
    }
    if (!IO_Manager_SetDeviceStateByName(name, 1)) {
        return PARAM_ERROR_CODE;
    }
    return 0;
}

uint8_t Driver_Disable_Cmd(void) {
    char name[DRIVER_NAME_MAX_LEN];
    if (Get_Param_String(name, sizeof(name)) == 0) {
        return PARAM_ERROR_CODE;
    }
    if (!IO_Manager_SetDeviceStateByName(name, 0)) {
        return PARAM_ERROR_CODE;
    }
    return 0;
}

uint8_t Driver_List_Cmd(void) {
    IO_Manager_PrintDeviceList();
    return 0;
}
