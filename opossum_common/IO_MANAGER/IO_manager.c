#include "IO_manager.h"
#include "../IO_config.h"
#include "xil_printf.h"


static io_device_t DeviceTable[] = IO_DEVICE_TABLE;
static const int NumDevices = sizeof(DeviceTable) / sizeof(io_device_t);

int IO_Manager_Init(void) {
    int Status;
    int success_count = 0;

    xil_printf("[CPU%d]IO_Manager Initialisation\n", THIS_CORE_ID);

    for(int i = 0; i<NumDevices; i++){
        io_device_t *dev = &DeviceTable[i];

        if(dev->owner != THIS_CORE && dev->owner != CORE_BOTH){
            continue;
        }

        if(dev->init != NULL){
            Status = dev->init(dev->driver_instance);
            if (Status != XST_SUCCESS) {
                dev->is_active = 0; // Marque le périphérique comme inactif en cas d'échec d'initialisation
                xil_printf("[CPU%d]Erreur d'initialisation du peripherique %d\n", THIS_CORE_ID, i);
            } else {
                dev->is_active = 1;
                success_count++;
                xil_printf("[CPU%d]Peripherique %d initialise avec succes\n", THIS_CORE_ID, i);
            }
        } else {
            dev->is_active = 1; // pas d'init nécessaire -> actif par défaut
        }

        if(dev->irq_id != 0 && dev->irq_handler != NULL){
            Status = IRQ_Manager_Connect(dev->irq_id, dev->irq_handler, dev->driver_instance);
            if (Status != XST_SUCCESS) {
                xil_printf("[CPU%d]Erreur de connexion de l'interruption pour le peripherique %d\n", THIS_CORE_ID, i);
            } else {
                xil_printf("[CPU%d]Interruption pour le peripherique %d connectee avec succes\n", THIS_CORE_ID, i);
            }
        }
    }
    xil_printf("[CPU%d]IO_Manager Initialisation terminee : %d peripheriques initialises avec succes sur %d\n", THIS_CORE_ID, success_count, NumDevices);
    return XST_SUCCESS;
}

void IO_Manager_SetDeviceState(dev_type_t type, u8 active) {
    for(int i = 0; i < NumDevices; i++){
        io_device_t *dev = &DeviceTable[i];
        
        if(dev->type == type) {
            // Demande de désactivation
            if(!active && dev->is_active) {
                if(dev->deinit != NULL) {
                    dev->deinit(dev->driver_instance); // Coupe le hardware
                }
                dev->is_active = 0; // Coupe la boucle logicielle
                xil_printf("[IO_Manager] Peripherique type %d desactive\n", type);
            }
            // Demande de réactivation
            else if (active && !dev->is_active) {
                if(dev->init != NULL) {
                    dev->init(dev->driver_instance); // Relance le hardware
                }
                dev->is_active = 1;
                xil_printf("[IO_Manager] Peripherique type %d reactive\n", type);
            }
        }
    }
}

void IO_Manager_Update(void) {
    for(int i = 0; i<NumDevices; i++){
        io_device_t *dev = &DeviceTable[i];

        // Verification de la propriete du peripherique
        if(dev->owner != THIS_CORE && dev->owner != CORE_BOTH){
            continue;
        }

        if(!dev->is_active){
            continue;
        }

        // Mise à jour du peripherique si la fonction est definie
        if(dev->update != NULL){
            dev->update(dev->driver_instance);
        }
    }
}