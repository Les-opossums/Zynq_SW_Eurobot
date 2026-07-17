#include "IO_manager.h"
#include "IO_config.h"
#include "xil_printf.h"

#ifndef THIS_CORE
    #error "THIS_CORE n'est pas défini lors de la compilation !"
#endif

static io_device_t DeviceTable[] = IO_DEVICE_TABLE;
static const int NumDevices = sizeof(DeviceTable) / sizeof(io_device_t);

int IO_Manager_Init(void) {
    int Status;
    int success_count = 0;

    xil_printf("[CPU%d]IO_Manager Initialisation\n", THIS_CORE);

    for(int i = 0; i<NumDevices; i++){
        io_device_t *dev = &DeviceTable[i];

        // 1 - Vérification de la propriété du périphérique
        if(dev->owner != THIS_CORE && dev->owner != CORE_BOTH){
            continue;
        }

        // 2 - Initialisation du périphérique
        if(dev->init != NULL){
            Status = dev->init(dev->driver_instance);
            if (Status != XST_SUCCESS) {
                xil_printf("[CPU%d]Erreur d'initialisation du périphérique %d\n", THIS_CORE, i);
            } else {
                success_count++;
                xil_printf("[CPU%d]Périphérique %d initialisé avec succès\n", THIS_CORE, i);
            }
        }

        // 3 - Gestion des interruptions si nécessaire
        if(dev->irq_id != 0 && dev->irq_handler != NULL){
            Status = IRQ_Manager_Connect(dev->irq_id, dev->irq_handler, dev->driver_instance);
            if (Status != XST_SUCCESS) {
                xil_printf("[CPU%d]Erreur de connexion de l'interruption pour le périphérique %d\n", THIS_CORE, i);
            } else {
                xil_printf("[CPU%d]Interruption pour le périphérique %d connectée avec succès\n", THIS_CORE, i);
            }
        }
        success_count++;
    }
    xil_printf("[CPU%d]IO_Manager Initialisation terminée : %d périphériques initialisés avec succès sur %d\n", THIS_CORE, success_count, NumDevices);
    return XST_SUCCESS;
}

void IO_Manager_Update(void) {
    for(int i = 0; i<NumDevices; i++){
        io_device_t *dev = &DeviceTable[i];

        // Vérification de la propriété du périphérique
        if(dev->owner != THIS_CORE && dev->owner != CORE_BOTH){
            continue;
        }

        // Mise à jour du périphérique si la fonction est définie
        if(dev->update != NULL){
            dev->update(dev->driver_instance);
        }
    }
}