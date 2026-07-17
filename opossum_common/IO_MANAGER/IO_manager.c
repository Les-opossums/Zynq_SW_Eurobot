#include "IO_manager.h"
#include "../IO_config.h"
#include "xil_printf.h"

// --- declarations des differents contextes de peripheriques (ex: GPIO PS, GPIO AXI, UART AXI, etc.) ---
ps_gpio_context_t PsGpio_Ctx; // Contexte du driver GPIO PS

led_color_t Led_Buffer[NBR_LED];
ws2812b_context_t Ws2812b_Ctx = {
    .base_addr = WS2812B_BASEADDR,
    .num_leds = NBR_LED,
    .led_buffer = Led_Buffer,
    .refresh_period_ms = 10
};

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
                xil_printf("[CPU%d]Erreur d'initialisation du peripherique %d\n", THIS_CORE_ID, i);
            } else {
                success_count++;
                xil_printf("[CPU%d]Peripherique %d initialise avec succes\n", THIS_CORE_ID, i);
            }
        }

        if(dev->irq_id != 0 && dev->irq_handler != NULL){
            Status = IRQ_Manager_Connect(dev->irq_id, dev->irq_handler, dev->driver_instance);
            if (Status != XST_SUCCESS) {
                xil_printf("[CPU%d]Erreur de connexion de l'interruption pour le peripherique %d\n", THIS_CORE_ID, i);
            } else {
                xil_printf("[CPU%d]Interruption pour le peripherique %d connectee avec succes\n", THIS_CORE_ID, i);
            }
        }
        // ligne "success_count++;" supprimée d'ici
    }
    xil_printf("[CPU%d]IO_Manager Initialisation terminee : %d peripheriques initialises avec succes sur %d\n", THIS_CORE_ID, success_count, NumDevices);
    return XST_SUCCESS;
}

void IO_Manager_Update(void) {
    for(int i = 0; i<NumDevices; i++){
        io_device_t *dev = &DeviceTable[i];

        // Verification de la propriete du peripherique
        if(dev->owner != THIS_CORE && dev->owner != CORE_BOTH){
            continue;
        }

        // Mise à jour du peripherique si la fonction est definie
        if(dev->update != NULL){
            dev->update(dev->driver_instance);
        }
    }
}