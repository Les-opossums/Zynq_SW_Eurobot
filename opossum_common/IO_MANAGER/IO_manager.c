#include "IO_manager.h"
#include "../IO_config.h"
#include "xil_printf.h"
#include "../TIMER_MANAGER/timing_stats.h"

static io_device_t DeviceTable[] = IO_DEVICE_TABLE;
static const int NumDevices = sizeof(DeviceTable) / sizeof(io_device_t);

#if defined(TIMING_MEASURE)
/* Un TimingStats par peripherique de la table : generique, couvre donc
 * automatiquement tout nouveau driver ajoute a IO_DEVICE_TABLE (ex: futurs
 * actionneurs), sans instrumentation manuelle supplementaire. */
static TimingStats DevTiming[sizeof(DeviceTable) / sizeof(io_device_t)];
static TimingStats IoUpdateTotal;
#endif

int IO_Manager_Init(void) {
    int Status;
    int success_count = 0;
    int core_devices = 0; // Compte les périphériques assignés à ce cœur

    xil_printf("\n\n[CPU%d] === Initialisation IO_Manager ===\n", THIS_CORE_ID);

    for(int i = 0; i < NumDevices; i++){
        io_device_t *dev = &DeviceTable[i];

        // Vérification de la propriété du périphérique
        if(dev->owner != THIS_CORE && dev->owner != CORE_BOTH){
            continue;
        }

        core_devices++;
        // Récupération du nom (avec sécurité si oublié dans la config)
        const char* dev_name = (dev->name != NULL) ? dev->name : "NON_NOMME";

        // --- 1. Initialisation du périphérique ---
        if(dev->init != NULL){
            Status = dev->init(dev->driver_instance);
            if (Status != XST_SUCCESS) {
                dev->is_active = 0; 
                xil_printf("    [CPU%d] [FAIL] %-12s : Erreur init\n", THIS_CORE_ID, dev_name);
            } else {
                dev->is_active = 1;
                success_count++;
                xil_printf("    [CPU%d] [ OK ] %-12s : Initialise\n", THIS_CORE_ID, dev_name);
            }
        } else {
            // Pas de fonction d'init définie, on considère qu'il est actif
            dev->is_active = 1; 
            success_count++;
            xil_printf("    [CPU%d] [ OK ] %-12s : Actif (pas d'init requise)\n", THIS_CORE_ID, dev_name);
        }

        // --- 2. Connexion de l'interruption ---
        if(dev->irq_id != 0 && dev->irq_handler != NULL){
            Status = IRQ_Manager_Connect(dev->irq_id, dev->irq_handler, dev->driver_instance);
            if (Status != XST_SUCCESS) {
                xil_printf("    [CPU%d]   -> [FAIL] Interruption ID %lu\n", THIS_CORE_ID, dev->irq_id);
            } else {
                xil_printf("    [CPU%d]   -> [ OK ] Interruption ID %lu connectee\n", THIS_CORE_ID, dev->irq_id);
            }
        }
    }
    
    xil_printf("[CPU%d] === IO_Manager Pret : %d/%d drivers actifs ===\n\n\n", THIS_CORE_ID, success_count, core_devices);
    return XST_SUCCESS;
}

void IO_Manager_SetDeviceState(dev_type_t type, u8 active) {
    for(int i = 0; i < NumDevices; i++){
        io_device_t *dev = &DeviceTable[i];
        
        if(dev->type == type) {
            const char* dev_name = (dev->name != NULL) ? dev->name : "NON_NOMME";

            // Demande de désactivation
            if(!active && dev->is_active) {
                if(dev->deinit != NULL) {
                    dev->deinit(dev->driver_instance); // Coupe le hardware
                }
                dev->is_active = 0; // Coupe la boucle logicielle
                xil_printf("[IO_Manager] %s desactive\n", dev_name);
            }
            // Demande de réactivation
            else if (active && !dev->is_active) {
                if(dev->init != NULL) {
                    dev->init(dev->driver_instance); // Relance le hardware
                }
                dev->is_active = 1;
                xil_printf("[IO_Manager] %s reactive\n", dev_name);
            }
        }
    }
}

void IO_Manager_Update(void) {
#if defined(TIMING_MEASURE)
    T_START(io_update_total);
#endif

    for(int i = 0; i < NumDevices; i++){
        io_device_t *dev = &DeviceTable[i];

        // Verification de la propriete du peripherique
        if(dev->owner != THIS_CORE && dev->owner != CORE_BOTH){
            continue;
        }

        // On ignore les périphériques désactivés (ex: lors d'un Arrêt d'Urgence)
        if(!dev->is_active){
            continue;
        }

        // Mise à jour du peripherique si la fonction est definie
        if(dev->update != NULL){
#if defined(TIMING_MEASURE)
            uint32_t _t_dev = Timer_us1;
            dev->update(dev->driver_instance);
            ts_update(&DevTiming[i], Timer_us1 - _t_dev);
#else
            dev->update(dev->driver_instance);
#endif
        }
    }

#if defined(TIMING_MEASURE)
    T_STOP(io_update_total, IoUpdateTotal);
#endif
}

void IO_Manager_PrintTiming(void) {
#if defined(TIMING_MEASURE)
    xil_printf("--- Timing IO_Manager (CPU%d) ---\r\n", THIS_CORE_ID);
    for (int i = 0; i < NumDevices; i++) {
        const char *dev_name = (DeviceTable[i].name != NULL) ? DeviceTable[i].name : "NON_NOMME";
        ts_print_one(dev_name, &DevTiming[i]);
    }
    ts_print_one("io_total", &IoUpdateTotal);
#endif
}