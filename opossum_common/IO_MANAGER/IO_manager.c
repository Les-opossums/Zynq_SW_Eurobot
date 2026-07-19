#include "IO_manager.h"
#include "../IO_config.h"
#include "xil_printf.h"
#include "../TIMER_MANAGER/timing_stats.h"
#include <string.h>

static io_device_t DeviceTable[] = IO_DEVICE_TABLE;
static const int NumDevices = sizeof(DeviceTable) / sizeof(io_device_t);

// Etat d'init de CE cœur, stocke silencieusement (cf IO_Manager_ExportStatus /
// IO_Manager_PrintCombinedInitReport, et le rapport unique imprime par CPU0
// dans main.c une fois les 2 cœurs synchronises).
static IO_Device_Status LocalStatus[IO_STATUS_MAX_DEVICES];

#if defined(TIMING_MEASURE)
/* Un TimingStats par peripherique de la table : generique, couvre donc
 * automatiquement tout nouveau driver ajoute a IO_DEVICE_TABLE (ex: futurs
 * actionneurs), sans instrumentation manuelle supplementaire. */
static TimingStats DevTiming[sizeof(DeviceTable) / sizeof(io_device_t)];
static TimingStats IoUpdateTotal;
#endif

int IO_Manager_Init(void) {
    int Status;

    for(int i = 0; i < NumDevices; i++){
        io_device_t *dev = &DeviceTable[i];

        // Vérification de la propriété du périphérique
        if(dev->owner != THIS_CORE && dev->owner != CORE_BOTH){
            continue;
        }

        // Récupération du nom (avec sécurité si oublié dans la config)
        const char* dev_name = (dev->name != NULL) ? dev->name : "NON_NOMME";

        // Slot de rapport (best-effort : silencieusement ignore au-dela de
        // IO_STATUS_MAX_DEVICES, l'init materielle continue normalement).
        IO_Device_Status *st = (i < IO_STATUS_MAX_DEVICES) ? &LocalStatus[i] : NULL;
        if (st) {
            st->used = 1U;
            strncpy(st->name, dev_name, IO_STATUS_NAME_LEN - 1);
            st->name[IO_STATUS_NAME_LEN - 1] = '\0';
            st->has_irq = (dev->irq_id != 0 && dev->irq_handler != NULL) ? 1U : 0U;
        }

        // --- 1. Initialisation du périphérique ---
        if(dev->init != NULL){
            Status = dev->init(dev->driver_instance);
            dev->is_active = (Status == XST_SUCCESS) ? 1U : 0U;
        } else {
            // Pas de fonction d'init définie, on considère qu'il est actif
            dev->is_active = 1U;
        }
        if (st) st->success = dev->is_active;

        // --- 2. Connexion de l'interruption ---
        if(dev->irq_id != 0 && dev->irq_handler != NULL){
            Status = IRQ_Manager_Connect(dev->irq_id, dev->irq_handler, dev->driver_instance);
            if (st) st->irq_ok = (Status == XST_SUCCESS) ? 1U : 0U;
        }
    }

    return XST_SUCCESS;
}

int IO_Manager_ExportStatus(IO_Device_Status *out, int max_count) {
    int n = (NumDevices < max_count) ? NumDevices : max_count;
    if (n > IO_STATUS_MAX_DEVICES) n = IO_STATUS_MAX_DEVICES;
    if (n > 0) memcpy(out, LocalStatus, (size_t)n * sizeof(IO_Device_Status));
    return n;
}

void IO_Manager_PrintCombinedInitReport(const IO_Device_Status *other_core_status, int other_count) {
    int local_ok = 0, local_total = 0, other_ok = 0, other_total = 0;

    xil_printf("\r\n=== Rapport d'initialisation des drivers (CPU0 + CPU1) ===\r\n");

    for (int i = 0; i < NumDevices; i++) {
        const IO_Device_Status *st = NULL;

        if (i < IO_STATUS_MAX_DEVICES && LocalStatus[i].used) {
            st = &LocalStatus[i];
            local_total++;
            if (st->success) local_ok++;
        } else if (other_core_status != NULL && i < other_count && other_core_status[i].used) {
            st = &other_core_status[i];
            other_total++;
            if (st->success) other_ok++;
        }

        if (st == NULL) {
            const char *dev_name = (DeviceTable[i].name != NULL) ? DeviceTable[i].name : "NON_NOMME";
            xil_printf("  [ ?? ] %-12s : non traite (owner inconnu)\r\n", dev_name);
            continue;
        }

        xil_printf("  [%s] %-12s", st->success ? " OK " : "FAIL", st->name);
        if (st->has_irq) {
            xil_printf("  IRQ:%s", st->irq_ok ? "OK" : "FAIL");
        }
        xil_printf("\r\n");
    }

    xil_printf("--- CPU0 : %d/%d actifs | CPU1 : %d/%d actifs ---\r\n",
               local_ok, local_total, other_ok, other_total);
    xil_printf("============================================================\r\n\r\n");
}

// Applique le changement d'etat a UN device precis (partage par les deux
// fonctions publiques ci-dessous : par type -- qui peut viser plusieurs
// devices a la fois, ex. plusieurs CAN -- ou par nom -- qui n'en vise
// jamais qu'un seul, sans ambiguite meme si plusieurs devices partagent le
// meme dev_type_t generique, ex. DEV_TYPE_UART_PS pour UART_COMM et
// UART_FEETECH).
static void io_manager_apply_device_state(io_device_t *dev, u8 active) {
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

void IO_Manager_SetDeviceState(dev_type_t type, u8 active) {
    for(int i = 0; i < NumDevices; i++){
        io_device_t *dev = &DeviceTable[i];
        if(dev->type == type) {
            io_manager_apply_device_state(dev, active);
        }
    }
}

// Comparaison de chaines insensible a la casse, sans dependance a ctype.h
// (juste de quoi comparer des noms de devices ASCII simples type "FEETECH").
static uint8_t io_manager_name_eq_ci(const char *a, const char *b) {
    while (*a != '\0' && *b != '\0') {
        char ca = *a, cb = *b;
        if (ca >= 'a' && ca <= 'z') ca = (char)(ca - 'a' + 'A');
        if (cb >= 'a' && cb <= 'z') cb = (char)(cb - 'a' + 'A');
        if (ca != cb) return 0;
        a++; b++;
    }
    return (*a == '\0' && *b == '\0');
}

uint8_t IO_Manager_SetDeviceStateByName(const char *name, u8 active) {
    for (int i = 0; i < NumDevices; i++) {
        io_device_t *dev = &DeviceTable[i];
        if (dev->name == NULL || !io_manager_name_eq_ci(dev->name, name)) {
            continue;
        }
        if (dev->owner != THIS_CORE && dev->owner != CORE_BOTH) {
            xil_printf("[IO_Manager] %s n'est pas gere par ce coeur (CPU%d)\r\n", dev->name, THIS_CORE_ID);
            return 0;
        }
        io_manager_apply_device_state(dev, active);
        return 1;
    }
    xil_printf("[IO_Manager] Peripherique inconnu : %s\r\n", name);
    return 0;
}

void IO_Manager_PrintDeviceList(void) {
    xil_printf("\r\n=== Etat des drivers (CPU%d) ===\r\n", THIS_CORE_ID);
    for (int i = 0; i < NumDevices; i++) {
        io_device_t *dev = &DeviceTable[i];
        if (dev->owner != THIS_CORE && dev->owner != CORE_BOTH) {
            continue; // uniquement les peripheriques geres par ce coeur
        }
        const char *dev_name = (dev->name != NULL) ? dev->name : "NON_NOMME";
        xil_printf("  [%s] %s\r\n", dev->is_active ? " ON " : "OFF ", dev_name);
    }
    xil_printf("================================\r\n\r\n");
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