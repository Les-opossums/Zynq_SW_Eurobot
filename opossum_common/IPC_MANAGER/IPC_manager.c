#include "IPC_manager.h"
#include "../CORE_ID/core_id.h" 
#include "../IRQ_MANAGER/IRQ_manager.h"

#include "xil_mmu.h"
#include "xpseudo_asm.h"
#include "xscugic.h"
#include "xparameters.h"
#include <string.h>

void IPC_Init(void) {
    //disable cache on OCM (Strongly Ordered, NONCACHEABLE, shareable)
    Xil_SetTlbAttributes(IPC_SHARED_MEM_ADDR,0x14de2); 
    dmb(); //waits until write has finished

    #if THIS_CORE_ID == CORE_ID_CPU0
        memset((void *)IPC_DATA, 0, sizeof(ipc_shared_data_t));
        dmb(); //waits until write has finished
    #endif
}

void IPC_SyncCores(void) {
    #if THIS_CORE_ID == CORE_ID_CPU0
        IPC_DATA->core0_ready = 1;
        dmb(); //waits until write has finished
        while (IPC_DATA->core1_ready == 0) {
            // Boucle d'attente active
            dmb(); //waits until write has finished
        }
    #else
        IPC_DATA->core1_ready = 1;
        dmb(); //waits until write has finished
        while (IPC_DATA->core0_ready == 0) {
            // Boucle d'attente active
            dmb(); //waits until write has finished
        }
    #endif
}

static void trigger_other_core_interrupt(void) {
    #if THIS_CORE_ID == CORE_ID_CPU0
        // Send SGI to core 1
        XScuGic_SoftwareIntr(IRQ_Manager_GetInstance(), IPC_SGI_INT_ID, 0x02); // CPU1 is bit 1
    #else
        // Send SGI to core 0
        XScuGic_SoftwareIntr(IRQ_Manager_GetInstance(), IPC_SGI_INT_ID, 0x01); // CPU0 is bit 0
    #endif
}


int IPC_SendToOtherCore(const void *data, size_t size,
                        volatile void *dest,
                        volatile uint32_t *flag_valid,
                        volatile uint32_t *flag_ack) {
    if (*flag_valid && !(*flag_ack)) {
        return 0; // Pas encore consommé
    }

    memcpy((void *)(uintptr_t)dest, data, size);

    // On lève les drapeaux
    *flag_valid = 1;
    *flag_ack = 0;

    // BARRIÈRE CRITIQUE : On force l'écriture des drapeaux en OCM 
    // AVANT d'envoyer l'interruption
    __asm__ volatile("dsb sy" ::: "memory");

    // On prévient le Core 1 immédiatement
    trigger_other_core_interrupt();

    return 1; // Succès
}

// Fonction générique de réception
int IPC_CheckFromOtherCore(void *data_out, size_t size,
                           const volatile void *src,
                           volatile uint32_t *flag_valid,
                           volatile uint32_t *flag_ack) {
    if (*flag_valid && !(*flag_ack)) {

        // Copie mémoire
        memcpy(data_out, (const void *)(const volatile uint8_t *)src, size);
        __asm__ volatile("dmb sy" ::: "memory");

        *flag_ack = 1;
        *flag_valid = 0;

        return 1; // Nouvelle donnée reçue
    }
    return 0; // Rien à lire
}
