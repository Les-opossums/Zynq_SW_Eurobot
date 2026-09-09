#include "IPC_manager.h"
#include "../CORE_ID/core_id.h" 
#include "../IRQ_MANAGER/IRQ_manager.h"
#include "../TIMER_MANAGER/timer_manager.h"   // Timer_us_Get (timeout de sync)

#include "xil_mmu.h"
#include "xpseudo_asm.h"
#include "xscugic.h"
#include "xparameters.h"
#include <string.h>

void IPC_Init(void) {
    //disable cache on OCM (Strongly Ordered, NONCACHEABLE, shareable)
    Xil_SetTlbAttributes(IPC_SHARED_MEM_ADDR,0x14de2);
    dmb(); //waits until write has finished

    // Seul CORE0 (le maitre, qui demarre en premier) doit remettre a zero la
    // zone partagee : il le fait AVANT de reveiller CORE1, donc il n'y a pas
    // de risque d'ecraser une donnee deja ecrite par l'autre coeur.
    // CORE1 ne doit surtout pas refaire ce memset : quand il boote (reveille
    // par CORE0), la structure est deja initialisee par CORE0 et CORE1 est
    // sur le point d'y ecrire son propre flag "je suis pret".
    #if THIS_CORE_ID == CORE_ID_CPU0
        memset((void *)IPC_DATA, 0, sizeof(ipc_shared_data_t));
        dmb(); //waits until write has finished
    #endif
}

uint8_t IPC_SyncCores(void) {
    uint32_t t_start = Timer_us_Get();
    #if THIS_CORE_ID == CORE_ID_CPU0
        IPC_DATA->core0_ready = 1;
        dmb(); //waits until write has finished
        while (IPC_DATA->core1_ready == 0) {
            // Boucle d'attente active, bornee : evite un blocage a vie si CPU1
            // ne se synchronise jamais.
            if ((uint32_t)(Timer_us_Get() - t_start) > IPC_SYNC_TIMEOUT_US) return 0;
            dmb(); //waits until write has finished
        }
    #else
        IPC_DATA->core1_ready = 1;
        dmb(); //waits until write has finished
        while (IPC_DATA->core0_ready == 0) {
            if ((uint32_t)(Timer_us_Get() - t_start) > IPC_SYNC_TIMEOUT_US) return 0;
            dmb(); //waits until write has finished
        }
    #endif
    return 1;
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

/* ===================================================================== *
 * Envoi FIABLE non bloquant (retransmission automatique)
 * ===================================================================== *
 * Un slot par canal 'dest'. On y garde une copie de la derniere valeur voulue
 * et un drapeau "en attente". La boucle principale draine les slots via
 * IPC_RetryPending() : les commandes one-shot ne sont donc plus perdues quand
 * deux ordres se suivent trop vite, sans jamais bloquer l'emetteur. */
typedef struct {
    volatile void     *dest;
    volatile uint32_t *flag_valid;
    volatile uint32_t *flag_ack;
    uint8_t            buf[IPC_RELIABLE_MAX_SIZE];
    size_t             size;
    uint8_t            pending;
} ipc_reliable_slot_t;

static ipc_reliable_slot_t s_reliable_slots[IPC_RELIABLE_MAX_SLOTS];

/* Slot associe a 'dest' (existant ou premier libre), NULL si table pleine. */
static ipc_reliable_slot_t *reliable_slot_for(volatile void *dest) {
    ipc_reliable_slot_t *free_slot = NULL;
    for (int i = 0; i < IPC_RELIABLE_MAX_SLOTS; i++) {
        if (s_reliable_slots[i].dest == dest) return &s_reliable_slots[i];
        if (free_slot == NULL && s_reliable_slots[i].dest == NULL) {
            free_slot = &s_reliable_slots[i];
        }
    }
    return free_slot;
}

int IPC_SendReliable(volatile void *dest, volatile uint32_t *flag_valid,
                     volatile uint32_t *flag_ack, const void *data, size_t size) {
    if (size > IPC_RELIABLE_MAX_SIZE) {
        /* Payload trop gros pour le retry : envoi best-effort direct. */
        (void)IPC_SendToOtherCore(data, size, dest, flag_valid, flag_ack);
        return -1;
    }

    ipc_reliable_slot_t *s = reliable_slot_for(dest);
    if (s == NULL) {
        /* Table pleine (ne devrait pas arriver) : envoi best-effort direct. */
        return IPC_SendToOtherCore(data, size, dest, flag_valid, flag_ack);
    }

    s->dest       = dest;
    s->flag_valid = flag_valid;
    s->flag_ack   = flag_ack;
    memcpy(s->buf, data, size);
    s->size = size;

    if (IPC_SendToOtherCore(s->buf, s->size, s->dest, s->flag_valid, s->flag_ack)) {
        s->pending = 0;
        return 1;
    }
    s->pending = 1; /* canal occupe : sera re-emis par IPC_RetryPending() */
    return 0;
}

void IPC_RetryPending(void) {
    for (int i = 0; i < IPC_RELIABLE_MAX_SLOTS; i++) {
        ipc_reliable_slot_t *s = &s_reliable_slots[i];
        if (s->dest != NULL && s->pending) {
            if (IPC_SendToOtherCore(s->buf, s->size, s->dest, s->flag_valid, s->flag_ack)) {
                s->pending = 0;
            }
        }
    }
}
