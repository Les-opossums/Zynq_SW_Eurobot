// --- xilinx includes ---
#include "PLATFORM/platform.h"   // Contient init_platform() et cleanup_platform()
#include "xil_io.h"     // Pour Xil_Out32
#include "xpseudo_asm.h"// Contient les macros natives sev() et dmb()
#include "sleep.h"
#include "xil_cache.h"


#include "CORE_ID/core_id.h"
#include "IO_config.h"
#include "IPC_MANAGER/IPC_manager.h"
#include "IRQ_MANAGER/IRQ_manager.h"
#include "TIMER_MANAGER/timer_manager.h"
#include "APP_COM/com_interpreter_loop.h"
#include "app_interface.h"
// temporaire pour debug
#include "APP_TEST/led_au_test.h"

#define sev() __asm__("sev")
// Adresse de depart (dans la DDR) du binaire CPU1 : doit correspondre a
// l'ORIGIN de ps7_ddr_0 dans le lscript.ld du projet opossum_core2.
#define CPU1_ENTRY_ADDR   0x10080000
// Registre special de "wake-up" lu par la boot ROM du Zynq : le CPU1,
// tenu en boucle WFE par la boot ROM, relit cette adresse a chaque SEV et
// branche dessus des qu'elle est non nulle.
#define CPU1_WAKE_REG     0xFFFFFFF0

// definition du callback quand l'un ou l'autre des cores reçoit une interruption SGI de l'autre core
void On_IPC_Message_Received(void *callback_ref) {
    #if THIS_CORE_ID == CORE_ID_CPU0
        // On est sur le core 0, on a reçu une interruption du core 1
    #else
        // On est sur le core 1, on a reçu une interruption du core 0
    #endif
}

int main(void){
    init_platform();
    
    IPC_Init();
    IRQ_Manager_Init();
    Timer_Manager_Init();

    #if THIS_CORE_ID == CORE_ID_CPU0
        // Le Global Timer est partage physiquement par les 2 cœurs : on ne le
        // (re)demarre qu'une seule fois, ici, AVANT le reveil de CPU1, pour
        // que les deux cœurs partagent la meme base de temps microseconde
        // (utilisee par TIMING_MEASURE dans asserv_loop.c).
        Timer_us_Init();
    #endif

    IRQ_Manager_Connect(IPC_SGI_INT_ID, On_IPC_Message_Received, NULL);

    // =========================================================
    // DÉMARRAGE SÉQUENTIEL MAÎTRE/ESCLAVE
    // =========================================================

    #if THIS_CORE_ID == CORE_ID_CPU0
        xil_printf("\n\n=== DEMARRAGE SYSTEME (CPU0 Master) ===\n");

        // 1. Le CPU 0 s'initialise entièrement
        IO_Manager_Init();
        usleep(50000); // Laisse l'UART se vider

        // 2. Le CPU 0 réveille le CPU 1 !
        xil_printf("[CPU0] Reveil du CPU1 a l'adresse 0x%08X...\n", CPU1_ENTRY_ADDR);
        dmb(); // Barrière mémoire : s'assure que l'écriture est terminée
        Xil_Out32(CPU1_WAKE_REG, CPU1_ENTRY_ADDR);
        sev(); // Send Event : Réveille le CPU 1

        // 3. On attend que le CPU 1 confirme qu'il a fini sa propre init
        // (IPC_DATA est en zone OCM non-cacheable, cf IPC_Init(), donc une
        // simple relecture volatile suffit, pas besoin d'invalider le cache)
        while (IPC_DATA->core1_init_done != 0x11111111) {
            // Boucle d'attente active
        }
        xil_printf("[CPU0] CPU1 en ligne !\n");

    #else
        // ---------------------------------------------------------
        // CODE DU CPU 1
        // Quand le CPU 1 arrive ici, c'est que le CPU 0 vient de le réveiller (ou Vitis)
        // ---------------------------------------------------------
        
        // 1. Le CPU 1 s'initialise
        IO_Manager_Init();
        usleep(50000);

        // 2. Le CPU 1 signale au CPU 0 qu'il a terminé
        // (IPC_DATA est en zone OCM non-cacheable, cf IPC_Init(), donc une
        // simple ecriture volatile suffit, pas besoin de flush de cache)
        IPC_DATA->core1_init_done = 0x11111111;

    #endif

    // =========================================================

    IRQ_Manager_Start();
    IPC_SyncCores();

    App_Init(); // init spécifique à l'application du coeur en cours de compilation

    xil_printf("[CPU%d] Entree dans la boucle principale\n", THIS_CORE_ID);
    int old_timer_ms1 = 0;

    while(1){
        IO_Manager_Update();

        #if THIS_CORE_ID == CORE_ID_CPU0
            // Recopie immediate des GPIO de securite lus par CORE0 (IO_Manager_Update
            // ci-dessus vient de les rafraichir) vers la zone partagee : ce sont des
            // "variables transparentes" (cf IPC_structure.h), lues en direct par
            // CORE1 dans asserv_loop.c pour couper les moteurs. Sans cette ligne,
            // IPC_DATA->AU_state restait a 0 en permanence (valeur du memset initial)
            // et l'arret d'urgence n'etait donc JAMAIS vu par CORE1.
            IPC_DATA->AU_state    = (uint32_t)AU_state;
            IPC_DATA->leash_state = (uint32_t)leash_state;
        #else

        #endif

        App_Loop(); // boucle spécifique à l'application du coeur en cours de compilation
    }

    cleanup_platform();
    return 0;
}