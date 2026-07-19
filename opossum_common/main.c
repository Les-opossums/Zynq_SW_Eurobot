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
#define ARM1_BASEADDR 0x10080000 
#define ARM1_STARTADR 0xFFFFFFF0

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
        xil_printf("[CPU0] Reveil du CPU1 a l'adresse 0x%08X...\n", ARM1_BASEADDR);
        Xil_Out32(ARM1_STARTADR, ARM1_BASEADDR);
        dmb(); // Barrière mémoire : s'assure que l'écriture est terminée
        sev(); // Send Event : Réveille le CPU 1

        // 3. (Optionnel) On peut utiliser l'IPC pour attendre que le CPU1 dise "Je suis prêt"
        // On attend que le CPU 1 confirme qu'il est vivant
        uint32_t sync = 0;
        while(sync != 0x11111111) {
            Xil_DCacheInvalidateRange((INTPTR)&IPC_DATA->core0_init_done, sizeof(uint32_t));
            sync = IPC_DATA->core0_init_done;
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
        IPC_DATA->core0_init_done = 0x11111111;
        Xil_DCacheFlushRange((INTPTR)&IPC_DATA->core0_init_done, sizeof(uint32_t));
        
    #endif

    // =========================================================

    IRQ_Manager_Start();
    IPC_SyncCores();

    App_Init(); // init spécifique à l'application du coeur en cours de compilation

    xil_printf("[CPU%d] Entree dans la boucle principale\n", THIS_CORE_ID);

    while(1){
        IO_Manager_Update();

        App_Loop(); // boucle spécifique à l'application du coeur en cours de compilation
    }

    cleanup_platform();
    return 0;
}