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
#include "WATCHDOG_MANAGER/watchdog.h"
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
// Timeout d'attente de l'init de CPU1 au demarrage. Base sur le Global Timer
// (Timer_us_Get), qui tourne meme sans interruptions (pas encore actives ici).
// Au-dela, CPU0 bascule en MODE DEGRADE (console/comms/OTA seuls, sans asserv).
#define CPU1_INIT_TIMEOUT_US   5000000u

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
    Watchdog_Init(WATCHDOG_DEFAULT_TIMEOUT_MS);

    // Presence de CPU1 : reste a 1 sauf si CPU0 constate un timeout de demarrage
    // ou de synchronisation de CPU1 (-> mode degrade).
    u8 cpu1_online = 1u;

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
        // Reveille CPU1 : ecrit l'adresse d'entree dans le registre de wake-up
        // (0xFFFFFFF0) puis SEV. dsb() garantit que l'ecriture est terminee et
        // ordonnee avant le SEV. (Pas de cache maintenance sur cette adresse :
        // Xil_DCacheFlushRange y provoquait un data abort sur cette MMU.)
        dmb();
        Xil_Out32(CPU1_WAKE_REG, CPU1_ENTRY_ADDR);
        dsb();
        sev(); // Send Event : reveille CPU1

        // 3. Attente de l'init de CPU1, avec re-emission periodique du reveil
        // (adresse + SEV) pour couvrir un evenement rate. IPC_DATA est en OCM
        // non-cacheable (cf IPC_Init()) : relecture volatile directe.
        {
            volatile u32 spins = 0;
            u32 t_start = Timer_us_Get();
            while (IPC_DATA->core1_init_done != 0x11111111) {
                // TIMEOUT borne : si CPU1 ne signale pas son init (plantage,
                // image CPU1 absente/corrompue...), CPU0 ne bloque plus a vie.
                if ((u32)(Timer_us_Get() - t_start) > CPU1_INIT_TIMEOUT_US) {
                    cpu1_online = 0u;
                    break;
                }
                if ((++spins & 0x000FFFFFu) == 0u) {
                    Xil_Out32(CPU1_WAKE_REG, CPU1_ENTRY_ADDR);
                    dsb();
                    sev();
                }
            }
        }

        // 4. Rapport d'init. Si CPU1 est present : rapport combine (CPU0 + CPU1),
        // sans entrelacement UART (cf sequence). Sinon : rapport CPU0 seul +
        // avertissement clair (mode degrade).
        if (cpu1_online) {
            xil_printf("[CPU0] CPU1 en ligne !\n");
            IO_Manager_PrintCombinedInitReport(IPC_DATA->core1_driver_status,
                                                (int)IPC_DATA->core1_driver_status_count);
        } else {
            xil_printf("\n[CPU0] *** CPU1 ABSENT (timeout %u ms) -> MODE DEGRADE ***\n",
                       (unsigned)(CPU1_INIT_TIMEOUT_US / 1000u));
            xil_printf("[CPU0] Console / comms / OTA actifs ; asservissement indisponible.\n");
            IO_Manager_PrintCombinedInitReport(NULL, 0);
        }

    #else
        // ---------------------------------------------------------
        // CODE DU CPU 1
        // Quand le CPU 1 arrive ici, c'est que le CPU 0 vient de le réveiller (ou Vitis)
        // ---------------------------------------------------------

        // 1. Le CPU 1 s'initialise
        IO_Manager_Init();
        usleep(50000);

        // 2. Le CPU 1 exporte son rapport d'init vers IPC_DATA, PUIS signale
        // au CPU 0 qu'il a terminé (l'ordre est important : CPU0 ne doit
        // lire le rapport qu'apres avoir vu core1_init_done, cf IPC_structure.h)
        // (IPC_DATA est en zone OCM non-cacheable, cf IPC_Init(), donc une
        // simple ecriture volatile suffit, pas besoin de flush de cache)
        IPC_DATA->core1_driver_status_count =
            (uint32_t)IO_Manager_ExportStatus(IPC_DATA->core1_driver_status, IO_STATUS_MAX_DEVICES);
        IPC_DATA->core1_init_done = 0x11111111;

    #endif

    // =========================================================

    IRQ_Manager_Start();

    // Synchronisation des 2 coeurs, avec timeout (IPC_SyncCores renvoie 0 si le
    // pair ne repond pas). CPU0 ne se synchronise que s'il attend CPU1 et
    // bascule en mode degrade si la sync echoue ; CPU1 attend toujours CPU0.
#if THIS_CORE_ID == CORE_ID_CPU0
    if (cpu1_online) {
        if (!IPC_SyncCores()) {
            xil_printf("[CPU0] Sync CPU1 timeout -> MODE DEGRADE\n");
            cpu1_online = 0u;
        }
    }
#else
    (void)IPC_SyncCores();
#endif

    // Armement du watchdog. On ne surveille le heartbeat du pair que s'il est
    // effectivement present (sinon, en mode degrade, un pair fige provoquerait
    // un reboot en boucle). Le CPU1 considere toujours CPU0 (maitre) present.
#if THIS_CORE_ID == CORE_ID_CPU0
    Watchdog_Enable(cpu1_online);
#else
    Watchdog_Enable(1u);
#endif

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

        // Re-emission des commandes IPC one-shot encore en attente (non bloquant).
        IPC_RetryPending();
        // Signale que ce coeur vient de boucler (nourrit le watchdog).
        Watchdog_Kick();
    }

    cleanup_platform();
    return 0;
}