#include "../../opossum_common/app_interface.h"
#include "../../opossum_common/IPC_MANAGER/IPC_manager.h"
#include "../../opossum_common/TIMER_MANAGER/timer_manager.h"
#include "../../opossum_common/TIMER_MANAGER/timing_stats.h"
#include "../../opossum_common/IO_config.h" // IO_Manager_PrintTiming
#include "APP_ASSERV_BRIDGE/asserv_commands.h"
#include "APP_COM/com_interpreter_loop.h"
#include "APP_COM/eth_interpreter_bridge.h"
#include "APP_ACTIONNEURS/feetech_Action.h"
#include "APP_LED_INDICATOR/led_au_animation.h"
#include "xil_printf.h"
// ... autres includes spécifiques ...

#if defined(TIMING_MEASURE)
static uint32_t last_timing_print_ms;
#endif

// Test/bring-up LIDAR LD19 : print periodique (1 Hz) du dernier scan recu,
// au format Teleplot (cf LIDAR_LD19_PrintScanTeleplot() dans DRIVER_LD19).
// La fonction de print "classique" (texte, LIDAR_LD19_PrintScanUart()) est
// gardee disponible dans le driver mais n'est plus appelee ici.
// A retirer une fois la chaine (IRQ DMA + accumulation de scan) validee.
static uint32_t last_lidar_test_ms;

// Detection de front montant sur AU_state (variable locale CPU0, rafraichie
// par AU_Callback() dans IO_globals.c) pour mettre les pinces en securite
// (AU_pinces() : abandon de toute action en cours, retour a l'etat IDLE).
extern volatile int AU_state;
static int last_au_state = 0;

void App_Init(void) {
    // Initialisations spécifiques au CPU0

    // Enregistre le handler de commandes Ethernet aupres du driver ETH.
    // A appeler apres IO_Manager_Init() (qui a deja appele ETH_IO_Init()/
    // eth_driver_init()), mais avant que la boucle principale ne commence
    // a "poller" les trames entrantes (ETH_IO_Update()).
    ETH_Interpreter_Bridge_Init();
}

void App_Loop(void) {
    Com_Interpreter_Update();
    Print_Position_loop();

    // Pinces FEETECH : sequence d'init au demarrage (une fois), puis mise a
    // jour de chaque pince a chaque tour de boucle. Init_Pinces_Loop() est
    // elle-meme une machine a etats qui ne fait plus rien une fois
    // pinces_initialized == 1 : l'appeler en continu est sans effet de bord.
    Init_Pinces_Loop();
    pince_loop();

    // Mise en securite immediate des pinces sur front montant de l'AU
    // (les moteurs CAN sont deja coupes cote CORE1 via IPC_DATA->AU_state,
    // cf asserv_loop.c ; ceci fait l'equivalent pour les servos/pompes).
    if (AU_state && !last_au_state) {
        AU_pinces();
    }
    last_au_state = AU_state;

    // Indicateur visuel : fondu rouge + halo tournant tant que l'AU est
    // actif (bandeau WS2812B), eteint au relachement.
    LED_AU_Animation_Update();

    // Test/bring-up LIDAR LD19 (cf declaration de last_lidar_test_ms plus haut).
    // ts_trigger_ms() n'existe que si TIMING_MEASURE est active (cf
    // TIMER_MANAGER/timing_stats.h) : on refait le meme calcul ici a la main
    // pour ne pas en dependre.
    if ((uint32_t)((uint32_t)Timer_ms1 - last_lidar_test_ms) >= 1000U) {
        last_lidar_test_ms = (uint32_t)Timer_ms1;
        LIDAR_LD19_PrintScanTeleplot(&Lidar_Ld19_Ctx);
    }

#if defined(TIMING_MEASURE)
    // Marge temps-reel CPU0 : peripheriques IO_Manager (dont le poll BNO085)
    // + reception/interpretation UART. Impression toutes les ~1s.
    if (ts_trigger_ms(1000U, &last_timing_print_ms)) {
        IO_Manager_PrintTiming();
        Com_Interpreter_PrintTiming();
    }
#endif
}
