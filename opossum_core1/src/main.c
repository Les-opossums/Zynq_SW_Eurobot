#include "main.h"

#define sev() __asm__("sev")
#define ARM1_STARTADR 0xFFFFFFF0
#define ARM1_BASEADDR 0x10080000

int old_timer_ms1 = 0;
int Status = 0;

int timer_lidar = 0;


LD19Instance LD19;

static eth_driver_config_t eth_cfg = {
    .mac_addr   = {0x00, 0x0a, 0x35, 0x00, 0x01, 0x12},
    .local_ip   = (192u << 24) | (168u << 16) | (1u << 8) | 10u,  // 192.168.1.10
    .netmask    = (255u << 24) | (255u << 16) | (255u << 8) | 0u, // 255.255.255.0
    .gateway_ip = (192u << 24) | (168u << 16) | (1u << 8) | 10u,  // pas de routeur, on met la sienne
    .peer_ip    = (192u << 24) | (168u << 16) | (1u << 8) | 20u,  // IP de ton PC
};

int AU_state,previous_AU_state = 0;

int main()
{
    init_shared_memory();
    
    u8 c;
    init_platform();


    print("ARM0: writing startaddress for ARM1\n\r");
    Xil_Out32(ARM1_STARTADR, ARM1_BASEADDR);
    dmb(); //waits until write has finished

    print("ARM0: sending the SEV to wake up ARM1\n\r");
    sev();

    Status = SetupInterruptSystem(&InterruptController);
    if (Status != XST_SUCCESS) {
        xil_printf("Interrupt Setup Failed\r\n");
    } else {
        xil_printf("Interrupt Setup Done\r\n");
    }


    Status = UART_Init();
    if (Status != XST_SUCCESS) {
        xil_printf("UART init failed\n\r");
        Status = 0;
    } else {
        xil_printf("UART init done\n\r");
        Status = 0;
    }

    // feetech init
    Init_Com_FEETECH();
    Status = UART1_Init();
    if (Status != XST_SUCCESS) {
        xil_printf("UART1 init failed\n\r");
        Status = 0;
    } else {
        xil_printf("UART1 init done\n\r");
        Status = 0;
    }

    Status = Init_Timer_ms1();
    if (Status != XST_SUCCESS) {
        xil_printf("Timer init failed\n\r");
        Status = 0;
    } else {
        xil_printf("Timer init done\n\r");
        Status = 0;
    }

    Status = UART_PL_Init();
    if (Status != XST_SUCCESS) {
        xil_printf("UART PL init failed\n\r");
        Status = 0;
    } else {
        xil_printf("UART PL init done\n\r");
        Status = 0;
    }

    Std_Com_Init();

    Status = eth_driver_init(&eth_cfg);
    if (Status != XST_SUCCESS) {
        xil_printf("Ethernet driver init failed, code=%d\n\r", Status);
        Status = 0;
    } else {
        xil_printf("Ethernet driver init done\n\r");
        Status = 0;
    }
    eth_driver_set_cmd_handler(on_eth_command_received);

    
    uint32_t au_recovery_timer = 0;       // Timer pour les 2 secondes
    uint8_t au_recovering = 0;            // Flag : 1 = on est en train d'attendre les 2s


    ws2812b_init();

    init_shared_memory();

    xil_printf("Init done\n\r");

    while(1){
        
        /* 
         *    Lecture des caractères reçus sur l'UART
         *    Si un caractère est reçu, on le passe à l'interpréteur.
         */
        if (Get_Std_In(&c)) {
            Interp(c);
        }

        /* 
         *    Gestion de l'Ethernet
         *    On appelle cette fonction régulièrement pour traiter les trames entrantes.
        */
        eth_driver_poll();

        /*
         *    Gestion de la communication standard
         */
        Std_Com_Loop();

        /*
         *    Mise à jour des variables métier avec l'état actuel des IO
         */
        IO_Manager_Update(); 

        /*
         *    Gestion du ruban led WS2812B
         */
        LED_loop();

        
        /*
         *    print de Robotdata 
         */
        Print_Position_loop();


        /*
         *    Gestion de l'AU et des actionneurs
            *   Si l'AU est actif, on bloque les actionneurs et on met le mode AU sur les LEDs.
            *   Si l'AU est relâché, on attend 2 secondes avant de reprendre les actionneurs.
         */
        if(AU_state){
            // 1. Détection de l'appui sur l'AU
            if(previous_AU_state == 0){
                xil_printf("AU %d\n\r", AU_state);
                eth_send_frame(ETH_MSG_AU, &AU_state, sizeof(AU_state));
            }

            // 2. On est en train d'appuyer sur l'AU, donc on bloque les actionneurs et on met le mode AU sur les LEDs
            LED_AU();
            AU_pinces();
            
            au_recovering = 0; // Si on appuie sur l'AU pendant qu'on attendait les 2s, on annule l'attente

        }else{
            // 1. Détection du relâchement de l'AU
            if (previous_AU_state == 1) {
                au_recovering = 1;              // On lance la phase de récupération
                au_recovery_timer = Timer_ms1;  // On enregistre l'heure de départ
                printf("AU relâché. Attente de 2 secondes avant reprise...\n");
            }

            // 2. Gestion de l'attente
            if (au_recovering) {
                // Optionnel : tu pourrais mettre un mode de LED spécifique ici (ex: clignotement rapide)
                if (Timer_ms1 - au_recovery_timer >= 3000) {
                    au_recovering = 0; // Les 2 secondes sont écoulées !
                    printf("Reprise des actionneurs !\n");
                }
            }

            // 3. Exécution classique si on n'est PAS en phase de récupération
            if (!au_recovering) {
                // 1. On refait l'init des pinces, on est sur que les cartes slaves sont correctement redémarrées si on sort du recovering
                Init_Pinces_Loop();

                // 2. On exécute la boucle de com avec les cartes FEETECH
                FEETECH_Loop();

                // 3. On exécute la boucle de gestion des actionneurs
                pince_loop();
            }

            LED_CLASSIC_MODE();
        }
        previous_AU_state = AU_state;
        IHM_loop();
    }
    cleanup_platform();
    return 0;
}
