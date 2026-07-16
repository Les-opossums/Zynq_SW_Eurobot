#include "main.h"
#include "lib_asserv/Lib_Asserv.h"

extern volatile uint32_t sgi_debug_count;
// #define COMM_VAL (*(volatile unsigned long *)(0xFFFF0000))
extern u32 MMUTable;

int old_timer_us1 = 0;
int old_timer_ms1 = 0;

int old_timer_AU = 0;

/**
 * @brief declaration pour l'imu
 * 
 */
BNO085_Dev imu;                /* Handle IMU — global ou static main */
int        imu_ok     = 0;     /* 1 si init réussie                  */
int        imu_last_print_ms = 0;

int AU_state, previous_AU_state = 0; // Pour détecter le moment où l'on relâche l'AU


void IMU_Init(void)
{
    int ret = BNO085_Init(&imu, IO_PIN_BNO_CS, IO_PIN_BNO_RST, IO_PIN_BNO_INT);
    if (ret != BNO085_OK) {
        xil_printf("[IMU] Echec initialisation (%d) — IMU desactive\r\n", ret);
        return;
    }
 
    /*
     * Activation des rapports souhaités.
     * Intervalles recommandés pour la localisation robot :
     *   Gyroscope           → 1 ms   (1000 Hz) — critique pour intégration
     *   Accélération linéaire → 5 ms  (200 Hz)
     *   Rotation Vector     → 10 ms  (200 Hz) — orientation absolue
     *   Game Rotation Vector → 10 ms (100 Hz) — orientation sans magnéto
     */
    xil_printf("[IMU] Activation des rapports...\r\n");
    BNO085_EnableReport(&imu, SH2_GYROSCOPE_CALIBRATED,  2000U);
    // BNO085_EnableReport(&imu, SH2_LINEAR_ACCELERATION,   10000U);  
    // BNO085_EnableReport(&imu, SH2_GAME_ROTATION_VECTOR,  10000U); 
    // BNO085_EnableReport(&imu, SH2_ROTATION_VECTOR,       10000U);

    imu_ok = 1;
    xil_printf("[IMU] Initialisation OK\r\n");
}
 
int main()
{
    Xil_SetTlbAttributes(0xFFFF0000,0x14de2); 

    init_platform();
    print("CPU1: init_platform\n\r");


    int Status = SetupInterruptSystem(&InterruptController);

    if (Status != XST_SUCCESS) {
        xil_printf("Interrupt Setup Failed\r\n");
    } else {
        xil_printf("Interrupt Setup Done\r\n");
    }

    // initialise timer
    Status = Init_Timer_us1();
    if (Status != XST_SUCCESS) {
        xil_printf("CPU1: Timer initialization failed\n\r");
        return XST_FAILURE;
    }
    
    Status = IO_Manager_Init();
    if (Status != XST_SUCCESS) {
        xil_printf("IO Manager init failed\n\r");
        Status = 0;
    } else {
        xil_printf("IO Manager init done\n\r");
        Status = 0;
    }

    // initialise shared memory
    init_shared_memory();


    Init_CAN();
    Init_Asserv();
    
    IMU_Init();


    int old_timer_can_stats = 0;
    int esc_init_timer = 0;    // Chronomètre pour l'attente des ESC
    int esc_ready = 1;         // Drapeau : 1 = ESC prêts, 0 = en attente

    while(1){ 

        /*
         *    Mise à jour des variables métier avec l'état actuel des IO
         */
        IO_Manager_Update(); 


        if(Timer_ms1 - old_timer_can_stats >= 1000) {
            // Affiche les stats sur le port série
            CAN_PrintErrorStats(); 
            old_timer_can_stats = Timer_ms1;
        }

        
        if (previous_AU_state == 1 && AU_state == 0) {
            esc_ready = 0;              // Les ESC ne sont pas encore prêts
            esc_init_timer = Timer_ms1; // On lance le chronomètre
        }
        previous_AU_state = AU_state;   // Mise à jour pour le prochain tour

        
        if(AU_state == 1){
            if(Timer_ms1 - old_timer_AU >= 100) {
                // xil_printf("AU activated, stopping asserv\n\r");
                old_timer_AU = Timer_ms1;
            }
            
            if (CAN_IsEnabled()) {
                CAN_Disable();
            }

            motion_free();
            Init_CAN_MOTOR_variables();
            Init_Asserv();
            
        } else {
            // L'arrêt d'urgence est relâché
            
            if (!esc_ready) {
                // On attend 2000 millisecondes (2 secondes) que les ESC bootent
                // Ajuste cette valeur selon le temps de démarrage réel de tes ESC
                if (Timer_ms1 - esc_init_timer >= 2000) {
                    esc_ready = 1; // Le délai est écoulé
                    
                    // Optionnel mais très utile : on remet les erreurs à zéro
                    // pour effacer les ACK errors qui auraient pu se produire
                    // pendant la coupure de l'alim.
                    CAN_ResetErrorStats(); 
                }
            } 
            else {
                // Les ESC sont censés être prêts, on relance la machine !
                if (!CAN_IsEnabled()) {
                    CAN_Enable();
                }

                Asserv_Loop();
            }
        }
    }

    cleanup_platform();
    return 0;
}
