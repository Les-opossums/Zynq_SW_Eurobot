#include "command_list.h"
#include "interpreteur.h"

#include "../APP_ASSERV_BRIDGE/asserv_commands.h"
#include "../APP_DRIVER_BRIDGE/driver_commands.h"
#include "../SYSTEM_MANAGER/system_reset.h"
#include "../CORE_ID/core_id.h"

/*
 * Les commandes actionneurs (pinces FEETECH) ne vivent que sur CPU0
 * (cf opossum_core1/src/APP_ACTIONNEURS/feetech_Action.c — placees dans le
 * dossier du projet CPU0 et non dans opossum_common car specifiques a cette
 * application). command_list.c est compile pour les deux projets (CPU0 et
 * CPU1) : on garde donc cet include et les entrees de la table sous ce
 * meme #if, pour que le projet CPU1 ne cherche jamais ce fichier ni ces
 * symboles.
 */
#if THIS_CORE_ID == CORE_ID_CPU0
#include "../../opossum_core1/src/APP_ACTIONNEURS/feetech_Action.h"
#endif

/*
 * Table des commandes disponibles.
 *
 * Au fur et à mesure que chaque module applicatif est porté dans la
 * nouvelle architecture, décommente l'entrée correspondante ci-dessous
 * (et ajoute l'include nécessaire en haut de ce fichier).
 */

const Command Command_List[] = {
    // --- Asserv ---
    { "SETLIDAR",     Set_Lidar_Cmd},
    { "SETCAMERA1",   Set_Camera_1_Cmd},
    { "SETCAMERA2",   Set_Camera_2_Cmd},
    { "SETCAMERA3",   Set_Camera_3_Cmd},
    { "ENKALMAN",     Enable_Kalman_Cmd},
    { "LIDARNOISE",   Set_Lidar_Noise_Cmd},
    { "MOVE",         Move_Cmd},
    { "MOVESEX",      Move_Seq_Cmd},      /* corrige : pointait vers Move_Cmd */
    { "ODOSPACING",   Set_Odo_Spacing_Cmd},
    { "SET",          SET_Cmd},
    { "SET0",         SET0_Cmd},
    { "VMAX",         VMAX_Cmd},
    { "VTMAX",        VTMAX_Cmd},
    { "AMAX",         AMAX_Cmd},          /* ajoute */
    { "SPEED",        Speed_Cmd},
    { "ASPEED",       Absolute_Speed_Cmd},
    { "FREE",         FREE_Cmd},
    { "BLOCK",        BLOCK_Cmd},
    { "PWM",          PWM_Func},
    { "SPEEDTEST",    Speed_Timed_Cmd},
    { "CALIBFF",      Start_Wheel_FF_Calibration_Cmd},
    { "GETPOS",       Get_Pos_Cmd},
    { "GETODO",       Get_Odo_Cmd},       /* ajoute */
    { "ASSERVDONE",   Asserv_Done_Cmd},   /* ajoute */
    { "PDE",          Activate_Position_Sending_Func},

    // --- Pilotage generique des drivers IO_Manager (les deux cœurs) ---
    // Nom = champ .name de IO_DEVICE_TABLE (cf IO_config.h), entre
    // guillemets, insensible a la casse. Ex: DRVDIS "FEETECH", DRVEN "WS2812B".
    { "DRVEN",        Driver_Enable_Cmd},
    { "DRVDIS",       Driver_Disable_Cmd},
    { "DRVLIST",      Driver_List_Cmd},

    // --- Systeme ---
    // Reset "chaud" complet du PS (CPU0+CPU1+peripheriques), via SLCR
    // PSS_RST_CTRL : ne revient jamais, le BootROM/FSBL redemarrent comme
    // a la mise sous tension.
    { "REBOOT",       Reboot_Cmd},

    // --- Actionneurs (pinces FEETECH, CPU0 uniquement) ---
#if THIS_CORE_ID == CORE_ID_CPU0
    { "STSSEND",      Send_FEETECH_Cmd},
    { "STSGET",       Get_FEETECH_Cmd},
    { "SCSSEND",      Send_FEETECH_SCS_Cmd},
    { "SCSGET",       Get_FEETECH_SCS_Cmd},
    { "PINCE",        pince_action_cmd},
    { "PINCEDEBUG",   pince_action_debug_cmd},
    { "SET_PINCE",    setup_pince_set_pos_cmd},
#endif
    // { "SERVO",        Servo_cmd},

    // --- LED (à porter — driver WS2812B pur, pas de commande directe pour l'instant) ---
    // { "LED",          LED_cmd},

    // --- IHM / debug (déjà disponibles) ---
    { "PRINTCMD",     Print_All_CMD_Cmd},
    { "HELP",         Print_All_CMD_Cmd},
    { "TEST",         Test_Interpreteur},
    // { "VERSION",      Version_cmd},
};

const uint16_t Command_List_Length = sizeof(Command_List) / sizeof(Command_List[0]);