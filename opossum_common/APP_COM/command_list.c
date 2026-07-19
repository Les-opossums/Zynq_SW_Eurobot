#include "command_list.h"
#include "interpreteur.h"

#include "../APP_ASSERV_BRIDGE/asserv_commands.h"

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

    // --- Actionneurs (à porter) ---
    // { "STSSEND",      Send_FEETECH_Cmd},
    // { "STSGET",       Get_FEETECH_Cmd},
    // { "SCSSEND",      Send_FEETECH_SCS_Cmd},
    // { "SCSGET",       Get_FEETECH_SCS_Cmd},
    // { "PINCE",        pince_action_cmd},
    // { "PINCEDEBUG",   pince_action_debug_cmd},
    // { "SET_PINCE",    setup_pince_set_pos_cmd},
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