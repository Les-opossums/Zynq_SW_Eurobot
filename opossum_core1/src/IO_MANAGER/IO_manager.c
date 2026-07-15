#include "IO_manager.h"
#include "../IO_config.h"
#include "xil_printf.h"

// Instanciation de la configuration depuis le header
static const io_pin_config_t io_table[IO_CONFIG_COUNT] = IO_CONFIG_TABLE;

// Instance globale (privée à ce fichier) du driver GPIO du PS
static XGpioPs Gpio;

/*
 * Initialise le contrôleur GPIO et configure chaque broche
 * Retourne XST_SUCCESS (0) si OK, sinon une erreur.
 */
int IO_Manager_Init(void) {
    XGpioPs_Config *ConfigPtr;
    int Status;

    // Initialisation du driver
    ConfigPtr = XGpioPs_LookupConfig(XPAR_PS7_GPIO_0_DEVICE_ID);
    if (ConfigPtr == NULL) {
        return -1;
    }

    Status = XGpioPs_CfgInitialize(&Gpio, ConfigPtr, ConfigPtr->BaseAddr);
    if (Status != XST_SUCCESS) {
        return Status;
    }

    // Boucle sur toutes les IO configurées dans IO_config.h
    for (int i = 0; i < IO_CONFIG_COUNT; i++) {
        XGpioPs_SetDirectionPin(&Gpio, io_table[i].pin_number, io_table[i].direction);
        
        // Si c'est une sortie, il faut aussi activer l'Output Enable
        if (io_table[i].direction == IO_DIR_OUTPUT) {
            XGpioPs_SetOutputEnablePin(&Gpio, io_table[i].pin_number, 1);
            
            // (Optionnel) Initialiser la sortie avec la valeur actuelle de la variable
            XGpioPs_WritePin(&Gpio, io_table[i].pin_number, *(io_table[i].data_ptr));
        }
    }

    return 0; // Succès
}

/*
 * Synchronise les variables avec le matériel.
 * A appeler régulièrement dans ta boucle principale ou dans un timer.
 */
void IO_Manager_Update(void) {
    for (int i = 0; i < IO_CONFIG_COUNT; i++) {
        
        if (io_table[i].direction == IO_DIR_INPUT) {
            // Lecture du composant matériel -> écriture dans la variable
            *(io_table[i].data_ptr) = XGpioPs_ReadPin(&Gpio, io_table[i].pin_number);
        } 
        else { // IO_DIR_OUTPUT
            // Lecture de la variable -> écriture sur le composant matériel
            XGpioPs_WritePin(&Gpio, io_table[i].pin_number, *(io_table[i].data_ptr));
        }
    }
}