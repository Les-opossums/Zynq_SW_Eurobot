#ifndef IO_MANAGER_H
#define IO_MANAGER_H

#include "xil_types.h"
#include "../IRQ_MANAGER/IRQ_manager.h"
#include "../CORE_ID/core_id.h"

// --- definition pour l'architecture multi-coeurs ---
typedef enum {
    CORE_CPU0 = 0,
    CORE_CPU1 = 1,
    CORE_BOTH = 2
} core_owner_t;

// --- types de périphériques gérés ---
typedef enum {
    DEV_TYPE_GPIO_PS,
    DEV_TYPE_GPIO_AXI,
    DEV_TYPE_UART_AXI,
    DEV_TYPE_UART_PS,
    DEV_TYPE_I2C,
    DEV_TYPE_SPI,
    DEV_TYPE_WS2812B,
    DEV_TYPE_IMU_BNO085,
    DEV_TYPE_CAN_MOTORS,
    DEV_TYPE_ETHERNET,
    DEV_TYPE_FEETECH,
    DEV_TYPE_LIDAR_LD19
} dev_type_t;

// ---structure universelle d'un périphérique ---
typedef struct {
    const char *name;       // Nom du périphérique (pour debug)
    dev_type_t type;       // Type de périphérique
    core_owner_t owner;    // Propriétaire du périphérique
    void *driver_instance; // Pointeur vers l'instance du driver (ex: XGpio pour GPIO)

    // paramètres d'intéruption (optionnels)
    u32 irq_id;            // ID de l'interruption associée (0 si pas d'interruption)
    Xil_InterruptHandler irq_handler; // Pointeur vers la fonction de gestion de l'interruption

    volatile u8 is_active; // Indique si le périphérique est actif (1) ou inactif (0)

    //pointeurs de fonctions (méthode de l'objet)
    int (*init)(void *instance); // Fonction d'initialisation du périphérique
    void(*update)(void *instance); // Fonction de mise à jour (lecture/écriture)
    void (*deinit)(void *instance); // Fonction de désinitialisation du périphérique (optionnelle)
}io_device_t;


int IO_Manager_Init(void);
void IO_Manager_Update(void);

/**
 * @brief Active/desactive tous les peripheriques d'un dev_type_t donne.
 * Attention : plusieurs peripheriques peuvent partager le meme type
 * generique (ex. DEV_TYPE_UART_PS pour UART_COMM ET UART_FEETECH) --
 * prefere IO_Manager_SetDeviceStateByName() pour cibler un seul
 * peripherique sans ambiguite.
 */
void IO_Manager_SetDeviceState(dev_type_t type, u8 active);

/**
 * @brief Active/desactive UN peripherique precis par son nom (cf champ
 * .name de IO_DEVICE_TABLE dans IO_config.h), comparaison insensible a la
 * casse. Ignore silencieusement (avec message) si le peripherique n'existe
 * pas ou n'est pas gere par ce cœur.
 * @return 1 si le peripherique a ete trouve (et gere par ce cœur), 0 sinon.
 */
uint8_t IO_Manager_SetDeviceStateByName(const char *name, u8 active);

/**
 * @brief Imprime l'etat courant (actif/inactif) de tous les peripheriques
 * geres par CE cœur (owner == THIS_CORE ou CORE_BOTH).
 */
void IO_Manager_PrintDeviceList(void);

// Impression des temps d'execution par peripherique (min/max/avg/last).
// Toujours declaree/appelable : no-op si TIMING_MEASURE n'est pas active
// (cf opossum_common/TIMER_MANAGER/timing_stats.h).
void IO_Manager_PrintTiming(void);

/* ================================================================= *
 * Rapport d'initialisation des drivers
 * ================================================================= *
 * IO_Manager_Init() n'imprime plus rien directement : chaque cœur stocke
 * silencieusement l'etat de SES peripheriques (evite les prints entrelaces
 * entre CPU0 et CPU1 sur l'UART partage). CPU1 exporte son tableau vers
 * IPC_DATA juste avant de signaler core1_init_done ; CPU0, une fois ce
 * flag vu, imprime un rapport UNIQUE et complet pour les 2 cœurs
 * (cf sequence dans main.c).
 */
#define IO_STATUS_NAME_LEN    16
#define IO_STATUS_MAX_DEVICES 16 /* marge pour les futurs drivers/actionneurs */

typedef struct __attribute__((packed)) {
    char name[IO_STATUS_NAME_LEN];
    u8   used;     /* 1 si ce slot a ete rempli par le cœur qui le possede */
    u8   success;  /* 1 = init OK, 0 = echec */
    u8   has_irq;  /* 1 si une interruption etait a connecter */
    u8   irq_ok;   /* 1 = connexion IRQ OK (valide seulement si has_irq) */
} IO_Device_Status;

/**
 * @brief Copie l'etat d'init des peripheriques de CE cœur dans out[].
 * @param out       Tableau destination (ex: IPC_DATA->core1_driver_status).
 * @param max_count Taille de out[] (typiquement IO_STATUS_MAX_DEVICES).
 * @return Nombre d'entrees copiees.
 */
int IO_Manager_ExportStatus(IO_Device_Status *out, int max_count);

/**
 * @brief Imprime un rapport unique et complet (peripheriques CPU0 + CPU1).
 * A appeler UNIQUEMENT depuis CPU0, une fois core1_init_done vu (cf main.c) :
 * combine le tableau local de CPU0 avec celui recu de CPU1 via IPC.
 * @param other_core_status Tableau recu de CPU1 (ex: IPC_DATA->core1_driver_status).
 * @param other_count       Nombre d'entrees valides dans other_core_status.
 */
void IO_Manager_PrintCombinedInitReport(const IO_Device_Status *other_core_status, int other_count);

#endif /* IO_MANAGER_H */