/**
 * @file BNO085.h
 * @brief Driver BNO085 via SPI pour Zynq7000
 *
 * Protocole : SHTP (Sensor Hub Transport Protocol) sur SPI Mode 3
 * Compatible bare-metal Xilinx SDK / Vitis
 *
 * GPIO sur PS GPIO (via l'instance partagée ps_gpio_context_t / PsGpio_Ctx) :
 *   CS   → broche PS GPIO 63 (sortie, actif bas)
 *   RST  → broche PS GPIO 61 (sortie, actif bas)
 *   INT  → broche PS GPIO 60 (entrée, actif bas, EDGE_FALLING)
 *
 * SPI sur PS SPI0 (XSpiPs, mode 3, CS manuel)
 *
 * Le driver ne possède pas sa propre instance XGpioPs : il reçoit un
 * pointeur vers le contexte GPIO PS partagé (PsGpio_Ctx), déjà initialisé
 * par l'IO_Manager avant que ce driver ne soit initialisé. Voir l'ordre
 * dans IO_DEVICE_TABLE (DEV_TYPE_GPIO_PS avant DEV_TYPE_IMU_BNO085).
 */

#ifndef BNO085_H
#define BNO085_H

#include "xspips.h"
#include "xparameters.h"
#include "xil_types.h"
#include "sleep.h"
#include "../DRIVER_PS_GPIO/driver_ps_gpio.h"   /* pour ps_gpio_context_t */

/* ─── Debug ──────────────────────────────────────────────────────────────
 * Decommenter pour reactiver tous les prints de diagnostic du driver
 * (init SPI, reset, activation des rapports SH-2, erreurs...). Desactive
 * par defaut : aucun cout en flash/CPU/UART en fonctionnement normal, et
 * plus de pollution de la console partagee avec l'autre cœur.
 */
// #define BNO085_DEBUG

#if defined(BNO085_DEBUG)
#include "xil_printf.h"
#define BNO085_LOG(...) xil_printf(__VA_ARGS__)
#else
#define BNO085_LOG(...) do {} while (0)
#endif

/* ─── SPI ───────────────────────────────────────────────────────────────── */

#define BNO085_SPI_DEVICE_ID     XPAR_XSPIPS_0_DEVICE_ID

/*
 * Prescaler SPI.
 * À 100 MHz PS : PRESCALE_64 → ~1.56 MHz  (safe, BNO085 max = 3 MHz)
 * À  50 MHz PS : PRESCALE_32 → ~1.56 MHz
 * Ajustez selon votre configuration Vivado.
 */
#define BNO085_SPI_PRESCALER     XSPIPS_CLK_PRESCALE_32

/* ─── Constantes SHTP ───────────────────────────────────────────────────── */

#define SHTP_HEADER_SIZE         4U
#define SHTP_MAX_CARGO_SIZE      256U
#define SHTP_MAX_PACKET_SIZE     512 //(SHTP_HEADER_SIZE + SHTP_MAX_CARGO_SIZE)

/** Identifiants de canaux SHTP */
#define SHTP_CHAN_COMMAND         0U
#define SHTP_CHAN_EXECUTABLE      1U
#define SHTP_CHAN_CONTROL         2U
#define SHTP_CHAN_REPORTS         3U
#define SHTP_CHAN_WAKE_REPORTS    4U
#define SHTP_CHAN_GYRO_RV         5U

/* ─── Identifiants de rapports SH-2 ────────────────────────────────────── */

#define SH2_ACCELEROMETER            0x01U
#define SH2_GYROSCOPE_CALIBRATED     0x02U
#define SH2_MAGNETIC_FIELD           0x03U
#define SH2_LINEAR_ACCELERATION      0x04U
#define SH2_ROTATION_VECTOR          0x05U
#define SH2_GRAVITY                  0x06U
#define SH2_GAME_ROTATION_VECTOR     0x08U
#define SH2_GEOMAGNETIC_ROTATION     0x09U
#define SH2_GYRO_ROTATION_VECTOR     0x2AU
#define SH2_RAW_ACCELEROMETER        0x14U
#define SH2_RAW_GYROSCOPE            0x15U

/** Commandes SH-2 envoyées sur le canal Control */
#define SH2_CMD_SET_FEATURE          0xFDU
#define SH2_CMD_GET_FEATURE          0xFEU
#define SH2_CMD_PRODUCT_ID_REQ       0xF9U
#define SH2_CMD_PRODUCT_ID_RESP      0xF8U

/* ─── Codes de retour ───────────────────────────────────────────────────── */

#define BNO085_OK                 0
#define BNO085_ERR_SPI           -1
#define BNO085_ERR_GPIO          -2
#define BNO085_ERR_TIMEOUT       -3
#define BNO085_ERR_NO_DATA       -5

/* ─── Timing ────────────────────────────────────────────────────────────── */

#define BNO085_INT_TIMEOUT_US    10000U   /* 10 ms pour attendre INT        */
#define BNO085_RESET_DELAY_MS    100U     /* Délai après reset (datasheet ≥ 50 ms) */

/* ─── Structures de données ─────────────────────────────────────────────── */

/** Vecteur 3D flottant */
typedef struct {
    float x;
    float y;
    float z;
} BNO085_Vec3;

/** Quaternion d'orientation */
typedef struct {
    float real;      /* w */
    float i;         /* x */
    float j;         /* y */
    float k;         /* z */
    float accuracy;  /* rad */
} BNO085_Quaternion;

/** Données fusionnées du capteur */
typedef struct {
    BNO085_Vec3       accel;         /**< Accélération brute         [m/s²]  */
    BNO085_Vec3       linear_accel;  /**< Accélération sans gravité  [m/s²]  */
    BNO085_Vec3       gyro;          /**< Vitesse angulaire calibrée [rad/s] */
    BNO085_Vec3       mag;           /**< Champ magnétique           [µT]    */
    BNO085_Quaternion rotation;      /**< Quaternion AHRS (réf. Nord)        */
    BNO085_Quaternion game_rv;       /**< Quaternion jeu (sans magnéto)      */
    float             yaw;           /**< Cap     [°] 0–360                  */
    float             pitch;         /**< Tangage [°]                        */
    float             roll;          /**< Roulis  [°]                        */
    u8                calib_status;  /**< Statut de calibration (0–3)        */
    u8                new_data;      /**< Flag : nouvelles données dispo      */
} BNO085_Data;

/**
 * @brief Handle principal du driver.
 *
 * gpio_ctx pointe vers l'instance GPIO PS partagée (typiquement &PsGpio_Ctx),
 * déjà initialisée par l'IO_Manager. Ce driver ne possède aucune instance
 * XGpioPs en propre : il pilote CS/RST/INT directement sur l'instance
 * commune, car le séquencement CS bas/haut doit se faire à l'intérieur
 * même d'une transaction SPI, indépendamment du cycle IO_Manager_Update.
 */
typedef struct {
    XSpiPs  spi;

    ps_gpio_context_t *gpio_ctx;  /**< Instance GPIO PS partagée (PsGpio_Ctx) */
    u32     pin_cs;                /**< Broche PS GPIO — CS actif bas         */
    u32     pin_rst;                /**< Broche PS GPIO — RST actif bas        */
    u32     pin_int;                /**< Broche PS GPIO — INT actif bas        */

    u8      tx_buf[SHTP_MAX_PACKET_SIZE];
    u8      rx_buf[SHTP_MAX_PACKET_SIZE];
    u8      seq[8];     /**< Numéro de séquence par canal SHTP    */

    BNO085_Data data;
} BNO085_Dev;

/* ─── API publique ──────────────────────────────────────────────────────── */

/**
 * @brief Initialise SPI + broches GPIO PS et remet le BNO085 en état de marche.
 * @param dev       Handle alloué par l'appelant (statique ou global recommandé).
 * @param gpio_ctx  Contexte GPIO PS partagé déjà initialisé (ex: &PsGpio_Ctx).
 * @param pin_cs    Numéro de broche PS GPIO pour CS.
 * @param pin_rst   Numéro de broche PS GPIO pour RST.
 * @param pin_int   Numéro de broche PS GPIO pour INT.
 * @return BNO085_OK ou code d'erreur négatif.
 */
int BNO085_Init(BNO085_Dev *dev, ps_gpio_context_t *gpio_ctx, u32 pin_cs, u32 pin_rst, u32 pin_int);

/**
 * @brief Reset matériel du BNO085 (RST bas puis haut) et attente démarrage.
 */
int BNO085_Reset(BNO085_Dev *dev);

/**
 * @brief Active un rapport de capteur à la fréquence souhaitée.
 * @param report_id    Ex : SH2_ROTATION_VECTOR
 * @param interval_us  Intervalle en µs (ex : 10000 = 100 Hz)
 */
int BNO085_EnableReport(BNO085_Dev *dev, u8 report_id, u32 interval_us);

/**
 * @brief Lit et traite tous les paquets SHTP disponibles.
 *        À appeler régulièrement dans la boucle principale (ou déclenché
 *        par le flag posé par l'interruption sur la broche INT).
 * @return BNO085_OK si au moins un paquet traité, BNO085_ERR_NO_DATA sinon.
 */
int BNO085_Poll(BNO085_Dev *dev);

/**
 * @brief Retourne un pointeur vers les dernières données.
 */
static inline BNO085_Data *BNO085_GetData(BNO085_Dev *dev) {
    return &dev->data;
}

/**
 * @brief Retourne 1 si de nouvelles données sont disponibles, 0 sinon.
 *        Remet le flag à 0 après lecture.
 */
static inline int BNO085_DataReady(BNO085_Dev *dev) {
    int ready = (int)dev->data.new_data;
    dev->data.new_data = 0U;
    return ready;
}

#endif /* BNO085_H */