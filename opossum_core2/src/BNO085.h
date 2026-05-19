/**
 * @file bno085.h
 * @brief Driver BNO085 via SPI pour Zynq7000 (PS SPI)
 *
 * Protocole : SHTP (Sensor Hub Transport Protocol) sur SPI Mode 3
 * Compatible bare-metal Xilinx SDK / Vitis
 *
 * Branchements suggérés (MIO) :
 *   MOSI  -> MIO[xx]  (SPI0 MOSI)
 *   MISO  -> MIO[xx]  (SPI0 MISO)
 *   SCLK  -> MIO[xx]  (SPI0 CLK)
 *   CS    -> MIO[xx]  (GPIO, contrôle manuel)
 *   INT   -> MIO[xx]  (GPIO entrée, active low)
 *   RST   -> MIO[xx]  (GPIO sortie, active low)
 *   WAKE  -> MIO[xx]  (GPIO sortie, PS0 = 0 pour mode SPI)
 */

#ifndef BNO085_H
#define BNO085_H

#include "xspips.h"
#include "xgpiops.h"
#include "xparameters.h"
#include "xil_types.h"
#include "sleep.h"

/* ─── Configuration matérielle ─────────────────────────────────────────── */

#define BNO085_SPI_DEVICE_ID     XPAR_XSPIPS_0_DEVICE_ID
#define BNO085_SPI_CLK_HZ        1000000U   /* 1 MHz  (max 3 MHz) */

#define BNO085_GPIO_DEVICE_ID    XPAR_XGPIOPS_0_DEVICE_ID

/*
 * Adaptez ces numéros de pin MIO à votre schéma de câblage.
 * Repérez-les dans votre Vivado Block Design (Processing System).
 */
#define BNO085_PIN_CS            54U   /* MIO 54 – sortie, CS actif bas  */
#define BNO085_PIN_INT           55U   /* MIO 55 – entrée, INT actif bas */
#define BNO085_PIN_RST           56U   /* MIO 56 – sortie, RST actif bas */
#define BNO085_PIN_WAKE          57U   /* MIO 57 – sortie, PS0/WAKE      */

/* ─── Constantes SHTP ───────────────────────────────────────────────────── */

#define SHTP_HEADER_SIZE         4U
#define SHTP_MAX_CARGO_SIZE      128U
#define SHTP_MAX_PACKET_SIZE     (SHTP_HEADER_SIZE + SHTP_MAX_CARGO_SIZE)

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

#define BNO085_OK                0
#define BNO085_ERR_SPI          -1
#define BNO085_ERR_GPIO         -2
#define BNO085_ERR_TIMEOUT      -3
#define BNO085_ERR_CHECKSUM     -4
#define BNO085_ERR_NO_DATA      -5

/* ─── Timeout ───────────────────────────────────────────────────────────── */

#define BNO085_INT_TIMEOUT_US    5000U  /* 5 ms pour attendre INT */
#define BNO085_RESET_DELAY_MS    50U    /* Délai après reset */

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
    BNO085_Vec3      accel;         /**< Accélération brute [m/s²]           */
    BNO085_Vec3      linear_accel;  /**< Accélération sans gravité [m/s²]    */
    BNO085_Vec3      gyro;          /**< Vitesse angulaire calibrée [rad/s]  */
    BNO085_Vec3      mag;           /**< Champ magnétique [µT]               */
    BNO085_Quaternion rotation;     /**< Quaternion AHRS (référence Nord)    */
    BNO085_Quaternion game_rv;      /**< Quaternion jeu (pas de magnéto)     */
    float            yaw;           /**< Cap [°] 0-360                       */
    float            pitch;         /**< Tangage [°]                         */
    float            roll;          /**< Roulis [°]                          */
    u8               status;        /**< Statut de calibration (0-3)         */
    u8               new_data;      /**< Flag : nouvelles données disponibles */
} BNO085_Data;

/** Handle principal du driver */
typedef struct {
    XSpiPs      spi;
    XGpioPs     gpio;
    u8          tx_buf[SHTP_MAX_PACKET_SIZE];
    u8          rx_buf[SHTP_MAX_PACKET_SIZE];
    u8          seq[8];         /**< Numéro de séquence par canal */
    BNO085_Data data;
} BNO085_Dev;

/* ─── API publique ──────────────────────────────────────────────────────── */

/**
 * @brief Initialise le SPI, les GPIO et remet le BNO085 en état de marche.
 * @param dev  Pointeur vers le handle alloué par l'appelant.
 * @return BNO085_OK ou code d'erreur.
 */
int BNO085_Init(BNO085_Dev *dev);

/**
 * @brief Effectue un reset matériel du BNO085 et attend qu'il soit prêt.
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
 *        À appeler régulièrement dans votre boucle principale ou ISR.
 * @return BNO085_OK si au moins un paquet traité, BNO085_ERR_NO_DATA sinon.
 */
int BNO085_Poll(BNO085_Dev *dev);

/**
 * @brief Retourne un pointeur vers les dernières données lues.
 */
static inline BNO085_Data *BNO085_GetData(BNO085_Dev *dev) {
    return &dev->data;
}

/**
 * @brief Retourne 1 si de nouvelles données sont disponibles, 0 sinon.
 *        Remet le flag à 0 après lecture.
 */
static inline int BNO085_DataReady(BNO085_Dev *dev) {
    int ready = dev->data.new_data;
    dev->data.new_data = 0;
    return ready;
}

#endif /* BNO085_H */