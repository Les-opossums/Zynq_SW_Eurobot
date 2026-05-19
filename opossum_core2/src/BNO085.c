/**
 * @file bno085.c
 * @brief Implémentation du driver BNO085 (SHTP/SH-2 sur SPI) pour Zynq7000
 *
 * Architecture du protocole :
 *
 *   ┌─────────────────────────────┐
 *   │      API utilisateur        │  BNO085_Poll / EnableReport / GetData
 *   ├─────────────────────────────┤
 *   │     Couche SH-2             │  Parsing des Feature Reports
 *   ├─────────────────────────────┤
 *   │     Couche SHTP             │  Header 4 octets + cargo
 *   ├─────────────────────────────┤
 *   │     Couche SPI (XSpiPs)     │  PS SPI Zynq, Mode 3, CS GPIO manuel
 *   └─────────────────────────────┘
 *
 * Flux SPI (lecture d'un paquet) :
 *   1. INT passe bas  → le BNO085 a des données
 *   2. CS = 0         → début de transaction
 *   3. RX 4 octets    → header SHTP  [len_L, len_H, canal, seq]
 *   4. len = (len_H<<8 | len_L) & 0x7FFF  (MSB = continuation flag)
 *   5. RX (len-4) octets → cargo / payload
 *   6. CS = 1         → fin de transaction
 */

#include "bno085.h"
#include "xil_printf.h"
#include <math.h>
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * Macros internes
 * ═══════════════════════════════════════════════════════════════════════════ */

#define CS_LOW(dev)   XGpioPs_WritePin(&(dev)->gpio, BNO085_PIN_CS,   0)
#define CS_HIGH(dev)  XGpioPs_WritePin(&(dev)->gpio, BNO085_PIN_CS,   1)
#define RST_LOW(dev)  XGpioPs_WritePin(&(dev)->gpio, BNO085_PIN_RST,  0)
#define RST_HIGH(dev) XGpioPs_WritePin(&(dev)->gpio, BNO085_PIN_RST,  1)
#define WAKE_LOW(dev) XGpioPs_WritePin(&(dev)->gpio, BNO085_PIN_WAKE, 0)
#define INT_READ(dev) XGpioPs_ReadPin (&(dev)->gpio, BNO085_PIN_INT)

/** Q-point vers float : valeur * 2^(-qpoint) */
static inline float q_to_float(s16 val, u8 qpoint) {
    return (float)val * powf(2.0f, -(float)qpoint);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Couche SPI bas-niveau
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialise le contrôleur PS SPI en Mode 3, master, 1 MHz.
 */
static int spi_init(BNO085_Dev *dev)
{
    XSpiPs_Config *cfg;
    int ret;

    cfg = XSpiPs_LookupConfig(BNO085_SPI_DEVICE_ID);
    if (!cfg) {
        xil_printf("[BNO085] Erreur: config SPI introuvable\r\n");
        return BNO085_ERR_SPI;
    }

    ret = XSpiPs_CfgInitialize(&dev->spi, cfg, cfg->BaseAddress);
    if (ret != XST_SUCCESS) {
        xil_printf("[BNO085] Erreur: init SPI (%d)\r\n", ret);
        return BNO085_ERR_SPI;
    }

    /* Auto-test interne */
    ret = XSpiPs_SelfTest(&dev->spi);
    if (ret != XST_SUCCESS) {
        xil_printf("[BNO085] Erreur: self-test SPI (%d)\r\n", ret);
        return BNO085_ERR_SPI;
    }

    /*
     * Mode 3 (CPOL=1, CPHA=1) – horloge au repos haute,
     * données capturées sur le front descendant.
     * CS géré manuellement via GPIO → on désactive le CS hardware.
     */
    XSpiPs_SetOptions(&dev->spi,
                      XSPIPS_MASTER_OPTION          |
                      XSPIPS_MANUAL_START_OPTION    |
                      XSPIPS_CLK_PHASE_1_OPTION     |  /* CPHA = 1 */
                      XSPIPS_CLK_ACTIVE_LOW_OPTION);   /* CPOL = 1 */

    XSpiPs_SetClkPrescaler(&dev->spi, XSPIPS_CLK_PRESCALE_32);
    /*
     * NOTE : La fréquence réelle dépend de l'horloge du PS.
     * Sur Zynq7000 à 100 MHz : 100 MHz / 32 = ~3.1 MHz  → ok pour BNO085
     * Sur Zynq7000 à 50  MHz :  50 MHz / 32 = ~1.6 MHz  → ok
     * Ajustez PRESCALE si votre PS tourne à une autre fréquence.
     */

    return BNO085_OK;
}

/**
 * @brief Transfert SPI full-duplex (envoi + réception simultanés).
 *
 * @param tx   Buffer d'envoi  (NULL → envoie des 0x00)
 * @param rx   Buffer de réception (NULL → données ignorées)
 * @param len  Nombre d'octets
 */
static int spi_transfer(BNO085_Dev *dev, const u8 *tx, u8 *rx, u16 len)
{
    /* Tampons locaux si l'appelant ne fournit pas de buffer */
    static u8 dummy_tx[SHTP_MAX_PACKET_SIZE];
    static u8 dummy_rx[SHTP_MAX_PACKET_SIZE];

    if (!tx) { memset(dummy_tx, 0x00, len); tx = dummy_tx; }
    if (!rx) { rx = dummy_rx; }

    if (len > SHTP_MAX_PACKET_SIZE) {
        xil_printf("[BNO085] Erreur: transfert trop grand (%d)\r\n", len);
        return BNO085_ERR_SPI;
    }

    /*
     * XSpiPs_PolledTransfer gère FIFO, start et attend la fin.
     * Le CS hardware est désactivé → on gère CS avant/après cet appel.
     */
    int ret = XSpiPs_PolledTransfer(&dev->spi, (u8 *)tx, rx, len);
    if (ret != XST_SUCCESS) {
        xil_printf("[BNO085] Erreur: transfert SPI (%d)\r\n", ret);
        return BNO085_ERR_SPI;
    }

    return BNO085_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * GPIO
 * ═══════════════════════════════════════════════════════════════════════════ */

static int gpio_init(BNO085_Dev *dev)
{
    XGpioPs_Config *cfg;
    int ret;

    cfg = XGpioPs_LookupConfig(BNO085_GPIO_DEVICE_ID);
    if (!cfg) return BNO085_ERR_GPIO;

    ret = XGpioPs_CfgInitialize(&dev->gpio, cfg, cfg->BaseAddr);
    if (ret != XST_SUCCESS) return BNO085_ERR_GPIO;

    /* Sorties */
    XGpioPs_SetDirectionPin(&dev->gpio, BNO085_PIN_CS,   1);
    XGpioPs_SetDirectionPin(&dev->gpio, BNO085_PIN_RST,  1);
    XGpioPs_SetDirectionPin(&dev->gpio, BNO085_PIN_WAKE, 1);
    XGpioPs_SetOutputEnablePin(&dev->gpio, BNO085_PIN_CS,   1);
    XGpioPs_SetOutputEnablePin(&dev->gpio, BNO085_PIN_RST,  1);
    XGpioPs_SetOutputEnablePin(&dev->gpio, BNO085_PIN_WAKE, 1);

    /* Entrée */
    XGpioPs_SetDirectionPin(&dev->gpio, BNO085_PIN_INT, 0);

    /* États initiaux sûrs */
    CS_HIGH(dev);
    RST_HIGH(dev);
    WAKE_LOW(dev);   /* PS0 = 0 → mode SPI */

    return BNO085_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Couche SHTP
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Attend que INT passe bas (BNO085 prêt à communiquer).
 * @return BNO085_OK ou BNO085_ERR_TIMEOUT
 */
static int shtp_wait_int(BNO085_Dev *dev)
{
    u32 timeout = BNO085_INT_TIMEOUT_US;
    while (INT_READ(dev) != 0) {
        usleep(1);
        if (--timeout == 0) {
            return BNO085_ERR_TIMEOUT;
        }
    }
    return BNO085_OK;
}

/**
 * @brief Envoie un paquet SHTP sur le bus SPI.
 *
 * @param channel   Canal SHTP cible
 * @param payload   Données du cargo
 * @param payload_len Longueur du cargo
 */
static int shtp_send(BNO085_Dev *dev, u8 channel,
                     const u8 *payload, u16 payload_len)
{
    u16 total = SHTP_HEADER_SIZE + payload_len;

    if (total > SHTP_MAX_PACKET_SIZE) return BNO085_ERR_SPI;

    /* Construction du header SHTP */
    dev->tx_buf[0] = (u8)(total & 0xFF);          /* longueur octet bas  */
    dev->tx_buf[1] = (u8)((total >> 8) & 0x7F);  /* longueur octet haut */
    dev->tx_buf[2] = channel;
    dev->tx_buf[3] = dev->seq[channel]++;         /* numéro de séquence  */

    if (payload && payload_len > 0)
        memcpy(&dev->tx_buf[4], payload, payload_len);

    CS_LOW(dev);
    usleep(1); /* setup time */

    int ret = spi_transfer(dev, dev->tx_buf, dev->rx_buf, total);

    usleep(1); /* hold time */
    CS_HIGH(dev);

    return ret;
}

/**
 * @brief Lit un paquet SHTP depuis le BNO085.
 *
 * @param channel_out  Canal SHTP reçu (sortie)
 * @param length_out   Longueur du cargo reçu (sortie)
 * @return BNO085_OK, BNO085_ERR_TIMEOUT, ou BNO085_ERR_NO_DATA
 */
static int shtp_receive(BNO085_Dev *dev, u8 *channel_out, u16 *length_out)
{
    /* Si INT est haut, rien à lire */
    if (INT_READ(dev) != 0) return BNO085_ERR_NO_DATA;

    CS_LOW(dev);
    usleep(1);

    /* Lecture du header seul (4 octets) */
    memset(dev->tx_buf, 0x00, SHTP_HEADER_SIZE);
    int ret = spi_transfer(dev, dev->tx_buf, dev->rx_buf, SHTP_HEADER_SIZE);
    if (ret != BNO085_OK) {
        CS_HIGH(dev);
        return ret;
    }

    /* Décodage de la longueur (bit 15 = flag continuation, ignoré ici) */
    u16 cargo_len = ((u16)(dev->rx_buf[1] & 0x7F) << 8) | dev->rx_buf[0];

    if (cargo_len == 0 || cargo_len > SHTP_MAX_PACKET_SIZE) {
        CS_HIGH(dev);
        return BNO085_ERR_NO_DATA;
    }

    u8  channel = dev->rx_buf[2];
    /* u8  seq  = dev->rx_buf[3]; */   /* inutilisé pour l'instant */

    u16 payload_len = cargo_len - SHTP_HEADER_SIZE;

    /* Lecture du cargo restant si présent */
    if (payload_len > 0) {
        memset(dev->tx_buf, 0x00, payload_len);
        ret = spi_transfer(dev, dev->tx_buf, dev->rx_buf, payload_len);
    }

    usleep(1);
    CS_HIGH(dev);

    if (ret != BNO085_OK) return ret;

    if (channel_out)  *channel_out = channel;
    if (length_out)   *length_out  = payload_len;

    return BNO085_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Parsing SH-2 – Feature Reports
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Convertit un quaternion en angles d'Euler (yaw/pitch/roll) en degrés.
 */
static void quat_to_euler(const BNO085_Quaternion *q,
                          float *yaw, float *pitch, float *roll)
{
    float sinr_cosp = 2.0f * (q->real * q->i + q->j * q->k);
    float cosr_cosp = 1.0f - 2.0f * (q->i * q->i + q->j * q->j);
    *roll = atan2f(sinr_cosp, cosr_cosp) * (180.0f / M_PI);

    float sinp = 2.0f * (q->real * q->j - q->k * q->i);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(90.0f, sinp);
    else
        *pitch = asinf(sinp) * (180.0f / M_PI);

    float siny_cosp = 2.0f * (q->real * q->k + q->i * q->j);
    float cosy_cosp = 1.0f - 2.0f * (q->j * q->j + q->k * q->k);
    *yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / M_PI);
    if (*yaw < 0.0f) *yaw += 360.0f;
}

/**
 * @brief Analyse un payload SH-2 reçu sur le canal Reports ou Wake Reports.
 */
static void sh2_parse_report(BNO085_Dev *dev, const u8 *buf, u16 len)
{
    if (len < 5) return;

    u8 report_id = buf[0];
    /* buf[1] = sequence number du rapport */
    u8 status    = buf[2] & 0x03;   /* bits 0-1 : statut calibration */

    dev->data.status = status;

    switch (report_id) {

    /* ── Rotation Vector (AHRS) – Q-points : i,j,k=Q14 ; real=Q14 ; acc=Q12 */
    case SH2_ROTATION_VECTOR:
        if (len < 14) break;
        dev->data.rotation.i        = q_to_float((s16)((buf[5]  << 8) | buf[4]),  14);
        dev->data.rotation.j        = q_to_float((s16)((buf[7]  << 8) | buf[6]),  14);
        dev->data.rotation.k        = q_to_float((s16)((buf[9]  << 8) | buf[8]),  14);
        dev->data.rotation.real     = q_to_float((s16)((buf[11] << 8) | buf[10]), 14);
        dev->data.rotation.accuracy = q_to_float((s16)((buf[13] << 8) | buf[12]), 12);
        quat_to_euler(&dev->data.rotation,
                      &dev->data.yaw, &dev->data.pitch, &dev->data.roll);
        dev->data.new_data = 1;
        break;

    /* ── Game Rotation Vector (pas de magnéto) – Q14 */
    case SH2_GAME_ROTATION_VECTOR:
        if (len < 12) break;
        dev->data.game_rv.i    = q_to_float((s16)((buf[5]  << 8) | buf[4]),  14);
        dev->data.game_rv.j    = q_to_float((s16)((buf[7]  << 8) | buf[6]),  14);
        dev->data.game_rv.k    = q_to_float((s16)((buf[9]  << 8) | buf[8]),  14);
        dev->data.game_rv.real = q_to_float((s16)((buf[11] << 8) | buf[10]), 14);
        dev->data.new_data = 1;
        break;

    /* ── Accéléromètre – Q8 */
    case SH2_ACCELEROMETER:
        if (len < 10) break;
        dev->data.accel.x = q_to_float((s16)((buf[5] << 8) | buf[4]), 8);
        dev->data.accel.y = q_to_float((s16)((buf[7] << 8) | buf[6]), 8);
        dev->data.accel.z = q_to_float((s16)((buf[9] << 8) | buf[8]), 8);
        dev->data.new_data = 1;
        break;

    /* ── Accélération linéaire (sans gravité) – Q8 */
    case SH2_LINEAR_ACCELERATION:
        if (len < 10) break;
        dev->data.linear_accel.x = q_to_float((s16)((buf[5] << 8) | buf[4]), 8);
        dev->data.linear_accel.y = q_to_float((s16)((buf[7] << 8) | buf[6]), 8);
        dev->data.linear_accel.z = q_to_float((s16)((buf[9] << 8) | buf[8]), 8);
        dev->data.new_data = 1;
        break;

    /* ── Gyroscope calibré – Q9 */
    case SH2_GYROSCOPE_CALIBRATED:
        if (len < 10) break;
        dev->data.gyro.x = q_to_float((s16)((buf[5] << 8) | buf[4]), 9);
        dev->data.gyro.y = q_to_float((s16)((buf[7] << 8) | buf[6]), 9);
        dev->data.gyro.z = q_to_float((s16)((buf[9] << 8) | buf[8]), 9);
        dev->data.new_data = 1;
        break;

    /* ── Champ magnétique – Q4 */
    case SH2_MAGNETIC_FIELD:
        if (len < 10) break;
        dev->data.mag.x = q_to_float((s16)((buf[5] << 8) | buf[4]), 4);
        dev->data.mag.y = q_to_float((s16)((buf[7] << 8) | buf[6]), 4);
        dev->data.mag.z = q_to_float((s16)((buf[9] << 8) | buf[8]), 4);
        dev->data.new_data = 1;
        break;

    default:
        /* Rapport non géré – silencieux */
        break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * API publique
 * ═══════════════════════════════════════════════════════════════════════════ */

int BNO085_Reset(BNO085_Dev *dev)
{
    xil_printf("[BNO085] Reset matériel...\r\n");

    CS_HIGH(dev);
    WAKE_LOW(dev);  /* PS0 = 0 → SPI mode */

    RST_LOW(dev);
    usleep(10000);  /* maintien 10 ms */
    RST_HIGH(dev);

    /* Attente démarrage (~50 ms) */
    usleep(BNO085_RESET_DELAY_MS * 1000U);

    /*
     * Le BNO085 envoie un paquet "Product ID Response" au démarrage
     * sur le canal Executable. On le consomme.
     */
    u8  ch;
    u16 len;
    int ret = BNO085_ERR_TIMEOUT;
    u32 tries = 20;

    while (tries--) {
        usleep(5000);
        if (shtp_receive(dev, &ch, &len) == BNO085_OK) {
            xil_printf("[BNO085] Reset OK – canal=%d, len=%d\r\n", ch, len);
            ret = BNO085_OK;
            break;
        }
    }

    if (ret != BNO085_OK)
        xil_printf("[BNO085] Attention: pas de réponse après reset\r\n");

    return ret;
}

int BNO085_Init(BNO085_Dev *dev)
{
    memset(dev, 0, sizeof(BNO085_Dev));

    xil_printf("[BNO085] Initialisation...\r\n");

    int ret = gpio_init(dev);
    if (ret != BNO085_OK) {
        xil_printf("[BNO085] Erreur GPIO\r\n");
        return ret;
    }

    ret = spi_init(dev);
    if (ret != BNO085_OK) {
        xil_printf("[BNO085] Erreur SPI\r\n");
        return ret;
    }

    ret = BNO085_Reset(dev);
    if (ret != BNO085_OK) return ret;

    xil_printf("[BNO085] Prêt\r\n");
    return BNO085_OK;
}

int BNO085_EnableReport(BNO085_Dev *dev, u8 report_id, u32 interval_us)
{
    /*
     * Commande Set Feature (SH-2 §6.5.4)
     * Format : [0xFD, report_id, 0x00, 0x00,
     *           period_LSB, period_2, period_3, period_MSB,
     *           batch_0, batch_1, batch_2, batch_3,
     *           sensor_spec_0..3]
     * Longueur totale cargo : 17 octets
     */
    u8 cmd[17] = {0};
    cmd[0]  = SH2_CMD_SET_FEATURE;
    cmd[1]  = report_id;
    /* cmd[2] et cmd[3] : feature flags = 0 */
    cmd[4]  = (u8)( interval_us        & 0xFF);
    cmd[5]  = (u8)((interval_us >>  8) & 0xFF);
    cmd[6]  = (u8)((interval_us >> 16) & 0xFF);
    cmd[7]  = (u8)((interval_us >> 24) & 0xFF);
    /* batch interval et sensor-specific = 0 */

    int ret = shtp_send(dev, SHTP_CHAN_CONTROL, cmd, sizeof(cmd));
    if (ret != BNO085_OK) {
        xil_printf("[BNO085] Erreur activation rapport 0x%02X\r\n", report_id);
        return ret;
    }

    xil_printf("[BNO085] Rapport 0x%02X activé @ %lu µs\r\n",
               report_id, (unsigned long)interval_us);
    return BNO085_OK;
}

int BNO085_Poll(BNO085_Dev *dev)
{
    u8  channel;
    u16 len;
    int got_data = 0;

    /*
     * On lit en boucle tant que INT est bas (plusieurs paquets possibles).
     * Limite à 10 itérations pour éviter de bloquer la boucle principale.
     */
    for (int i = 0; i < 10; i++) {
        int ret = shtp_receive(dev, &channel, &len);
        if (ret == BNO085_ERR_NO_DATA) break;
        if (ret != BNO085_OK)          break;
        if (len == 0)                  continue;

        /* On ne parse que les canaux de données */
        if (channel == SHTP_CHAN_REPORTS ||
            channel == SHTP_CHAN_WAKE_REPORTS ||
            channel == SHTP_CHAN_GYRO_RV) {
            sh2_parse_report(dev, dev->rx_buf, len);
            got_data = 1;
        }
        /* Canal Executable : messages de statut – ignorés silencieusement */
    }

    return got_data ? BNO085_OK : BNO085_ERR_NO_DATA;
}