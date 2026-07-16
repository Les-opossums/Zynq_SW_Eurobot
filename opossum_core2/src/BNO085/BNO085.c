/**
 * @file BNO085.c
 * @brief Driver BNO085 (SHTP/SH-2 sur SPI) pour Zynq7000 — GPIO via IO_manager
 *
 * Flux SPI (lecture d'un paquet SHTP) :
 * 1. INT passe bas   → BNO085 a des données prêtes
 * 2. CS = 0          → début de transaction
 * 3. Transfert 4 B   → header : [len_L, len_H, canal, seq]
 * 4. len = (len_H<<8 | len_L) & 0x7FFF  (bit 15 = flag continuation)
 * 5. Transfert restant → cargo / payload  (dans rx_buf depuis l'offset 0)
 * 6. CS = 1          → fin de transaction
 */

#include "../main.h"

int bno_cs_state;  /* CS actif bas */
int bno_rst_state; /* RST actif bas */
int bno_int_state; /* INT actif bas */
int bno_wake_state; /* WAKE actif bas */

/* ═══════════════════════════════════════════════════════════════════════════
 * Macros GPIO — Accès direct via IO_manager
 * ═══════════════════════════════════════════════════════════════════════════ */

#define CS_LOW(dev)    IO_Manager_DirectWrite((dev)->pin_cs, 0U)
#define CS_HIGH(dev)   IO_Manager_DirectWrite((dev)->pin_cs, 1U)
#define RST_LOW(dev)   IO_Manager_DirectWrite((dev)->pin_rst, 0U)
#define RST_HIGH(dev)  IO_Manager_DirectWrite((dev)->pin_rst, 1U)

/* INT est actif bas : retourne 0 quand le BNO085 a des données */
#define INT_READ(dev)  IO_Manager_DirectRead((dev)->pin_int)

/* ─── Utilitaire Q-point ─────────────────────────────────────────────────── */

/** Convertit un entier signé avec Q-point en flottant : val × 2^(−q) */
static inline float q_to_float(s16 val, u8 qpoint) {
    return (float)val * powf(2.0f, -(float)qpoint);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Couche SPI bas-niveau (inchangée — PS SPI)
 * ═══════════════════════════════════════════════════════════════════════════ */

static int spi_init(BNO085_Dev *dev)
{
    XSpiPs_Config *cfg = XSpiPs_LookupConfig(BNO085_SPI_DEVICE_ID);
    if (!cfg) {
        xil_printf("[BNO085] Erreur : config SPI introuvable\r\n");
        return BNO085_ERR_SPI;
    }

    int ret = XSpiPs_CfgInitialize(&dev->spi, cfg, cfg->BaseAddress);
    if (ret != XST_SUCCESS) {
        xil_printf("[BNO085] Erreur : CfgInitialize SPI (%d)\r\n", ret);
        return BNO085_ERR_SPI;
    }

    ret = XSpiPs_SelfTest(&dev->spi);
    if (ret != XST_SUCCESS) {
        xil_printf("[BNO085] Erreur : self-test SPI (%d)\r\n", ret);
        return BNO085_ERR_SPI;
    }

    /*
     * Mode 3 (CPOL=1, CPHA=1).
     * XSPIPS_FORCE_SSELECT_OPTION désactive le CS hardware du PS SPI
     * car on gère CS manuellement via IO_manager.
     */
    XSpiPs_SetOptions(&dev->spi,
                      XSPIPS_MASTER_OPTION        |
                      XSPIPS_MANUAL_START_OPTION  |
                      XSPIPS_CLK_PHASE_1_OPTION   |   /* CPHA = 1 */
                      XSPIPS_CLK_ACTIVE_LOW_OPTION|   /* CPOL = 1 */
                      XSPIPS_FORCE_SSELECT_OPTION);   /* CS hw désactivé */

    XSpiPs_SetClkPrescaler(&dev->spi, BNO085_SPI_PRESCALER);

    return BNO085_OK;
}

/**
 * @brief Transfert SPI full-duplex.
 * @param tx   Données à envoyer (NULL → zéros)
 * @param rx   Buffer de réception (NULL → données ignorées)
 * @param len  Nombre d'octets
 */
static int spi_transfer(BNO085_Dev *dev, const u8 *tx, u8 *rx, u16 len)
{
    /* Buffers de secours si l'appelant passe NULL */
    static u8 dummy_tx[SHTP_MAX_PACKET_SIZE];
    static u8 dummy_rx[SHTP_MAX_PACKET_SIZE];

    if (len > SHTP_MAX_PACKET_SIZE) {
        xil_printf("[BNO085] Erreur : transfert trop grand (%d)\r\n", (int)len);
        return BNO085_ERR_SPI;
    }

    if (!tx) { memset(dummy_tx, 0x00, len); tx = dummy_tx; }
    if (!rx) { rx = dummy_rx; }

    int ret = XSpiPs_PolledTransfer(&dev->spi, (u8 *)tx, rx, len);
    if (ret != XST_SUCCESS) {
        xil_printf("[BNO085] Erreur : PolledTransfer (%d)\r\n", ret);
        return BNO085_ERR_SPI;
    }

    return BNO085_OK;
}


/* ═══════════════════════════════════════════════════════════════════════════
 * Couche SHTP
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Envoie un paquet SHTP.
 */
static int shtp_send(BNO085_Dev *dev, u8 channel,
                     const u8 *payload, u16 payload_len)
{
    u16 total = (u16)(SHTP_HEADER_SIZE + payload_len);

    if (total > SHTP_MAX_PACKET_SIZE) {
        xil_printf("[BNO085] shtp_send : payload trop grand (%d)\r\n",
                   (int)payload_len);
        return BNO085_ERR_SPI;
    }

    dev->tx_buf[0] = (u8)(total & 0xFFU);
    dev->tx_buf[1] = (u8)((total >> 8) & 0x7FU);  /* bit 15 = 0 (pas continuation) */
    dev->tx_buf[2] = channel;
    dev->tx_buf[3] = dev->seq[channel]++;

    if (payload && payload_len > 0U)
        memcpy(&dev->tx_buf[4], payload, payload_len);

    CS_LOW(dev);
    usleep(2);  /* t_CSS : setup time CS avant SCLK */

    int ret = spi_transfer(dev, dev->tx_buf, dev->rx_buf, total);

    usleep(2);  /* t_CSH : hold time */
    CS_HIGH(dev);

    return ret;
}

/**
 * @brief Lit un paquet SHTP.
 *
 * Après retour BNO085_OK :
 * - *channel_out  : canal SHTP du paquet reçu
 * - *length_out   : longueur du cargo (payload, sans les 4 octets header)
 * - dev->rx_buf[] : contient le payload (offset 0)
 */
static int shtp_receive(BNO085_Dev *dev, u8 *channel_out, u16 *length_out)
{
    /* INT haut → rien à lire */
    if (INT_READ(dev) != 0U) return BNO085_ERR_NO_DATA;

    CS_LOW(dev);
    usleep(2);

    /* --- Lecture du header (4 octets) --- */
    u8 hdr_tx[SHTP_HEADER_SIZE] = {0, 0, 0, 0};
    u8 hdr_rx[SHTP_HEADER_SIZE];

    int ret = spi_transfer(dev, hdr_tx, hdr_rx, SHTP_HEADER_SIZE);
    if (ret != BNO085_OK) { CS_HIGH(dev); return ret; }

    /*
     * Longueur totale du paquet (header inclus).
     * Bit 15 de len_H = flag continuation → masqué par 0x7F.
     */
    u16 total_len = (u16)(((u16)(hdr_rx[1] & 0x7FU) << 8U) | hdr_rx[0]);

    if (total_len < SHTP_HEADER_SIZE || total_len > SHTP_MAX_PACKET_SIZE) {
        CS_HIGH(dev);
        return BNO085_ERR_NO_DATA;
    }

    u8  channel     = hdr_rx[2];
    u16 payload_len = total_len - (u16)SHTP_HEADER_SIZE;

    /* --- Lecture du payload --- */
    if (payload_len > 0U) {
        memset(dev->tx_buf, 0x00, payload_len);
        ret = spi_transfer(dev, dev->tx_buf, dev->rx_buf, payload_len);
        if (ret != BNO085_OK) { CS_HIGH(dev); return ret; }
    }

    usleep(2);
    CS_HIGH(dev);

    if (channel_out) *channel_out = channel;
    if (length_out)  *length_out  = payload_len;

    return BNO085_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Parsing SH-2
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Convertit un quaternion (w, x, y, z) en angles d'Euler [degrés].
 *
 * Convention : ZYX (yaw → pitch → roll).
 * Entrée : quaternion normalisé (|q| = 1).
 */

static void quat_to_euler(const BNO085_Quaternion *q,
                          float *yaw, float *pitch, float *roll)
{
    /* Roll (rotation autour de X) */
    float sinr = 2.0f * (q->real * q->i + q->j * q->k);
    float cosr = 1.0f - 2.0f * (q->i * q->i + q->j * q->j);
    *roll = atan2f(sinr, cosr) * (180.0f / (float)M_PI);

    /* Pitch (rotation autour de Y) — protection contre singularité ±90° */
    float sinp = 2.0f * (q->real * q->j - q->k * q->i);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(90.0f, sinp);
    else
        *pitch = asinf(sinp) * (180.0f / (float)M_PI);

    /* Yaw (rotation autour de Z) — ramené dans [0, 360°] */
    float siny = 2.0f * (q->real * q->k + q->i * q->j);
    float cosy = 1.0f - 2.0f * (q->j * q->j + q->k * q->k);
    *yaw = atan2f(siny, cosy) * (180.0f / (float)M_PI);
    if (*yaw < 0.0f) *yaw += 360.0f;
}

/**
 * @brief Parse un payload SH-2 (Feature Report).
 *
 * Format commun des rapports d'entrée (Input Reports) SH-2 :
 * buf[0]   : Report ID
 * buf[1]   : Sequence number
 * buf[2]   : Status / delay LSB  (bits 0-1 = calib status)
 * buf[3]   : Delay MSB
 * buf[4+]  : Données (Q-point variables selon le rapport)
 */
static void sh2_parse_report(BNO085_Dev *dev, const u8 *buf, u16 len)
{
    u16 offset = 0;

    // Le BNO085 peut concaténer plusieurs rapports dans le même payload
    while (offset < len) {
        u8 report_id = buf[offset];

        // 1. L'intrus : le Base Timestamp (toujours 5 octets, ID = 0xFB)
        if (report_id == 0xFB) {
            offset += 5;
            continue;
        }

        // Sécurité anti-débordement
        if (offset + 4 > len) break;

        u8 status = buf[offset + 2] & 0x03U;
        dev->data.calib_status = status;

        switch (report_id) {

        /* ── Rotation Vector AHRS (14 octets) */
        case SH2_ROTATION_VECTOR:
            if (offset + 14 > len) return;
            dev->data.rotation.i        = q_to_float((s16)((buf[offset + 5]  << 8) | buf[offset + 4]),  14);
            dev->data.rotation.j        = q_to_float((s16)((buf[offset + 7]  << 8) | buf[offset + 6]),  14);
            dev->data.rotation.k        = q_to_float((s16)((buf[offset + 9]  << 8) | buf[offset + 8]),  14);
            dev->data.rotation.real     = q_to_float((s16)((buf[offset + 11] << 8) | buf[offset + 10]), 14);
            dev->data.rotation.accuracy = q_to_float((s16)((buf[offset + 13] << 8) | buf[offset + 12]), 12);
            quat_to_euler(&dev->data.rotation,
                          &dev->data.yaw, &dev->data.pitch, &dev->data.roll);
            dev->data.new_data = 1U;
            offset += 14;
            break;

        /* ── Game Rotation Vector (12 octets) */
        case SH2_GAME_ROTATION_VECTOR:
            if (offset + 12 > len) return;
            dev->data.game_rv.i    = q_to_float((s16)((buf[offset + 5]  << 8) | buf[offset + 4]),  14);
            dev->data.game_rv.j    = q_to_float((s16)((buf[offset + 7]  << 8) | buf[offset + 6]),  14);
            dev->data.game_rv.k    = q_to_float((s16)((buf[offset + 9]  << 8) | buf[offset + 8]),  14);
            dev->data.game_rv.real = q_to_float((s16)((buf[offset + 11] << 8) | buf[offset + 10]), 14);
            
            quat_to_euler(&dev->data.game_rv, &dev->data.yaw, &dev->data.pitch, &dev->data.roll);
            dev->data.rotation = dev->data.game_rv;

            dev->data.new_data = 1U;
            offset += 12;
            break;

        /* ── Accélération brute (10 octets) */
        case SH2_ACCELEROMETER:
            if (offset + 10 > len) return;
            dev->data.accel.x = q_to_float((s16)((buf[offset + 5] << 8) | buf[offset + 4]), 8);
            dev->data.accel.y = q_to_float((s16)((buf[offset + 7] << 8) | buf[offset + 6]), 8);
            dev->data.accel.z = q_to_float((s16)((buf[offset + 9] << 8) | buf[offset + 8]), 8);
            dev->data.new_data = 1U;
            offset += 10;
            break;

        /* ── Accélération linéaire (gravité soustraite) (10 octets) */
        case SH2_LINEAR_ACCELERATION:
            if (offset + 10 > len) return;
            dev->data.linear_accel.x = q_to_float((s16)((buf[offset + 5] << 8) | buf[offset + 4]), 8);
            dev->data.linear_accel.y = q_to_float((s16)((buf[offset + 7] << 8) | buf[offset + 6]), 8);
            dev->data.linear_accel.z = q_to_float((s16)((buf[offset + 9] << 8) | buf[offset + 8]), 8);
            dev->data.new_data = 1U;
            offset += 10;
            break;

        /* ── Gyroscope calibré (10 octets) */
        case SH2_GYROSCOPE_CALIBRATED:
            if (offset + 10 > len) return;
            dev->data.gyro.x = q_to_float((s16)((buf[offset + 5] << 8) | buf[offset + 4]), 9);
            dev->data.gyro.y = q_to_float((s16)((buf[offset + 7] << 8) | buf[offset + 6]), 9);
            dev->data.gyro.z = q_to_float((s16)((buf[offset + 9] << 8) | buf[offset + 8]), 9);
            dev->data.new_data = 1U;
            offset += 10;
            break;

        /* ── Champ magnétique (10 octets) */
        case SH2_MAGNETIC_FIELD:
            if (offset + 10 > len) return;
            dev->data.mag.x = q_to_float((s16)((buf[offset + 5] << 8) | buf[offset + 4]), 4);
            dev->data.mag.y = q_to_float((s16)((buf[offset + 7] << 8) | buf[offset + 6]), 4);
            dev->data.mag.z = q_to_float((s16)((buf[offset + 9] << 8) | buf[offset + 8]), 4);
            dev->data.new_data = 1U;
            offset += 10;
            break;

        default:
            // Si l'ID est inconnu, on ne sait pas combien d'octets il pèse.
            // On stoppe l'analyse de ce paquet pour ne pas désaligner le buffer.
            return;
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * API publique
 * ═══════════════════════════════════════════════════════════════════════════ */

int BNO085_Reset(BNO085_Dev *dev)
{
    xil_printf("[BNO085] Reset...\r\n");

    CS_HIGH(dev);

    RST_LOW(dev);
    usleep(10000U);
    RST_HIGH(dev);

    // Attente plus longue : certains firmware BNO085 prennent jusqu'à 300ms
    usleep(300000U);

    // Diagnostic : affiche l'état de INT pendant 500ms
    xil_printf("[BNO085] Attente INT bas (500ms max)...\r\n");
    int int_went_low = 0;
    for (int ms = 0; ms < 500; ms++) {
        u32 int_state = INT_READ(dev);
        if (ms % 50 == 0) {
            // Log toutes les 50ms pour voir l'évolution
            xil_printf("[BNO085] t=%dms INT=%lu\r\n", ms, (unsigned long)int_state);
        }
        if (int_state == 0U) {
            xil_printf("[BNO085] INT bas detecte a t=%dms\r\n", ms);
            int_went_low = 1;
            break;
        }
        usleep(1000U);
    }

    if (!int_went_low) {
        xil_printf("[BNO085] ERREUR : INT ne passe jamais bas\r\n");
        xil_printf("[BNO085]   -> Verifier cablage MISO/MOSI/SCK/CS/INT\r\n");
        xil_printf("[BNO085]   -> Verifier alimentation 3.3V BNO085\r\n");
        xil_printf("[BNO085]   -> Verifier PS0=HIGH bien recu par le chip\r\n");
        return BNO085_ERR_TIMEOUT;
    }

    // Tentative de lecture du paquet de boot
    u8  ch  = 0U;
    u16 len = 0U;
    for (int tries = 0; tries < 10; tries++) {
        int ret = shtp_receive(dev, &ch, &len);
        xil_printf("[BNO085] shtp_receive try=%d ret=%d ch=%d len=%d\r\n",
                   tries, ret, (int)ch, (int)len);
        if (ret == BNO085_OK) {
            xil_printf("[BNO085] Reset OK\r\n");
            return BNO085_OK;
        }
        usleep(10000U);
    }

    return BNO085_ERR_TIMEOUT;
}

int BNO085_Init(BNO085_Dev *dev, u32 pin_cs, u32 pin_rst, u32 pin_int)
{
    memset(dev, 0, sizeof(BNO085_Dev));

    /* Sauvegarde de la configuration matérielle */
    dev->pin_cs  = pin_cs;
    dev->pin_rst = pin_rst;
    dev->pin_int = pin_int;

    xil_printf("[BNO085] Initialisation...\r\n");

    /* Assure des états initiaux sûrs (l'init GPIO est gérée par IO_manager) */
    CS_HIGH(dev);
    RST_HIGH(dev);

    /* 1. SPI PS */
    int ret = spi_init(dev);
    if (ret != BNO085_OK) {
        xil_printf("[BNO085] Erreur init SPI\r\n");
        return ret;
    }

    /* 2. Reset + boot */
    ret = BNO085_Reset(dev);
    if (ret != BNO085_OK) return ret;

    xil_printf("[BNO085] Pret\r\n");
    return BNO085_OK;
}

int BNO085_EnableReport(BNO085_Dev *dev, u8 report_id, u32 interval_us)
{
    u8 cmd[17] = {0};

    cmd[0] = 0xFD;       // SH2_CMD_SET_FEATURE
    cmd[1] = report_id;  
    cmd[2] = 0;          
    cmd[3] = 0;          
    cmd[4] = 0;          
    cmd[5] = (u8)( interval_us        & 0xFFU);
    cmd[6] = (u8)((interval_us >>  8) & 0xFFU);
    cmd[7] = (u8)((interval_us >> 16) & 0xFFU);
    cmd[8] = (u8)((interval_us >> 24) & 0xFFU);

    // Vider les paquets en attente avant d'envoyer une commande
    u8 ch_dummy; u16 len_dummy;
    for (int tries = 0; tries < 5; tries++) {
        if (shtp_receive(dev, &ch_dummy, &len_dummy) != BNO085_OK) break;
    }
    // On DOIT attendre que l'IMU abaisse INT pour pouvoir lui écrire
    u32 timeout = 0;
    while (INT_READ(dev) != 0) {
        usleep(100); // Pause de 100 µs
        timeout++;
        if (timeout > 5000) { // Timeout sécurité après 500 ms
            xil_printf("[BNO085] Timeout INT avant commande 0x%02X\r\n", report_id);
            return BNO085_ERR_SPI;
        }
    }

    // Maintenant que INT est bas, on peut envoyer la commande en toute sécurité
    int ret = shtp_send(dev, 2, cmd, sizeof(cmd));
    if (ret != BNO085_OK) {
        xil_printf("[BNO085] Erreur activation rapport 0x%02X\r\n", report_id);
        return ret;
    }

    xil_printf("[BNO085] Rapport 0x%02X active @ %lu us\r\n", report_id, interval_us);
    return BNO085_OK;
}

int BNO085_Poll(BNO085_Dev *dev)
{
    u8  channel  = 0U;
    u16 len      = 0U;
    int got_data = 0;

    /*
     * Lecture en boucle tant que INT est bas.
     * Limite à 2 paquets par appel pour ne pas monopoliser le CPU.
     */
    for (int i = 0; i < 2; i++) {
        int ret = shtp_receive(dev, &channel, &len);

        if (ret == BNO085_ERR_NO_DATA) break;
        if (ret != BNO085_OK)          break;
        if (len  == 0U)                continue;

        if (channel == SHTP_CHAN_REPORTS      ||
            channel == SHTP_CHAN_WAKE_REPORTS ||
            channel == SHTP_CHAN_GYRO_RV) {
            sh2_parse_report(dev, dev->rx_buf, len);
            got_data = 1;
        }
        /* Canal Executable (statut, product ID) → ignoré */
    }

    return got_data ? BNO085_OK : BNO085_ERR_NO_DATA;
}