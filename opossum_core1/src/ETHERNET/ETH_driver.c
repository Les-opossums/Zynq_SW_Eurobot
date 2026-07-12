#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "eth_driver.h"

#include "lwip/init.h"
#include "lwip/udp.h"
#include "netif/xadapter.h"
#include "netif/etharp.h"   /* si non trouve, essaie "lwip/etharp.h" selon ta version de lwip211 */

#include "xparameters.h"
#include "xtime_l.h"

#ifndef ETH_DEBUG_MAX_LEN
#define ETH_DEBUG_MAX_LEN 200
#endif

static struct netif    g_netif;
static struct udp_pcb *g_dbg_pcb;
static struct udp_pcb *g_tlm_pcb;
static struct udp_pcb *g_cmd_pcb;

static ip_addr_t g_peer_ip;

static uint16_t g_seq_debug;
static uint16_t g_seq_telemetry;

/* NO_SYS_NO_TIMERS=1 (reglage par defaut du BSP lwip211 Xilinx en mode
 * bare-metal) desactive sys_check_timeouts() : c'est a l'appli d'appeler
 * elle-meme chaque timer protocolaire necessaire. On n'utilise que de
 * l'UDP (pas de TCP/DHCP), donc seul le vieillissement du cache ARP
 * nous concerne. */
#define ETH_ARP_TMR_INTERVAL_MS 5000
static uint32_t g_last_arp_tmr_ms;

static eth_cmd_handler_t  g_cmd_handler;
static eth_driver_stats_t g_stats;

/* ------------------------------------------------------------------------
 * CRC16-CCITT (poly 0x1021, init 0xFFFF) -- meme famille que ta sortie
 * calibration UART, pas de table pour rester simple, cout negligeable
 * pour des trames de quelques dizaines/centaines d'octets.
 * ------------------------------------------------------------------------ */
static uint16_t crc16_ccitt(uint16_t crc, const uint8_t *data, uint32_t len)
{
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (int i = 0; i < 8; i++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ------------------------------------------------------------------------
 * Horodatage micro-seconde, base sur le global timer du Zynq (xtime_l.h).
 * Domaine d'horloge propre au Zynq, independant de la calibration
 * JeVoisClock cote vision (ne pas confondre les deux).
 * ------------------------------------------------------------------------ */
static uint32_t eth_get_timestamp_us(void)
{
    XTime t;
    XTime_GetTime(&t);
    return (uint32_t)((t * 1000000ULL) / COUNTS_PER_SECOND);
}

/* ------------------------------------------------------------------------
 * Reception commandes (port ETH_CMD_PORT)
 * ------------------------------------------------------------------------ */
static void eth_cmd_rx_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                           const ip_addr_t *addr, u16_t port)
{
    (void)arg; (void)pcb; (void)addr; (void)port;
    if (p == NULL) {
        return;
    }

    uint8_t rx_buf[sizeof(eth_frame_header_t) + ETH_MAX_PAYLOAD];

    if (p->tot_len < sizeof(eth_frame_header_t) || p->tot_len > sizeof(rx_buf)) {
        g_stats.rx_malformed++;
        pbuf_free(p);
        return;
    }

    /* Regroupe la chaine de pbuf en un buffer lineaire (utile si un jour
     * un payload s'etale sur plusieurs pbuf). */
    uint16_t total_len = (uint16_t)p->tot_len;
    pbuf_copy_partial(p, rx_buf, total_len, 0);
    pbuf_free(p);

    eth_frame_header_t hdr;
    memcpy(&hdr, rx_buf, sizeof(hdr));

    if (hdr.magic != ETH_FRAME_MAGIC ||
        (uint32_t)(sizeof(hdr) + hdr.payload_len) != total_len) {
        g_stats.rx_malformed++;
        return;
    }

    uint16_t rx_crc = hdr.crc16;
    ((eth_frame_header_t *)rx_buf)->crc16 = 0;
    uint16_t calc_crc = crc16_ccitt(0xFFFF, rx_buf, total_len);

    if (calc_crc != rx_crc) {
        g_stats.rx_crc_error++;
        return;
    }

    g_stats.rx_ok++;

    if (g_cmd_handler != NULL) {
        g_cmd_handler((eth_msg_type_t)hdr.msg_type, rx_buf + sizeof(hdr), hdr.payload_len);
    }
}

/* ------------------------------------------------------------------------
 * Init / poll
 * ------------------------------------------------------------------------ */
int eth_driver_init(const eth_driver_config_t *cfg)
{
    ip_addr_t ipaddr, netmask, gw;

    memset(&g_stats, 0, sizeof(g_stats));
    g_seq_debug       = 0;
    g_seq_telemetry   = 0;
    g_cmd_handler     = NULL;
    g_last_arp_tmr_ms = 0;

    lwip_init();

    IP4_ADDR(&ipaddr,   (cfg->local_ip   >> 24) & 0xFF, (cfg->local_ip   >> 16) & 0xFF,
                         (cfg->local_ip   >>  8) & 0xFF,  cfg->local_ip         & 0xFF);
    IP4_ADDR(&netmask,  (cfg->netmask    >> 24) & 0xFF, (cfg->netmask    >> 16) & 0xFF,
                         (cfg->netmask    >>  8) & 0xFF,  cfg->netmask          & 0xFF);
    IP4_ADDR(&gw,        (cfg->gateway_ip>> 24) & 0xFF, (cfg->gateway_ip>> 16) & 0xFF,
                         (cfg->gateway_ip>>  8) & 0xFF,  cfg->gateway_ip       & 0xFF);
    IP4_ADDR(&g_peer_ip,(cfg->peer_ip    >> 24) & 0xFF, (cfg->peer_ip    >> 16) & 0xFF,
                         (cfg->peer_ip    >>  8) & 0xFF,  cfg->peer_ip          & 0xFF);

    memset(&g_netif, 0, sizeof(g_netif));

    /* XPAR_XEMACPS_0_BASEADDR : adapte si tu es sur GEM1 ou un nom different
     * selon ton design Vivado -- verifie dans xparameters.h genere par ton
     * projet. */
    /* xemac_add() retourne un struct netif* (NULL = echec), pas un code
     * d'erreur entier -- attention a ne pas comparer a != 0. */
    if (xemac_add(&g_netif, &ipaddr, &netmask, &gw,
                  (unsigned char *)cfg->mac_addr,
                  XPAR_XEMACPS_0_BASEADDR) == NULL) {
        return -1;
    }

    netif_set_default(&g_netif);
    netif_set_up(&g_netif);

    /* L'activation de l'interruption EMAC est en general geree par
     * xemac_add() en mode interrupt (cf lwip211 xlwip_config.h,
     * XLWIP_CONFIG_INCLUDE_XEMACPS_INTERRUPT_MODE). Si chez toi ce n'est
     * pas automatique, il faudra un XScuGic_Connect() explicite sur
     * XPAR_XEMACPS_0_INTR vers ton GIC deja initialise, comme pour tes
     * autres peripheriques -- a verifier sur ta config BSP reelle. */

    g_dbg_pcb = udp_new();
    g_tlm_pcb = udp_new();
    g_cmd_pcb = udp_new();
    if (!g_dbg_pcb || !g_tlm_pcb || !g_cmd_pcb) {
        return -2;
    }

    udp_bind(g_dbg_pcb, IP_ADDR_ANY, 0);
    udp_bind(g_tlm_pcb, IP_ADDR_ANY, 0);
    udp_bind(g_cmd_pcb, IP_ADDR_ANY, ETH_CMD_PORT);
    udp_recv(g_cmd_pcb, eth_cmd_rx_cb, NULL);

    return 0;
}

void eth_driver_poll(void)
{
    xemacif_input(&g_netif);

    /* etharp_tmr() doit etre appele environ toutes les ARP_TMR_INTERVAL
     * (5s cote lwIP) pour vieillir/purger le cache ARP. Comparaison en
     * uint32_t : robuste au wraparound du compteur us/1000 (~71min). */
    uint32_t now_ms = eth_get_timestamp_us() / 1000U;
    if ((uint32_t)(now_ms - g_last_arp_tmr_ms) >= ETH_ARP_TMR_INTERVAL_MS) {
        etharp_tmr();
        g_last_arp_tmr_ms = now_ms;
    }
}

void eth_driver_set_cmd_handler(eth_cmd_handler_t handler)
{
    g_cmd_handler = handler;
}

/* ------------------------------------------------------------------------
 * Envoi
 * ------------------------------------------------------------------------ */
static int eth_send_frame_internal(struct udp_pcb *pcb, uint16_t dst_port,
                                    eth_msg_type_t type, const void *payload,
                                    uint16_t payload_len, uint16_t *seq_counter)
{
    /* buffer statique -> non reentrant : ne jamais appeler depuis une ISR,
     * uniquement depuis la boucle principale (execution sequentielle). */
    static uint8_t tx_buf[sizeof(eth_frame_header_t) + ETH_MAX_PAYLOAD];

    if (payload_len > ETH_MAX_PAYLOAD) {
        g_stats.tx_dropped_too_big++;
        return -1;
    }

    eth_frame_header_t *hdr = (eth_frame_header_t *)tx_buf;
    hdr->magic        = ETH_FRAME_MAGIC;
    hdr->version      = ETH_PROTOCOL_VERSION;
    hdr->msg_type     = (uint8_t)type;
    hdr->seq          = (*seq_counter)++;
    hdr->timestamp_us = eth_get_timestamp_us();
    hdr->payload_len  = payload_len;
    hdr->crc16        = 0;

    if (payload_len) {
        memcpy(tx_buf + sizeof(eth_frame_header_t), payload, payload_len);
    }

    uint16_t total_len = (uint16_t)(sizeof(eth_frame_header_t) + payload_len);
    hdr->crc16 = crc16_ccitt(0xFFFF, tx_buf, total_len);

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, total_len, PBUF_RAM);
    if (p == NULL) {
        g_stats.tx_dropped_no_pbuf++;
        return -2;
    }

    pbuf_take(p, tx_buf, total_len);
    err_t err = udp_sendto(pcb, p, &g_peer_ip, dst_port);
    pbuf_free(p);

    if (err != ERR_OK) {
        g_stats.tx_dropped_send_err++;
        return -3;
    }

    g_stats.tx_ok++;
    return 0;
}

int eth_printf(const char *fmt, ...)
{
    char text_buf[ETH_DEBUG_MAX_LEN];
    va_list args;

    va_start(args, fmt);
    int len = vsnprintf(text_buf, sizeof(text_buf), fmt, args);
    va_end(args);

    if (len < 0) {
        return -1;
    }
    if (len >= (int)sizeof(text_buf)) {
        len = sizeof(text_buf) - 1; /* tronque plutot que planter */
    }

    return eth_send_frame_internal(g_dbg_pcb, ETH_DEBUG_PORT, ETH_MSG_DEBUG_TEXT,
                                    text_buf, (uint16_t)len, &g_seq_debug);
}

int eth_send_frame(eth_msg_type_t type, const void *payload, uint16_t payload_len)
{
    return eth_send_frame_internal(g_tlm_pcb, ETH_TELEMETRY_PORT, type,
                                    payload, payload_len, &g_seq_telemetry);
}

int eth_send_odom(const eth_payload_odom_t *odom)
{
    return eth_send_frame(ETH_MSG_ODOM, odom, sizeof(*odom));
}

int eth_send_imu(const eth_payload_imu_t *imu)
{
    return eth_send_frame(ETH_MSG_IMU, imu, sizeof(*imu));
}

int eth_send_heartbeat(uint32_t uptime_ms, uint8_t state, uint8_t flags)
{
    eth_payload_heartbeat_t hb = { uptime_ms, state, flags };
    return eth_send_frame(ETH_MSG_HEARTBEAT, &hb, sizeof(hb));
}

const eth_driver_stats_t *eth_driver_get_stats(void)
{
    return &g_stats;
}