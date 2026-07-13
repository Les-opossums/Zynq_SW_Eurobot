#ifndef ETH_DRIVER_H
#define ETH_DRIVER_H

#include <stdint.h>
#include "eth_protocol.h"

typedef struct {
    uint32_t tx_ok;
    uint32_t tx_dropped_no_pbuf;
    uint32_t tx_dropped_send_err;
    uint32_t tx_dropped_too_big;
    uint32_t rx_ok;
    uint32_t rx_crc_error;
    uint32_t rx_malformed;
} eth_driver_stats_t;

/* Callback appele quand une commande valide arrive du Raspberry (port ETH_CMD_PORT).
 * Attention: appele en contexte "boucle principale" (depuis eth_driver_poll()),
 * jamais directement en ISR -- mais reste court : poste un flag/une commande
 * vers ta boucle rapide si besoin, meme logique que ton pattern SGI existant. */
typedef void (*eth_cmd_handler_t)(eth_msg_type_t type, const uint8_t *payload, uint16_t len);

/* Config reseau statique (lien direct ou switch dedie robot). Adresses en
 * uint32_t "lisibles", ex: (192u<<24)|(168u<<16)|(1u<<8)|10u pour 192.168.1.10 */
typedef struct {
    uint8_t  mac_addr[6];
    uint32_t local_ip;
    uint32_t netmask;
    uint32_t gateway_ip;
    uint32_t peer_ip;      /* IP du Raspberry Pi (destination debug/telemetrie) */
} eth_driver_config_t;

/* A appeler une fois au demarrage, apres l'init de ton GIC existant. */
int  eth_driver_init(const eth_driver_config_t *cfg);

/* A appeler regulierement (ta section 10ms est un bon candidat, pas la
 * boucle 1ms : le temps de traitement lwIP n'est pas deterministe).
 * Fait avancer la pile lwIP : reception + timers ARP/TCP internes. */
void eth_driver_poll(void);

/* Enregistre le handler de commandes recues du Raspberry. */
void eth_driver_set_cmd_handler(eth_cmd_handler_t handler);

/* Debug "printf-like", non bloquant, best-effort (UDP). Silencieusement
 * drop si le Pi n'ecoute pas ou si le pool de pbuf est plein -- ne doit
 * jamais faire attendre l'appelant. A n'appeler QUE depuis la boucle
 * principale, jamais depuis une ISR (buffer interne non reentrant). */
int eth_printf(const char *fmt, ...);

/* Envoi generique d'une trame structuree (telemetrie ou autre). */
int eth_send_frame(eth_msg_type_t type, const void *payload, uint16_t payload_len);

/* Statistiques (pbuf exhaustion, erreurs CRC, etc. -- utile en debug terrain) */
const eth_driver_stats_t *eth_driver_get_stats(void);

#endif /* ETH_DRIVER_H */