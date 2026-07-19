#ifndef ETH_DRIVER_H
#define ETH_DRIVER_H

#include <stdint.h>
#include "../../ETH_protocol.h"

/**
 * @brief Structure contenant les statistiques du driver Ethernet.
 * 
 */
typedef struct {
    uint32_t tx_ok;
    uint32_t tx_dropped_no_pbuf;
    uint32_t tx_dropped_send_err;
    uint32_t tx_dropped_too_big;
    uint32_t rx_ok;
    uint32_t rx_crc_error;
    uint32_t rx_malformed;
} eth_driver_stats_t;

 /**
  * @brief Callback appele quand une commande valide arrive du Raspberry (port ETH_CMD_PORT).
  * @param type Type de la commande reçue.
  * @param payload Pointeur vers le payload de la commande.
  * @param len Longueur du payload.
  * @note Attention: appele en contexte "boucle principale" (depuis eth_driver_poll()),
  * jamais directement en ISR -- mais reste court : poste un flag/une commande
  * vers ta boucle rapide si besoin, meme logique que ton pattern SGI existant.
  */
typedef void (*eth_cmd_handler_t)(eth_msg_type_t type, const uint8_t *payload, uint16_t len);


 /**
  * @brief Structure contenant la configuration du driver Ethernet.
  * @note Adresses en uint32_t "lisibles", ex: (192u<<24)|(168u<<16)|(1u<<8)|10u pour 192.168.1.10
  * 
  */
typedef struct {
    uint8_t  mac_addr[6];
    uint32_t local_ip;
    uint32_t netmask;
    uint32_t gateway_ip;
    uint32_t peer_ip;      /* IP du Raspberry Pi (destination debug/telemetrie) */
} eth_driver_config_t;


/**
 * @brief Initialise le driver Ethernet.
 * 
 * @param cfg Pointeur vers la configuration du driver.
 * @return int Code de retour (0 en cas de succes).
 */
int  eth_driver_init(const eth_driver_config_t *cfg);

/**
 * @brief Poll le driver Ethernet pour traiter les trames entrantes et les timers internes.
 * 
 */
void eth_driver_poll(void);

/**
 * @brief Set the command handler for incoming Ethernet commands.
 * 
 * @param handler Callback function to handle incoming commands.
 */
void eth_driver_set_cmd_handler(eth_cmd_handler_t handler);

/**
 * @brief Debug "printf-like", non bloquant, best-effort (UDP).
 * 
 * @param fmt Format string.
 * @return int Code de retour.
 * 
 * @note Silencieusement drop si le Pi n'ecoute pas ou si le pool de pbuf est plein
 * ne doit jamais faire attendre l'appelant. 
 * A n'appeler QUE depuis la boucle principale, jamais depuis une ISR (buffer interne non reentrant).
 */
int eth_printf(const char *fmt, ...);

/**
 * @brief Envoie une trame Ethernet avec un type de message et un payload.
 * 
 * @param type Type de message à envoyer (ETH_MSG_*).
 * @param payload Pointeur vers le payload à envoyer.
 * @param payload_len Longueur du payload à envoyer.
 * @return int 
 */
int eth_send_frame(eth_msg_type_t type, const void *payload, uint16_t payload_len);

/**
 * @brief Récupère les statistiques du driver Ethernet.
 * 
 * @return const eth_driver_stats_t* Pointeur vers les statistiques.
 */
const eth_driver_stats_t *eth_driver_get_stats(void);

struct netif *eth_driver_get_netif(void);

#endif /* ETH_DRIVER_H */