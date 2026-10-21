#ifndef ETH_PROTOCOL_H
#define ETH_PROTOCOL_H

/* ------------------------------------------------------------------------
 * Protocole Ethernet Zynq <-> PC : canaux UDP, identifiants de message,
 * en-tete de trame, ETH_PROTOCOL_VERSION et ETH_PROTOCOL_HASH.
 *
 * Source unique : opossum_interfaces/protocol.yaml (sous-module), d'ou est
 * genere le fichier inclus ci-dessous. Ne rien ajouter ici : modifier
 * protocol.yaml puis lancer `python3 gen.py` dans opossum_interfaces/.
 * Si l'include echoue : `git submodule update --init opossum_interfaces`.
 * ------------------------------------------------------------------------ */
#include "../opossum_interfaces/generated/c/ETH_protocol.h"

#endif /* ETH_PROTOCOL_H */
