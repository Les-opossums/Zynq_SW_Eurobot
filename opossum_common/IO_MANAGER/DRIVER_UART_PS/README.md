# Driver UART PS

Le sous-dossier **UART PS** contient le pilote permettant la gestion du contrôleur UART du **Processing System (PS)** pour le processeur **Zynq-7000**. Il est conçu pour s'intégrer nativement à l'architecture centralisée de l'**IO_MANAGER**.

## Fonctionnalités principales

### Gestion par buffers circulaires

Le driver exploite des tampons circulaires (*ring buffers*) distincts pour isoler la réception (**RX**) et la transmission (**TX**).

La taille de ces tampons est définie statiquement à **300 octets** via la macro :

```c
UART_PS_RING_BUFFER_SIZE
```

### Réception sous interruption (ISR)

L'arrivée de données sur le port série, ou l'expiration d'un délai d'inactivité (*timeout*), déclenche un événement matériel :

- `XUARTPS_EVENT_RECV_DATA`
- `XUARTPS_EVENT_RECV_TOUT`

Le gestionnaire d'interruption `UART_PS_Handler` lit la FIFO matérielle et copie les données de manière asynchrone dans le buffer **RX**.

### Transmission non bloquante

L'envoi physique des données est géré périodiquement par la fonction :

```c
UART_PS_Update()
```

Cette méthode :

- regroupe les octets du buffer **TX** par blocs pouvant atteindre **64 octets** ;
- tente de les transmettre sans bloquer le processeur ;
- replace l'index de lecture du buffer si la FIFO matérielle ne peut pas absorber toutes les données, afin de réessayer lors du cycle suivant.

### Sécurité anti-débordement

Si le buffer de réception (**RX**) est plein, le pilote ignore volontairement les nouveaux octets entrants afin d'éviter tout dépassement de capacité (*overflow*).

### Redirection de console virtuelle

En activant l'indicateur `is_console` dans le contexte du driver, le port UART devient la console principale.

Le pilote surcharge alors la fonction système `write()` afin d'intercepter et de rediriger les appels à `printf()` ou `xil_printf()` directement vers le buffer de transmission de ce port UART.

---

## Structure de données

Le fonctionnement du pilote repose sur une structure principale de contexte :

```c
uart_ps_context_t
```

Cette structure encapsule :

- une instance du pilote matériel `XUartPs` ;
- l'identifiant matériel (`device_id`) ;
- le débit binaire (`baudrate`) ;
- deux instances de `uart_ring_buffer_t` dédiées aux buffers **RX** et **TX** ;
- l'indicateur booléen `is_console`.

---

## Interface API standard

Le pilote respecte la signature requise par la table des périphériques de l'**IO_MANAGER**.

### Initialisation

```c
int UART_PS_Init(void *instance);
```

Cette fonction :

- remet à zéro les index des buffers ;
- vérifie la configuration matérielle ;
- initialise le périphérique ;
- effectue un auto-test ;
- configure le débit (`baudrate`) ;
- masque les interruptions pertinentes ;
- enregistre la console si nécessaire.

### Mise à jour

```c
void UART_PS_Update(void *instance);
```

Doit être appelée périodiquement afin de transférer les données du buffer **TX** applicatif vers le registre de transmission matériel.

---

## API applicative

### Envoyer un octet

```c
void UART_PS_SendByte(uart_ps_context_t *ctx, u8 symbol);
```

Ajoute un octet à la fin du buffer de transmission.

### Envoyer un buffer

```c
void UART_PS_SendBuffer(uart_ps_context_t *ctx, const u8 *data, u16 len);
```

Ajoute une séquence de `len` octets dans le buffer de transmission.

### Lire un octet

```c
u8 UART_PS_GetByte(uart_ps_context_t *ctx, u8 *c);
```

Récupère et retire le plus ancien octet du buffer de réception.

**Valeur de retour :**

- `1` : lecture réussie ;
- `0` : buffer vide.

### Obtenir l'espace libre du buffer TX

```c
u16 UART_PS_TxFreeSpace(uart_ps_context_t *ctx);
```

Retourne le nombre d'emplacements encore disponibles dans le buffer de transmission.