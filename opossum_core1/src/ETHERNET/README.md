# 🌐 Driver Ethernet UDP Bare-Metal (Zynq ↔ ROS 2)

Ce dossier contient le driver réseau UDP haute performance permettant la communication entre une **carte Zynq-7000 (Bare-Metal)** et un **Raspberry Pi (ROS 2 / Linux)**.

Il est basé sur **lwIP** et utilise une architecture **Data-Driven (multiplexée)**. Le protocole réseau est totalement découplé des données applicatives du robot, garantissant une excellente évolutivité et une latence minimale.

---

# 📂 Architecture des fichiers

L'architecture est divisée en **3 couches distinctes** :

## 🔒 `ETH_driver.c` / `ETH_driver.h` — Le moteur

Gère :

- la pile **lwIP**
- les sockets **UDP**
- la création des en-têtes réseau
- le routage des messages en **O(1)**

> Ce code est générique et ne doit (presque) jamais être modifié.

---

## ⚙️ `ETH_protocol.h` — Le protocole

Définit :

- les ports UDP (canaux)
- les IDs des messages
- le **Magic Word**
- la structure du header réseau

---

## 📦 `robot_messages.h` — L'application

Le "dictionnaire" du robot.

Contient uniquement les structures de données :

- odométrie
- IMU
- heartbeat
- commandes
- etc.

> Ce fichier est le seul contrat à partager avec le code C++ du Raspberry Pi.

---

# 🚀 Guide d'utilisation rapide

## 1. Initialisation

Dans votre `main.c`, configurez le réseau puis appelez régulièrement le polling réseau.

> **Important :** `eth_driver_poll()` doit être appelé dans une boucle non bloquante, idéalement toutes les **1 à 10 ms**.

```c
#include "ETH_driver.h"
#include "robot_messages.h"

// Configuration réseau statique
static eth_driver_config_t eth_cfg = {
    .mac_addr = {0x00, 0x0a, 0x35, 0x00, 0x01, 0x12},
    .local_ip = (192u<<24) | (168u<<16) | (1u<<8) | 10u,   // 192.168.1.10
    .netmask  = (255u<<24) | (255u<<16) | (255u<<8) | 0u,
    .peer_ip  = (192u<<24) | (168u<<16) | (1u<<8) | 20u    // 192.168.1.20
};

int main(void)
{
    // Initialisation
    eth_driver_init(&eth_cfg);

    // Callback de réception (optionnel)
    eth_driver_set_cmd_handler(on_network_command_received);

    while (1)
    {
        // Logique robot...

        // Fait avancer la pile réseau lwIP
        eth_driver_poll();
    }
}
```

---

## 2. Envoyer des données

Le driver encapsule automatiquement les données dans le bon paquet réseau et choisit le port UDP approprié.

### Exemple : message texte

```c
eth_printf("Uptime : %d ms | Batterie : %d V\n", uptime, bat_voltage);
```

### Exemple : structure binaire

```c
eth_payload_odom_t my_odom = {
    .x = 1.2,
    .y = -0.5,
    .theta = 3.14
};

eth_send_frame(ETH_MSG_ODOM, &my_odom, sizeof(my_odom));
```

---

## 3. Recevoir des données

Créer une fonction callback puis l'enregistrer lors de l'initialisation.

```c
void on_network_command_received(
    eth_msg_type_t type,
    const uint8_t *payload,
    uint16_t len)
{
    if (type == ETH_MSG_CMD_GENERIC)
    {
        // Texte brut
        for (uint16_t i = 0; i < len; i++)
        {
            Interp((char)payload[i]);
        }
    }
    else if (type == ETH_MSG_ODOM_TARGET)
    {
        // Structure binaire
        eth_payload_target_t *target = (eth_payload_target_t *)payload;

        robot_set_target(target->x, target->y);
    }
}
```

---

# 🛠️ Ajouter un nouveau message (architecture Data-Driven)

Grâce à l'architecture basée sur les **X-Macros**, il n'est jamais nécessaire de modifier le code du driver.

Il suffit d'ajouter une structure puis de la déclarer dans la liste des messages.

---

## Étape 1 — Créer la structure

Dans `robot_messages.h` :

> ⚠️ Toujours utiliser `__attribute__((packed))` afin d'éviter les problèmes de padding entre le processeur **32 bits (Zynq)** et le **64 bits (Raspberry Pi)**.

```c
typedef struct __attribute__((packed))
{
    uint16_t distance_mm;
    uint16_t angle_deg;
    uint8_t quality;

} eth_payload_lidar_t;
```

---

## Étape 2 — Déclarer le message

Dans `ETH_protocol.h`, ajouter simplement une ligne dans la macro :

```c
#define ETH_MESSAGE_LIST(X)               \
    X(HEARTBEAT,  0x01, ETH_CHANNEL_TELEMETRY) \
    X(ODOM,       0x10, ETH_CHANNEL_TELEMETRY) \
    /* ... autres messages ... */         \
    X(LIDAR_DATA, 0x14, ETH_CHANNEL_TELEMETRY)
```

C'est tout.

À la compilation, le driver génère automatiquement :

- `ETH_MSG_LIDAR_DATA`
- la table de routage
- le mapping des ports UDP

Vous pouvez immédiatement envoyer votre nouvelle structure :

```c
eth_send_frame(
    ETH_MSG_LIDAR_DATA,
    &lidar_data,
    sizeof(lidar_data)
);
```

---

# ⚠️ Réception côté ROS 2 / Raspberry Pi

Les données envoyées par `eth_send_frame()` sont des **paquets binaires**, et non du texte.

Chaque paquet UDP commence par un **header de 10 octets**.

| Champ | Taille | Description |
|--------|--------|-------------|
| `magic` | 2 octets | Toujours `0xC0DE` |
| `version` | 1 octet | Version du protocole (`1`) |
| `msg_type` | 1 octet | Type du message (`0x10`, etc.) |
| `seq` | 2 octets | Numéro de séquence (détection des pertes UDP) |
| `length` | 2 octets | Taille du payload |
| `timestamp` | 2 octets | Réservé / non utilisé |

---

## Décodage côté Raspberry Pi

Le code de réception doit :

1. Lire le header.
2. Vérifier que `magic == 0xC0DE`.
3. Lire `msg_type`.
4. Utiliser le type correspondant pour interpréter le payload.

Par exemple :

- **C++** → `reinterpret_cast<>`
- **Python** → `struct.unpack()`

Le payload est ensuite directement converti vers la structure définie dans `robot_messages.h`.

---

# ✅ Résumé

Le driver sépare complètement :

- **le transport réseau** (`ETH_driver`)
- **le protocole** (`ETH_protocol`)
- **les données applicatives** (`robot_messages`)

Cette architecture permet d'ajouter de nouveaux messages sans modifier le driver, simplement en déclarant une nouvelle structure et une nouvelle entrée dans la liste des messages.






# 🛠️ Patch BSP - Forcer un lien Ethernet à 100 Mbps (RTL8201F)

Ce correctif permet de forcer le lien Ethernet à **100 Mbps** en modifiant la fonction de détection du PHY Realtek dans le **Board Support Package (BSP)** généré par **Vitis**.

L'objectif est de supprimer les appels liés au Gigabit et de n'annoncer que les capacités **10/100 Mbps**.

---

## 1. Localiser le fichier à modifier

Dans votre **Workspace Vitis**, ouvrez le projet **Platform** (ou **BSP**) puis naviguez jusqu'au fichier suivant :

```text
[Nom_du_BSP]/
└── ps7_cortexa9_0/
    └── libsrc/
        └── lwip21x_v.../
            └── src/
                └── contrib/
                    └── ports/
                        └── xilinx/
                            └── netif/
                                └── xemacpsif_physpeed.c
```

---

## 2. Remplacer la fonction `get_Realtek_phy_speed`

Ouvrez le fichier **`xemacpsif_physpeed.c`**.

Recherchez la fonction :

```c
get_Realtek_phy_speed(...)
```

Puis remplacez-la **intégralement** par le code suivant :

```c
static u32_t get_Realtek_phy_speed(XEmacPs *xemacpsp, u32_t phy_addr)
{
	u16_t control;
	u16_t status;
	u32_t timeout_counter = 0;

	xil_printf("Start PHY autonegotiation (Patched for RTL8201F)\r\n");

	// 1. Annonce des capacités 10/100 Mbps (Full/Half Duplex)
	XEmacPs_PhyRead(xemacpsp, phy_addr, IEEE_AUTONEGO_ADVERTISE_REG, &control);
	control |= IEEE_ASYMMETRIC_PAUSE_MASK;
	control |= IEEE_PAUSE_MASK;
	control |= ADVERTISE_100;
	control |= ADVERTISE_10;
	XEmacPs_PhyWrite(xemacpsp, phy_addr, IEEE_AUTONEGO_ADVERTISE_REG, control);

	// /!\ Patch : L'annonce du Gigabit a été supprimée ici pour éviter de perturber le PHY.

	// 2. Redémarrage de l'auto-négociation
	XEmacPs_PhyRead(xemacpsp, phy_addr, IEEE_CONTROL_REG_OFFSET, &control);
	control |= IEEE_CTRL_AUTONEGOTIATE_ENABLE;
	control |= IEEE_STAT_AUTONEGOTIATE_RESTART;
	XEmacPs_PhyWrite(xemacpsp, phy_addr, IEEE_CONTROL_REG_OFFSET, control);

	// 3. Reset du PHY
	XEmacPs_PhyRead(xemacpsp, phy_addr, IEEE_CONTROL_REG_OFFSET, &control);
	control |= IEEE_CTRL_RESET_MASK;
	XEmacPs_PhyWrite(xemacpsp, phy_addr, IEEE_CONTROL_REG_OFFSET, control);

	// Attente de la fin du reset
	while (1) {
		XEmacPs_PhyRead(xemacpsp, phy_addr, IEEE_CONTROL_REG_OFFSET, &control);
		if (control & IEEE_CTRL_RESET_MASK)
			continue;
		else
			break;
	}

	XEmacPs_PhyRead(xemacpsp, phy_addr, IEEE_STATUS_REG_OFFSET, &status);

	xil_printf("Waiting for PHY to complete autonegotiation.\r\n");

	// 4. Attente de la fin de l'auto-négociation
	while (!(status & IEEE_STAT_AUTONEGOTIATE_COMPLETE)) {
		sleep(1);
		timeout_counter++;

		if (timeout_counter == 30) {
			xil_printf("Auto negotiation error \r\n");
			return XST_FAILURE;
		}

		XEmacPs_PhyRead(xemacpsp, phy_addr, IEEE_STATUS_REG_OFFSET, &status);
	}

	xil_printf("autonegotiation complete \r\n");

	// 5. Patch : On ignore la lecture du registre spécifique Gigabit
	// et on force le retour à 100 Mbps.
	return 100;
}
```

---

## 3. Recompiler le projet

> **Attention**
>
> Vitis ne recompile pas toujours automatiquement le BSP lorsqu'il n'y a pas eu de modification matérielle.

Effectuez les étapes suivantes :

1. Sauvegardez le fichier.
2. Faites un clic droit sur le projet **Platform/BSP** puis sélectionnez **Build Project**.
3. Faites un clic droit sur votre projet **Application** puis sélectionnez **Build Project**.

---

## Vérification

Au prochain démarrage de la carte, la console série devrait afficher :

```text
link speed for phy address 1: 100
```

Si ce message apparaît, le patch a bien été appliqué et le lien Ethernet est désormais forcé à **100 Mbps**.