# Driver LIDAR LD19 / LD06

Ce dossier contient le pilote du lidar LD19 (ou LD06, protocole strictement compatible — même trame UART 230400 bauds, même CRC8) connecté via une IP VHDL maison (`lidar_top_for_dma`, cf `opossum_hw`) plutôt que directement en UART logiciel : le parsing de trame, le filtrage distance/angle/intensité et un éventuel clustering sont faits **dans le PL**, et seuls des points déjà décodés (mot 32 bits `distance|angle`) remontent au CPU via un AXI DMA.

## Chaîne matérielle

```
LD19/LD06 (UART 230400 bds) -> uart_rx -> lidar_parser_ld06 -> lidar_filter -> lidar_cluster -> AXI-Stream -> axi_dma_0 (S2MM) -> DDR
```

* **`uart_rx` + `lidar_parser_ld06`** : reconstituent les trames du protocole LD19/LD06 (header `0x54`, CRC8) et sortent un flux de points `distance (mm) | angle (0,01°) | intensité`.
* **`lidar_filter`** : filtre distance/angle/intensité configurable par registres AXI4-Lite (cf carte des registres ci-dessous) ; un point hors plage ressort avec `distance = 0` (invalide) mais garde son angle, pour préserver l'alignement fixe de 12 points/paquet.
* **`lidar_cluster`** : en mode passthrough (`CL_CTRL = 0`, réglage par défaut), laisse passer le nuage complet inchangé. Un mode clustering (`CL_CTRL = 1`) existe côté VHDL mais **n'est pas décodé par ce driver** (cf avertissement plus bas).
* **`axi_dma_0`** : **S2MM uniquement**, mode *Direct Register* (pas de Scatter-Gather) — un transfert = un paquet de 12 points (48 octets).

Le registre de configuration/statut (`lidar_filter_regs`) est mappé en AXI4-Lite sur `XPAR_LIDAR_TOP_FOR_DMA_0_BASEADDR`.

### Carte des registres (`lidar_filter_regs`)

| Offset | Accès | Registre | Description |
|---|---|---|---|
| `0x00` | RW | `DIST_MIN` | Distance min (mm) |
| `0x04` | RW | `DIST_MAX` | Distance max (mm) |
| `0x08` | RW | `ANGLE_MIN` | Angle min (0,01°) |
| `0x0C` | RW | `ANGLE_MAX` | Angle max (0,01°) |
| `0x10` | RW | `INTENSITY_MIN` | Intensité min |
| `0x14` | RW | `CTRL` | bit0 = filtre actif |
| `0x18` | RO | `FRAME_COUNT` | Trames CRC OK (parseur) |
| `0x1C` | RO | `ERROR_COUNT` | Erreurs CRC/VerLen/timeout |
| `0x20` | RO | `SPEED` | Vitesse de rotation (°/s) |
| `0x24` | RO | `UART_FERR` | Erreurs de framing UART |
| `0x28` | RW | `CL_CTRL` | bit0 : 0 = nuage complet, 1 = clusters |
| `0x2C` | RW | `CL_BREAK` | Seuil de rupture clustering |
| `0x30` | RW | `CL_WALL` | Largeur mur (mm) |
| `0x34` | RW | `CL_PARAMS` | Paramètres clustering |
| `0x38` | RO | `CLUSTER_COUNT` | Clusters émis |

## Architecture logicielle

Le driver s'intègre dans l'`IO_MANAGER` (device `LIDAR_LD19`, `CORE_CPU0`, cf [IO_config.h](../../IO_config.h)) avec un fonctionnement **piloté par interruption** :

* **`LIDAR_LD19_Init()`** : recherche/config de l'instance `XAxiDma`, vérifie qu'elle est bien en mode *Direct Register* (pas SG), écrit les registres de filtre par défaut, active les interruptions DMA (`XAxiDma_IntrEnable`, masque complet), amorce la première réception.
* **`LIDAR_LD19_IntrHandler()`** *(irq_handler de `IO_DEVICE_TABLE`, branchée sur `XPAR_FABRIC_AXI_DMA_0_S2MM_INTROUT_INTR`)* : acquitte l'IRQ, gère l'erreur DMA (reset + réamorçage), sinon décode le paquet reçu (12 points), l'accumule dans le scan en cours, réamorce immédiatement le transfert suivant. **Ne fait aucune E/S** (pas de `xil_printf`/Ethernet) — travail volontairement minimal pour rester rapide en contexte ISR.
* **`LIDAR_LD19_Update()`** *(update de `IO_DEVICE_TABLE`, boucle principale)* : détecte qu'un nouveau scan complet est disponible (comparaison de `scan_id`) et déclenche, uniquement dans ce cas, l'affichage Teleplot et/ou l'envoi Ethernet — c'est-à-dire tout ce qui touche à de l'E/S et ne doit **pas** se faire depuis l'ISR (`eth_send_frame()`/`eth_printf()` s'appuient sur un tampon interne non réentrant, cf [DRIVER_ETH](../DRIVER_ETH/README.md)).
* **`LIDAR_LD19_Deinit()`** : coupe les interruptions DMA et reset le contrôleur — appelée automatiquement par `IO_Manager_SetDeviceStateByName("LIDAR_LD19", 0)` (commande `DRVDIS`, cf [APP_DRIVER_BRIDGE](../../APP_DRIVER_BRIDGE/README.md)) si le lidar n'est pas branché ou pas utilisé.

### Scan complet (tour à 360°)

Un "scan" (`lidar_scan_t`) regroupe tous les points valides d'un tour complet, détecté par un saut d'angle en arrière de plus de 180° (`LIDAR_LD19_WRAP_THRESHOLD_CDEG`), avec :
* `points[]` / `count` : nuage du tour (`LIDAR_LD19_MAX_POINTS_PER_SCAN = 600` points max, marge par rapport aux ~450 points/tour typiques du LD19 à 10 Hz),
* `timestamp_ms` : horodatage (`Timer_ms1`) à la fin du tour,
* `scan_id` : identifiant incrémental (0 = aucun scan reçu depuis le démarrage).

Double-buffer ping-pong (`scan_buf[2]`, `building_idx`/`ready_idx`) : l'ISR remplit `building_idx` et bascule `ready_idx` en fin de tour ; `LIDAR_LD19_GetLastScan()` ne lit que `ready_idx`, sans risque de lecture partielle.

```c
const lidar_scan_t *scan = LIDAR_LD19_GetLastScan(&Lidar_Ld19_Ctx);
if (scan != NULL) {
    // scan->points[0..scan->count-1], scan->timestamp_ms, scan->scan_id
}
```

## API applicative

* **`LIDAR_LD19_GetLastScan(ctx)`** : dernier scan complet disponible (`NULL` si aucun reçu depuis le démarrage).
* **`LIDAR_LD19_SetConfig(ctx, const lidar_ld19_config_t*)`** / **`LIDAR_LD19_GetConfig(ctx, lidar_ld19_config_t*)`** : réglage à chaud des filtres distance/angle/intensité et de leur activation, effet immédiat (écriture registre directe, pas besoin de réinitialiser le driver).
  ⚠️ Le champ `cluster_mode` de `lidar_ld19_config_t` bascule le registre `CL_CTRL`, mais **le décodage côté C ne sait lire que le format "nuage complet"** — l'activer sans adapter `LIDAR_LD19_Update()`/l'ISR produit des points incohérents. À laisser à 0 tant que le décodage clusters n'est pas implémenté.
* **`LIDAR_LD19_SetEthernetStreaming(ctx, enable)`** : active/désactive l'envoi des scans complets par Ethernet (voir ci-dessous).
* **`LIDAR_LD19_PrintScanUart(ctx)`** : print de test/bring-up (résumé du dernier scan sur la console UART) — à appeler explicitement depuis l'application (ex. `App_Loop()` de `core0_loop.c`, throttlé à 1 Hz), **pas** une fonction de l'`IO_Manager`. Outil de vérification manuelle, indépendant du flux Teleplot/Ethernet.

## Sorties disponibles

### Teleplot (console UART)

Format [Teleplot](https://teleplot.fr) `>lidar:x:y|xy` (nuage de points seulement, pas de trace numérique `scan_id`/`timestamp` pour ne pas polluer la console). Envoyé automatiquement par `LIDAR_LD19_Update()` dès qu'un scan complet est disponible, décimé (`LIDAR_LD19_SCAN_TELEPLOT_DECIMATION`, 1 point sur 8 par défaut) car un scan entier (~450 points) dépasserait largement le débit de la console UART (115200 bauds ≈ 11,5 ko/s, cf `UART_COMM_BAUDRATE` dans `IO_config.h`) s'il était envoyé en une seule rafale.

### Ethernet (`ETH_MSG_LIDAR_SCAN_CHUNK`)

Activé via `LIDAR_LD19_SetEthernetStreaming(ctx, 1)`. Un scan complet (~450 points, ~1800 octets) dépasse la limite `ETH_MAX_PAYLOAD` (512 octets, cf [ETH_protocol.h](../../ETH_protocol.h)) : il est donc découpé en morceaux (`LIDAR_LD19_ETH_POINTS_PER_CHUNK = 100` points/chunk, ~415 octets max avec l'en-tête) envoyés via `eth_send_frame()` sur le canal `ETH_CHANNEL_TELEMETRY`. Chaque chunk transporte `lidar_id`, `scan_id`, `timestamp_ms`, `chunk_index`/`chunk_count` (pour la réassemblage côté Raspberry Pi) et `point_count` points. Envoyé uniquement depuis `LIDAR_LD19_Update()` (boucle principale), jamais depuis l'ISR.

## Multi-lidar

Toute l'état spécifique à une instance (buffers DMA, scan en cours, dernier scan consommé, activation du streaming Ethernet, `lidar_id`...) vit dans `lidar_ld19_context_t` — aucune variable statique locale aux fonctions du driver. Ajouter un deuxième lidar ne nécessite **aucune modification** de `lidar_ld19.c`/`.h` :

1. Une nouvelle instance `lidar_ld19_context_t` (nouveau `.dma_device_id`, `.regs_base`, `.lidar_id`) dans [IO_globals.c](../../IO_globals.c).
2. Une nouvelle entrée dans `IO_DEVICE_TABLE` (nouveau `.name`, ex. `"LIDAR_LD19_2"`, nouvel `.irq_id` si le second AXI DMA a sa propre ligne IRQ) dans [IO_config.h](../../IO_config.h).

Le `lidar_id` (inclus dans les trames Ethernet) permet au récepteur de distinguer les lidars entre eux.

## Point d'attention matériel : interruption DMA

`LIDAR_LD19_INTR_ID` pointe vers `XPAR_FABRIC_AXI_DMA_0_S2MM_INTROUT_INTR` (ID GIC 62), câblée **directement** (pas de partage via `xlconcat`) dans le hardware actuellement exporté — distincte de l'IRQ `AXI_UARTLITE_0` (ID 61). Si le block design est un jour modifié pour combiner ces deux interruptions via un `xlconcat` et ré-exporté, `LIDAR_LD19_IntrHandler()` devra être adapté pour démultiplexer (il vérifie déjà défensivement que le statut IRQ lu concerne bien le DMA avant de traiter quoi que ce soit).

## Debug

Prints de diagnostic (erreurs d'init, d'armement DMA, d'IRQ) désactivés par défaut — décommenter `#define LIDAR_LD19_DEBUG` dans `lidar_ld19.h`. Indépendant de `LIDAR_LD19_PRINT_POINTS` (affichage Teleplot) et de `LIDAR_LD19_PrintScanUart()` (test manuel), qui restent actifs même sans ce define.

## Voir aussi

* [DRIVER_ETH](../DRIVER_ETH/README.md) — driver Ethernet utilisé pour le streaming des scans (`eth_send_frame`, contrainte non-réentrante)
* [IO_MANAGER](../README.md) — table des périphériques et cycle de vie générique
* [APP_DRIVER_BRIDGE](../../APP_DRIVER_BRIDGE/README.md) — commandes `DRVEN "LIDAR_LD19"` / `DRVDIS "LIDAR_LD19"` / `DRVLIST`
* [opossum_common](../../README.md) — vue d'ensemble de l'architecture
