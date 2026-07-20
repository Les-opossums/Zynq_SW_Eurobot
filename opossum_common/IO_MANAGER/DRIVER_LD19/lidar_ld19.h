#ifndef DRIVER_LD19_LIDAR_H
#define DRIVER_LD19_LIDAR_H

#include "xil_types.h"
#include "xaxidma.h"

/* ─── Debug ──────────────────────────────────────────────────────────────
 * Decommenter pour les messages de diagnostic du driver (erreurs d'init,
 * de (re)armement DMA, statut FRAME_COUNT/ERROR_COUNT/SPEED/UART_FERR tant
 * qu'aucun paquet DMA n'arrive...). Utile en cas de souci de cablage/
 * reception ; a laisser desactive en usage normal. Le print des points
 * recus est gere separement par LIDAR_LD19_PRINT_POINTS ci-dessous.
 */
// #define LIDAR_LD19_DEBUG

#if defined(LIDAR_LD19_DEBUG)
#include "xil_printf.h"
#define LIDAR_LD19_LOG(...) xil_printf(__VA_ARGS__)
#else
#define LIDAR_LD19_LOG(...) do {} while (0)
#endif

/* Mettre a 0 pour couper l'affichage des points (une fois la chaine
 * validee) sans toucher au reste du driver. */
#define LIDAR_LD19_PRINT_POINTS 1

/* Le LD19 sort ~4500 points/s : bien trop pour la console UART (115200
 * bauds ~= 11.5 ko/s, une ligne Teleplot fait ~25 octets -> ~460 lignes/s
 * max). On envoie donc 1 point sur LIDAR_LD19_PRINT_DECIMATION, en continu
 * (pas de pause groupee par paquet) pour un affichage fluide qui balaie
 * l'angle progressivement plutot que par a-coups. Baisser cette valeur
 * pour plus de densite (quitte a augmenter le baud de la console UART,
 * cf UART_COMM_BAUDRATE dans IO_config.h), l'augmenter si ca sature encore. */
#define LIDAR_LD19_PRINT_DECIMATION 6U

/* Nouveau mode d'affichage : au lieu de streamer en continu paquet par
 * paquet (ancien mode ci-dessus, garde en commentaire dans lidar_ld19.c),
 * on envoie le scan complet (tour a 360 deg, cf lidar_scan_t plus bas)
 * d'un seul bloc des qu'il vient de se terminer. Un scan fait ~450 points :
 * envoyes tous d'un coup, ca depasserait largement le debit de la console
 * UART (cf calcul ci-dessus) -- d'ou une decimation dediee, plus forte que
 * celle du mode continu puisqu'ici tout part en une seule rafale. */
#define LIDAR_LD19_SCAN_TELEPLOT_DECIMATION 8U

/* Filtre distance applique cote materiel (lidar_filter_regs, cf ci-dessous) :
 * les points hors [MIN, MAX] arrivent avec distance=0 (invalides), et sont
 * donc deja ignores par la suite -- pas de traitement supplementaire cote C.
 * MAX choisi large par rapport a une table Eurobot (3m x 2m, diagonale
 * ~3.6m) pour couper le bruit lointain (reflets, hors-table) sans rogner
 * le terrain de jeu. A ajuster si besoin. */
#define LIDAR_LD19_FILTER_DIST_MIN_MM 30U   /* coupe le bruit tres proche (capot du capteur) */
#define LIDAR_LD19_FILTER_DIST_MAX_MM 4000U /* coupe les points lointains parasites */

/* Format DMA "nuage complet" (cf lidar_top.vhd / lidar_filter_regs.vhd,
 * CL_CTRL=0, reglage par defaut au reset) : 1 mot 32 bits par point,
 * [31:16] distance (mm) | [15:0] angle (0,01 deg, 0..35999), 12
 * points/paquet, tlast au dernier point du paquet. */
#define LIDAR_LD19_POINTS_PER_PKT 12

/* --- Registres AXI4-Lite de lidar_filter_regs (offsets en octets) ---
 * cf opossum_hw (VHDL) : lidar_filter_regs.vhd pour le detail complet. */
#define LIDAR_LD19_REG_DIST_MIN      0x00 /* RW distance min (mm) */
#define LIDAR_LD19_REG_DIST_MAX      0x04 /* RW distance max (mm) */
#define LIDAR_LD19_REG_ANGLE_MIN     0x08 /* RW angle min (0,01 deg) */
#define LIDAR_LD19_REG_ANGLE_MAX     0x0C /* RW angle max (0,01 deg) */
#define LIDAR_LD19_REG_INTENSITY_MIN 0x10 /* RW intensite min */
#define LIDAR_LD19_REG_CTRL          0x14 /* RW bit0 = filtre actif */
#define LIDAR_LD19_REG_FRAME_COUNT   0x18 /* RO trames CRC OK (parseur) */
#define LIDAR_LD19_REG_ERROR_COUNT   0x1C /* RO erreurs CRC/VerLen/timeout */
#define LIDAR_LD19_REG_SPEED         0x20 /* RO vitesse rotation (deg/s) */
#define LIDAR_LD19_REG_UART_FERR     0x24 /* RO erreurs de framing UART */
#define LIDAR_LD19_REG_CL_CTRL       0x28 /* RW bit0 : 0=nuage complet, 1=clusters */
#define LIDAR_LD19_REG_CL_BREAK      0x2C /* RW seuil de rupture clustering */
#define LIDAR_LD19_REG_CL_WALL       0x30 /* RW largeur mur (mm) */
#define LIDAR_LD19_REG_CL_PARAMS     0x34 /* RW parametres clustering */
#define LIDAR_LD19_REG_CLUSTER_COUNT 0x38 /* RO clusters emis */

/* Un point du nuage, deja decode depuis le mot DMA brut. */
typedef struct {
    uint16_t distance_mm; /* 0 = point invalide/filtre */
    uint16_t angle_cdeg;  /* angle en centiemes de degre, 0..35999 */
} lidar_point_t;

/* ─── Scan complet (un tour a 360 deg) ───────────────────────────────────
 * Un "scan" regroupe tous les points valides d'un tour complet du LD19,
 * avec un timestamp unique (Timer_ms1 au moment ou le tour se termine) et
 * un identifiant qui s'incremente a chaque nouveau tour : c'est la brique
 * de base pour du traitement en aval (clustering, detection de bordure/
 * mur, localisation...), qui n'a besoin de raisonner qu'a l'echelle du
 * tour plutot que paquet par paquet.
 *
 * MAX_POINTS_PER_SCAN est une marge par rapport aux ~450 points/tour
 * typiques du LD19 (a 10 Hz) : si jamais un tour depasse cette taille
 * (detection de fin de tour ratee, bruit...), les points en trop sont
 * silencieusement ignores (cf lidar_ld19.c), sans deborder le tampon. */
#define LIDAR_LD19_MAX_POINTS_PER_SCAN 600

/* Detection de fin de tour : un "saut" d'angle vers l'arriere de plus de
 * cette valeur (en centiemes de degre) signe un passage par le point de
 * depart (35999 -> ~0). 18000 = 180 deg : tres large marge par rapport au
 * pas angulaire normal entre deux points consecutifs (< 1 deg), pour ne
 * jamais confondre bruit/jitter avec un vrai tour complet. */
#define LIDAR_LD19_WRAP_THRESHOLD_CDEG 18000U

typedef struct {
    lidar_point_t points[LIDAR_LD19_MAX_POINTS_PER_SCAN];
    uint16_t count;         /* nombre de points valides dans points[] */
    uint32_t timestamp_ms;  /* Timer_ms1 a la fin du tour */
    uint32_t scan_id;       /* incremente a chaque tour ; 0 = aucun scan recu depuis le demarrage */
} lidar_scan_t;

/* --- Contexte de l'instance (cf IO_config.h / IO_globals.c) ---
 * dma_device_id / regs_base sont a renseigner avant l'appel a
 * LIDAR_LD19_Init() (cf IO_config.h). rx_raw est le tampon DMA brut
 * (aligne cache), last_packet le dernier paquet decode (12 points).
 *
 * scan_buf[2] : double-buffer ping-pong pour les scans complets.
 * building_idx designe le buffer en cours de remplissage (rempli par
 * LIDAR_LD19_Update()) ; ready_idx designe le dernier scan termine, celui
 * que LIDAR_LD19_GetLastScan() renvoie. Comme le remplissage et la lecture
 * se font tous les deux depuis la boucle principale (pas d'IRQ sur ce
 * driver), il n'y a pas de risque de lecture partielle tant que
 * l'appelant ne garde pas le pointeur au-dela d'un appel a
 * LIDAR_LD19_Update(). */
typedef struct {
    u32     dma_device_id; /* XPAR_AXI_DMA_0_DEVICE_ID */
    UINTPTR regs_base;     /* XPAR_LIDAR_TOP_FOR_DMA_0_BASEADDR (AXI-Lite config/statut) */

    XAxiDma axi_dma;

    uint32_t rx_raw[LIDAR_LD19_POINTS_PER_PKT] __attribute__((aligned(64)));
    lidar_point_t last_packet[LIDAR_LD19_POINTS_PER_PKT];

    uint32_t packet_count;

    lidar_scan_t scan_buf[2];
    uint8_t  building_idx;   /* 0 ou 1 : buffer en cours de remplissage */
    uint8_t  ready_idx;      /* 0 ou 1 : dernier scan complet disponible */
    uint8_t  has_last_angle; /* 0 tant qu'aucun point n'a encore ete vu (evite une fausse fin de tour au demarrage) */
    uint16_t last_angle_cdeg;
    uint32_t next_scan_id;   /* prochain identifiant a attribuer (demarre a 1) */
} lidar_ld19_context_t;

/* --- Prototypes standards pour l'IO_Manager --- */
int  LIDAR_LD19_Init(void *instance);
void LIDAR_LD19_Update(void *instance);

/**
 * @brief Renvoie le dernier scan (tour a 360 deg) complet et timestampe.
 * @return Pointeur vers le scan, ou NULL si aucun tour complet n'a encore
 * ete recu depuis le demarrage. Le pointeur reste valide jusqu'au prochain
 * tour complet (double-buffering) : a consommer avant le prochain appel a
 * LIDAR_LD19_Update() si on veut une garantie stricte, ou en comparant
 * scan_id a chaque appel pour detecter un nouveau tour.
 */
const lidar_scan_t *LIDAR_LD19_GetLastScan(lidar_ld19_context_t *ctx);

#endif /* DRIVER_LD19_LIDAR_H */
