#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

// --- Paramètres matériels ---
// À REMPLACER par l'adresse exacte trouvée dans ton xparameters.h
#define WS2812B_BASEADDR    0x43C00000 // Exemple générique, mets ton XPAR_AXI_WS2812B_...
#define NBR_LED             44

// Code d'erreur pour la fonction de commande (à adapter selon ton projet)
#define PARAM_ERROR_CODE    1

// --- Structure pour le tampon logiciel des LEDs ---
typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} LED_Color_t;

// --- Déclaration des variables globales (visibles par d'autres fichiers si besoin) ---
extern LED_Color_t led[NBR_LED];
extern uint8_t led_animation_mode;

// --- Prototypes des fonctions ---
void ws2812b_set_led_rgb(uint32_t led_index, uint8_t r, uint8_t g, uint8_t b);
void LED_loop(void);
void LED_MODE(void);
void LED_AU(void);
void LED_CLASSIC_MODE(void);
uint8_t LED_cmd(void);

#endif // LED_CONTROLLER_H