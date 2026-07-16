#include "../main.h"

// --- Instanciation du tableau de LEDs ---
LED_Color_t led[NBR_LED];

// --- Variables d'état et compteurs ---
uint32_t led_color = 0x0000FF;
uint32_t led_id = 0;
uint32_t led_color1 = 0x0100000;   // BRG
uint32_t led_color2 = 0xFF0000;    // BRG

uint8_t  cpt = 0;
uint8_t  led_animation_mode = 40;
uint8_t  led_mode_state = 0;

// --- Variables de Timer locales au module ---
uint32_t LED_old_timer_ms1 = 0;
uint32_t led_mode_timer_old = 0;

// --- Variables pour les animations ---
uint8_t sens_animation_au = 0;
uint8_t chargement_blue = 0;
uint8_t chargement_jaune = 0;
int validation_state = 0;
int cpt2 =  NBR_LED / 2;


// =========================================================================
// FONCTIONS DE COMMUNICATION AXI
// =========================================================================

/**
 * @brief Envoie directement la couleur sur l'IP AXI
 */
void ws2812b_set_led_rgb(uint32_t led_index, uint8_t r, uint8_t g, uint8_t b) {
    if (led_index < NBR_LED) {
        uint32_t addr = WS2812B_BASEADDR + (led_index * 4);
        
        // Ordre des couleurs adapté à ton ancien code (Blue << 16 | Red << 8 | Green)
        uint32_t color_data = (b << 16) | (r << 8) | g;
        
        Xil_Out32(addr, color_data);
    }
}


// =========================================================================
// LOGIQUE PRINCIPALE ET ANIMATIONS
// =========================================================================

/**
 * @brief Fonction à appeler régulièrement dans ta boucle principale (main)
 */
void LED_loop(void){
    // Rafraîchissement matériel toutes les 10ms
    if (Timer_ms1 - LED_old_timer_ms1 > 10){
        LED_old_timer_ms1 = Timer_ms1;
        
        // On envoie le tampon logiciel 'led[]' vers l'IP AXI
        for (int i = 0; i < NBR_LED; i++){ 
            ws2812b_set_led_rgb(i, led[i].red, led[i].green, led[i].blue);
        }
    }
    
    // Mise à jour du tampon logiciel selon l'animation en cours
    LED_MODE();
}

/**
 * @brief Met à jour le tampon de LEDs en fonction du mode
 */
void LED_MODE(void){
    switch(led_animation_mode){
        case 0: // default color (Tout éteint)
            for (int i = 0; i < NBR_LED; i++){
                led[i].red = 0;
                led[i].green = 0;
                led[i].blue = 0;
            }
            break;

        case 40: // case of AU (Aller-retour rouge)
            if(Timer_ms1 - led_mode_timer_old > 10){
                led_mode_timer_old = Timer_ms1;
                for (int i = 0; i < NBR_LED; i++){
                    if (sens_animation_au == 0){
                        if(i < cpt){
                            led[i].red = 0xFF; led[i].green = 0; led[i].blue = 0;
                        }else{
                            led[i].red = 0; led[i].green = 0; led[i].blue = 0;
                        }
                    }else{
                        if(i < cpt){
                            led[i].red = 0; led[i].green = 0; led[i].blue = 0;
                        }else{
                            led[i].red = 0xFF; led[i].green = 0; led[i].blue = 0;
                        }
                    }
                }
                cpt++;
                if(cpt > NBR_LED){ // Quand on arrive au bout, on change de sens
                    cpt = 0;
                    sens_animation_au = !sens_animation_au;
                }
            }
            break;

        case 60: // leash (Remplissage gris)
            if(Timer_ms1 - led_mode_timer_old > 20){
                led_mode_timer_old = Timer_ms1;
                cpt = (timer_match * 44) / 100000;
                if (cpt < NBR_LED){
                    for (int i = 0; i < NBR_LED; i++){
                        if(i < cpt){
                            led[i].red = 0x10; led[i].green = 0x10; led[i].blue = 0x10;
                        }else{
                            led[i].red = 0; led[i].green = 0; led[i].blue = 0;
                        }
                    }
                }
            }
            break;
    }
}

void LED_AU(void){
    led_animation_mode = 40;
}

void LED_CLASSIC_MODE(void){
    led_animation_mode = current_mode;
}

/**
 * @brief Commande depuis l'UART pour modifier une LED spécifique
 */
uint8_t LED_cmd(void) {
    u32 id;
    if (Get_Param_u32(&id)) return PARAM_ERROR_CODE;
    
    u32 red;
    if (Get_Param_u32(&red)) return PARAM_ERROR_CODE;
    
    u32 green;
    if (Get_Param_u32(&green)) return PARAM_ERROR_CODE;
    
    u32 blue;
    if (Get_Param_u32(&blue)) return PARAM_ERROR_CODE;

    if (red > 255 || green > 255 || blue > 255){
        xil_printf("Color out of range\n\r");
    }else {
        xil_printf("LED number %lu, color: %lu, %lu, %lu\n\r", id, red, green, blue);
        
        led_color = (green << 16) | (red << 8) | blue;
        
        // Met à jour la RAM CPU (sera poussée au FPGA au prochain passage dans LED_loop)
        if(id < NBR_LED) {
            led[id].red = (uint8_t)red;
            led[id].green = (uint8_t)green;
            led[id].blue = (uint8_t)blue;
        }
    }
    return 0;
}