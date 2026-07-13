#ifndef WS2812B_H
#define WS2812B_H

#define AXI_LED_ADDR XPAR_AXI_GPIO_25_BASEADDR
#define AXI_LED_DATA XPAR_AXI_GPIO_26_BASEADDR

#define NBR_LED 44

#define DEFAULT_RED 0x0000FF
#define DEFAULT_GREEN 0x00FF00
#define DEFAULT_BLUE 0xFF0000

typedef struct{
   uint8_t red;
   uint8_t green;
   uint8_t blue;
}LED;

extern int validation_state;
extern uint8_t led_animation_mode;

void ws2812b_init();

void ws2812b_set_color(int led_id, LED color);
void start_transfer();

void LED_loop();
void LED_MODE();

void LED_AU(void);
void LED_CLASSIC_MODE(void);

uint8_t LED_cmd(void);

#endif // WS2812B_H
