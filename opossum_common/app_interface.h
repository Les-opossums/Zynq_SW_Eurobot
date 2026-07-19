#ifndef APP_INTERFACE_H
#define APP_INTERFACE_H

// Appelée une seule fois avant le while(1)
void App_Init(void);

// Appelée en boucle dans le while(1)
void App_Loop(void);

#endif /* APP_INTERFACE_H */