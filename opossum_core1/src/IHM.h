#ifndef IHM_H
#define IHM_H

extern int yellow_state;
extern int green_state;
extern int blue_state;

extern int current_mode;
extern int timer_match;

void IHM_loop(void);

uint8_t Version_cmd(void);

#endif // IHM_H
