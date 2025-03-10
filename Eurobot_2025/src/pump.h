#ifndef __PUMP_H
#define __PUMP_H

#define MASK_PUMP_1 0x01
#define MASK_PUMP_2 0x02
#define MASK_PUMP_3 0x04
#define MASK_PUMP_4 0x08
#define MASK_PUMP_5 0x10
#define MASK_PUMP_6 0x20

void Init_Pump(void);
void Pump_Loop(void);

uint8_t Pump_cmd(void);


#endif