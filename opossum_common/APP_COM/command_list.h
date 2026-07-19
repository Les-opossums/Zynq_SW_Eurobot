#ifndef COMMAND_LIST_H
#define COMMAND_LIST_H

#include "xil_types.h"

typedef struct {
    char* Name;
    uint8_t (*Func)(void);
} Command;

extern const Command Command_List[];
extern const uint16_t Command_List_Length;

#endif /* COMMAND_LIST_H */