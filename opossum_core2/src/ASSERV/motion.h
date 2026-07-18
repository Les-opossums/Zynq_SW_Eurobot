#ifndef ASSERV_MOTION_H
#define ASSERV_MOTION_H

#include "../../../opossum_common/common_type.h"

typedef enum {
    MOTION_FREE,
    MOTION_POSITION,
    MOTION_SPEED,
    MOTION_ABSOLUTE_SPEED,
    MOTION_BLOCKED
} motion_mode_t;

void motion_init(void);
void motion_set_position(const Position *target);
void motion_set_speed(const Speed *speed);
void motion_set_absolute_speed(const Speed *speed);
void motion_free(void);
void motion_block(void);
void motion_step(const Position *current_position);

#endif /* ASSERV_MOTION_H */
