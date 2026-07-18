#ifndef ASSERV_MOTION_H
#define ASSERV_MOTION_H

#include "../../../opossum_common/common_type.h"

/* Mode d'asservissement actif */
typedef enum {
    ASSERV_MODE_OFF = 0,
    ASSERV_MODE_FREE,
    ASSERV_MODE_POS,
    ASSERV_MODE_SPEED,
    ASSERV_MODE_ABSOLUTE_SPEED,
    ASSERV_MODE_BREAK
} asserv_mode_t;

/* Etat d'avancement du mouvement en cours, remonte a CORE0 via IPC
 * (champ IPC_DATA->motion_done, meme encodage 0/1/2). */
typedef enum {
    MOTION_SUCCESS = 0,
    MOTION_MOVING  = 1,
    MOTION_BRAKED  = 2
} motion_status_t;

extern asserv_mode_t   asserv_mode;
extern motion_status_t motion_done;

void motion_init(void);
void motion_set_position(const Position *target);
void motion_set_speed(const Speed *speed);
void motion_set_absolute_speed(const Speed *speed);
void motion_free(void);
void motion_block(void);
void motion_step(const Position *current_position);

int Get_asserv_done(void);

#endif /* ASSERV_MOTION_H */