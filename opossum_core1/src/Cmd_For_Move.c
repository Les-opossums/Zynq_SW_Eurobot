#include "main.h"

#define MOVE_SEQ_FIFO_SIZE 16

typedef struct {
    Position positions[MOVE_SEQ_FIFO_SIZE];
    int head;
    int tail;
    int count;
} MoveSeqFifo;

static MoveSeqFifo move_seq_fifo = {0};
static int         move_seq_active = 0;  // 1 = un move est en cours

static int move_seq_push(Position pos) {
    if (move_seq_fifo.count >= MOVE_SEQ_FIFO_SIZE) return 0;
    move_seq_fifo.positions[move_seq_fifo.tail] = pos;
    move_seq_fifo.tail = (move_seq_fifo.tail + 1) % MOVE_SEQ_FIFO_SIZE;
    move_seq_fifo.count++;
    return 1;
}

static int move_seq_pop(Position *pos) {
    if (move_seq_fifo.count == 0) return 0;
    *pos = move_seq_fifo.positions[move_seq_fifo.head];
    move_seq_fifo.head = (move_seq_fifo.head + 1) % MOVE_SEQ_FIFO_SIZE;
    move_seq_fifo.count--;
    return 1;
}

static void move_seq_clear(void) {
    move_seq_fifo.head  = 0;
    move_seq_fifo.tail  = 0;
    move_seq_fifo.count = 0;
    move_seq_active     = 0;
}

//MOVE
uint8_t Move_Cmd(void) {
    if (AU_state) {
        xil_printf("INVALID COMMAND : AU\n");
        return 0;
    }else{
        move_seq_clear();
        if (Get_Param_Float(&local_data.cmd_position.x))     return PARAM_ERROR_CODE;
        if (Get_Param_Float(&local_data.cmd_position.y))     return PARAM_ERROR_CODE;
        if (Get_Param_Float(&local_data.cmd_position.t))     return PARAM_ERROR_CODE;
        // ecriture dans la mémoire partagée
        SEND_FIELD(&local_data, cmd_position);
        return 0;
    }
}

// SPEED
uint8_t Speed_Cmd(void) {
    if (AU_state) {
        printf("INVALID COMMAND : AU\n");
        return 0;
    }else{
        if (Get_Param_Float(&local_data.cmd_speed.vx)) return PARAM_ERROR_CODE;
        if (Get_Param_Float(&local_data.cmd_speed.vy)) return PARAM_ERROR_CODE;
        if (Get_Param_Float(&local_data.cmd_speed.vt)) return PARAM_ERROR_CODE;

        // ecriture dans la mémoire partagée
        SEND_FIELD(&local_data, cmd_speed);
        return 0;
    }
}

// ASPEED
uint8_t Absolute_Speed_Cmd(void) {
    if (AU_state) {
        printf("INVALID COMMAND : AU\n");
        return 0;
    }else{
        if (Get_Param_Float(&local_data.cmd_abs_speed.vx)) return PARAM_ERROR_CODE;
        if (Get_Param_Float(&local_data.cmd_abs_speed.vy)) return PARAM_ERROR_CODE;
        if (Get_Param_Float(&local_data.cmd_abs_speed.vt)) return PARAM_ERROR_CODE;
        
        // ecriture dans la mémoire partagée
        SEND_FIELD(&local_data, cmd_abs_speed);
        return 0;
    }
}

// FREE
uint8_t FREE_Cmd(void) {
    //ecriture dans la mémoire partagée
    local_data.asserv_mode = 0; // Mode libre
    SEND_FIELD(&local_data, asserv_mode);
    return 0;
}

// block
uint8_t BLOCK_Cmd(void) {
    if (AU_state) { 
        printf("INVALID COMMAND : AU\n"); 
        return 0; 
    }else{
        move_seq_clear();
        local_data.asserv_mode = 4;
        SEND_FIELD(&local_data, asserv_mode);
        return 0;
    }
}

uint8_t Start_Wheel_FF_Calibration_Cmd(void) {
    if (AU_state) { 
        printf("INVALID COMMAND : AU\n"); 
        return 0; 
    }else{
        local_data.asserv_mode = 5;
        SEND_FIELD(&local_data, asserv_mode);
        return 0;
    }
}

uint8_t Asserv_Done_Cmd(void) {
    // lecture de la mémoire partagée
    CHECK_FIELD(&local_data, asserv_done);
    printf("%d\n", local_data.asserv_done);
    return 0;
}

uint8_t Get_Pos_Cmd(void) {
    // lecture de la mémoire partagée
    if (CHECK_FIELD(&local_data, kalman_out)) {
        printf("GETPOS ");
        printf("%.4f ", (double)  (local_data.kalman_out.x));
        printf("%.4f ", (double)  (local_data.kalman_out.y));
        printf("%.4f\n", (double) (local_data.kalman_out.t));
    } else {
        printf("GETPOS ERRROR: Position not valid\n");
    }
    return 0;
}

uint8_t Get_Odo_Cmd(void) {
    // lecture de la mémoire partagée
    int status1;
    int status2;
    status1 = CHECK_FIELD(&local_data, kalman_out);
    status2 = CHECK_FIELD(&local_data, speed_robot);

    if(!status1 || !status2) {
        printf("GETODO ERROR: Position or speed not valid\n");
        return 0;
    }else{
        printf("ODO ");
        printf("%.4f ", (double)(local_data.kalman_out.x));
        printf("%.4f ", (double)(local_data.kalman_out.y));
        printf("%.4f ", (double)(local_data.kalman_out.t));
        printf("%.4f ", (double)(local_data.speed_robot.vx));
        printf("%.4f ", (double)(local_data.speed_robot.vy));
        printf("%.4f\n", (double)(local_data.speed_robot.vt));
        return 0;
    }
}

uint8_t SET_Cmd(void) {
    // Récupération de la position à définir
    if (Get_Param_Float(&local_data.set_pos.x)) return PARAM_ERROR_CODE;
    if (Get_Param_Float(&local_data.set_pos.y)) return PARAM_ERROR_CODE;
    if (Get_Param_Float(&local_data.set_pos.t)) return PARAM_ERROR_CODE;
    // ecriture de la mémoire partagée
    SEND_FIELD(&local_data, set_pos);
    return 0;
}

uint8_t SET0_Cmd(void) {
    local_data.set_pos.x = 0.0f; // position x
    local_data.set_pos.y = 0.0f; // position y
    local_data.set_pos.t = 0.0f; // position t
    // ecriture de la mémoire partagée
    SEND_FIELD(&local_data, set_pos);
    return 0;
}

uint8_t Set_Lidar_Cmd(void) {
    // Récupération des mesures LIDAR
    if (Get_Param_Float(&local_data.set_lidar.lidar_position_x))     return PARAM_ERROR_CODE;
    if (Get_Param_Float(&local_data.set_lidar.lidar_position_y))     return PARAM_ERROR_CODE;
    if (Get_Param_Float(&local_data.set_lidar.lidar_position_t))     return PARAM_ERROR_CODE; 
    if (Get_Param_u32(&local_data.set_lidar.delay))                  return PARAM_ERROR_CODE;
    // printf("lidar pos: %f %f %f\n", 
    //         (double)(local_data.set_lidar.lidar_position_x), 
    //         (double)(local_data.set_lidar.lidar_position_y), 
    //         (double)(local_data.set_lidar.lidar_position_t));
    // ecriture dans la mémoire partagée
    SEND_FIELD(&local_data, set_lidar);
    return 0;
}

uint8_t Set_Lidar_Noise_Cmd(void) {
    if(Get_Param_Float(&local_data.kalman_noise_lidar.process_noise_lidar_x)) return PARAM_ERROR_CODE;
    if(Get_Param_Float(&local_data.kalman_noise_lidar.process_noise_lidar_y)) return PARAM_ERROR_CODE;
    if(Get_Param_Float(&local_data.kalman_noise_lidar.process_noise_lidar_t)) return PARAM_ERROR_CODE;

    SEND_FIELD(&local_data, kalman_noise_lidar);
    return 0;
}

uint8_t Set_Camera_1_Cmd(void) {
    // Récupération des mesures caméra
    if (Get_Param_Float(&local_data.set_camera_1.camera_position_x))     return PARAM_ERROR_CODE;
    if (Get_Param_Float(&local_data.set_camera_1.camera_position_y))     return PARAM_ERROR_CODE;
    if (Get_Param_Float(&local_data.set_camera_1.camera_position_t))     return PARAM_ERROR_CODE; 
    if (Get_Param_u32(&local_data.set_camera_1.delay))                  return PARAM_ERROR_CODE;
    if(Get_Param_Float(&local_data.set_camera_1.noise_x))                 return PARAM_ERROR_CODE;
    if(Get_Param_Float(&local_data.set_camera_1.noise_y))                 return PARAM_ERROR_CODE;
    if(Get_Param_Float(&local_data.set_camera_1.noise_t))                 return PARAM_ERROR_CODE;
    // ecriture dans la mémoire partagée
    SEND_FIELD(&local_data, set_camera_1);
    return 0;
}

uint8_t Set_Camera_2_Cmd(void) {
    // Récupération des mesures caméra
    if (Get_Param_Float(&local_data.set_camera_2.camera_position_x))     return PARAM_ERROR_CODE;
    if (Get_Param_Float(&local_data.set_camera_2.camera_position_y))     return PARAM_ERROR_CODE;
    if (Get_Param_Float(&local_data.set_camera_2.camera_position_t))     return PARAM_ERROR_CODE; 
    if (Get_Param_u32(&local_data.set_camera_2.delay))                  return PARAM_ERROR_CODE;
    if(Get_Param_Float(&local_data.set_camera_2.noise_x))                 return PARAM_ERROR_CODE;
    if(Get_Param_Float(&local_data.set_camera_2.noise_y))                 return PARAM_ERROR_CODE;
    if(Get_Param_Float(&local_data.set_camera_2.noise_t))                 return PARAM_ERROR_CODE;

    // ecriture dans la mémoire partagée
    SEND_FIELD(&local_data, set_camera_2);
    return 0;
}

uint8_t Set_Camera_3_Cmd(void) {
    // Récupération des mesures caméra
    if (Get_Param_Float(&local_data.set_camera_3.camera_position_x))     return PARAM_ERROR_CODE;
    if (Get_Param_Float(&local_data.set_camera_3.camera_position_y))     return PARAM_ERROR_CODE;
    if (Get_Param_Float(&local_data.set_camera_3.camera_position_t))     return PARAM_ERROR_CODE; 
    if (Get_Param_u32(&local_data.set_camera_3.delay))                  return PARAM_ERROR_CODE;
    if(Get_Param_Float(&local_data.set_camera_3.noise_x))                 return PARAM_ERROR_CODE;
    if(Get_Param_Float(&local_data.set_camera_3.noise_y))                 return PARAM_ERROR_CODE;
    if(Get_Param_Float(&local_data.set_camera_3.noise_t))                 return PARAM_ERROR_CODE;
    // ecriture dans la mémoire partagée
    SEND_FIELD(&local_data, set_camera_3);
    return 0;
}

// VMAX
uint8_t VMAX_Cmd(void) {
    if (Get_Param_Float(&local_data.vmax))     return PARAM_ERROR_CODE;
    // ecriture dans la mémoire partagée
    SEND_FIELD(&local_data, vmax);
    return 0;
}

// VTMAX
uint8_t VTMAX_Cmd(void) {
    if (Get_Param_Float(&local_data.vtmax))    return PARAM_ERROR_CODE;
    // ecriture dans la mémoire partagée
    SEND_FIELD(&local_data, vtmax);
    return 0;
}

// AMAX
uint8_t AMAX_Cmd(void) {
    if (Get_Param_Float(&local_data.amax))    return PARAM_ERROR_CODE;  //almax
    // ecriture dans la mémoire partagée
    SEND_FIELD(&local_data, amax);
    return 0;
}

uint8_t PWM_Func(void)
{
    if (Get_Param_Float(&local_data.cmd_esc.command1))    return PARAM_ERROR_CODE;
    if (Get_Param_Float(&local_data.cmd_esc.command2))    return PARAM_ERROR_CODE;
    if (Get_Param_Float(&local_data.cmd_esc.command3))    return PARAM_ERROR_CODE;
    if (Get_Param_Float(&local_data.cmd_esc.command4))    return PARAM_ERROR_CODE;
    
    // ecriture dans la mémoire partagée
    SEND_FIELD(&local_data, cmd_esc);
    return 0;
}

uint8_t Enable_Kalman_Cmd(void) {
    if (Get_Param_u32(&local_data.enable_kalman.enable_lidar_kalman))         return PARAM_ERROR_CODE;
    if (Get_Param_u32(&local_data.enable_kalman.enable_camera_kalman))         return PARAM_ERROR_CODE;
    // ecriture dans la mémoire partagée
    SEND_FIELD(&local_data, enable_kalman);
    return 0;
}

uint8_t Set_Odo_Spacing_Cmd(void) {
    if (Get_Param_Float(&local_data.odo_spacing)) return PARAM_ERROR_CODE;

    // ecriture dans la mémoire partagée
    SEND_FIELD(&local_data, odo_spacing);
    return 0;
}


int auto_printpos_en = 1; // si on active l'envoi de la position
uint32_t auto_printpos_delay_uart = 100; // en ms
uint32_t Last_Timer_print_pos_uart = 0; // dernier envoi de la position
uint32_t auto_printpos_delay_eth = 1; // en ms
uint32_t Last_Timer_print_pos_eth = 0; // dernier envoi de la position

uint8_t Activate_Position_Sending_Func (void) {
    uint32_t state;
    if (Get_Param_u32(&state)) return PARAM_ERROR_CODE;
    auto_printpos_en = (state != 0);
    Last_Timer_print_pos_uart = Timer_ms1;
    Last_Timer_print_pos_eth  = Timer_ms1;

    uint32_t Delay;
    if (!Get_Param_u32(&Delay)) {   // s'il y a un 2eme param, on s'en sert comme delai entre les prints UART (en ms, of course)
        auto_printpos_delay_uart = Delay;
    }
    return 0;
}

static void compute_robot_state(eth_payload_robot_state_t *rs, float *speed_linear, float *speed_direction) {
    *speed_linear    = sqrtf(local_data.speed_robot.vx * local_data.speed_robot.vx + local_data.speed_robot.vy * local_data.speed_robot.vy);
    *speed_direction = atan2f(local_data.speed_robot.vy, local_data.speed_robot.vx);

    rs->timestamp_ms    = (uint32_t)Timer_ms1;
    rs->x                = local_data.kalman_out.x;
    rs->y                = local_data.kalman_out.y;
    rs->theta            = local_data.kalman_out.t;
    rs->speed_linear     = *speed_linear;
    rs->speed_direction  = *speed_direction;
    rs->speed_angular    = local_data.speed_robot.vt;
    rs->motion_done      = (uint8_t)local_data.asserv_done; // <-- a adapter au vrai nom/type du champ chez toi
}

void Print_Position_loop(void) {
    if (!auto_printpos_en) {
        return;
    }
    if (!CHECK_FIELD(&local_data, kalman_out) || !CHECK_FIELD(&local_data, speed_robot)) {
        // printf("POS ERROR: Position or speed not valid\n");
        return;
    }

    if ((Timer_ms1 - Last_Timer_print_pos_uart) >= auto_printpos_delay_uart) {
        Last_Timer_print_pos_uart = Timer_ms1;

        float speed_linear    = sqrtf(local_data.speed_robot.vx * local_data.speed_robot.vx + local_data.speed_robot.vy * local_data.speed_robot.vy);
        float speed_direction = atan2f(local_data.speed_robot.vy, local_data.speed_robot.vx);
        printf("ROBOTDATA %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f\n", local_data.kalman_out.x, local_data.kalman_out.y, local_data.kalman_out.t, speed_linear, speed_direction, local_data.speed_robot.vt);
    }

    if ((Timer_ms1 - Last_Timer_print_pos_eth) >= auto_printpos_delay_eth) {
        Last_Timer_print_pos_eth = Timer_ms1;

        eth_payload_robot_state_t rs;
        float speed_linear, speed_direction;
        compute_robot_state(&rs, &speed_linear, &speed_direction);
        eth_send_robot_state(&rs);
    }
}

/*############################################################################*/
/*                         SPEEDTEST - Réglage PID vitesse                    */
/*                                                                            */
/* Commande : SPEEDTEST vx vy vt duration_ms [print_period_ms]               */
/*   - vx, vy, vt       : consigne de vitesse robot (m/s, m/s, rad/s)        */
/*   - duration_ms      : durée d'application de la consigne (ms)            */
/*   - print_period_ms  : période d'impression (ms, défaut = 10)             */
/*                                                                            */
/* Format de sortie :                                                         */
/*   ROBOTDATA vx_cmd vy_cmd vt_cmd vx vy vt                                 */
/*     - vx/vy/vt_cmd : consigne contrainte (après limiteur d'accélération)  */
/*     - vx/vy/vt     : vitesse mesurée par odométrie                        */
/*############################################################################*/
 
// Variables d'état internes de la commande SPEEDTEST
static int      speed_timed_active     = 0;
static uint32_t speed_timed_end_time   = 0;  // Timer_ms1 à partir duquel on passe en FREE
static uint32_t speed_timed_last_print = 0;  // dernier instant d'impression
static uint32_t speed_timed_print_per  = 10; // période entre deux prints (ms)
 
// SPEEDTEST vx vy vt duration_ms [print_period_ms]
uint8_t Speed_Timed_Cmd(void) {
    if (AU_state) {
        printf("INVALID COMMAND : AU\n");
        return 0;
    }
 
    // --- Paramètres obligatoires
    float vx, vy, vt;
    uint32_t duration_ms;
 
    if (Get_Param_Float(&vx))        return PARAM_ERROR_CODE;
    if (Get_Param_Float(&vy))        return PARAM_ERROR_CODE;
    if (Get_Param_Float(&vt))        return PARAM_ERROR_CODE;
    if (Get_Param_u32(&duration_ms)) return PARAM_ERROR_CODE;
 
    // --- Paramètre optionnel : période d'impression
    uint32_t print_period_ms = 10;
    Get_Param_u32(&print_period_ms); // ignoré si absent
 
    // --- Envoi de la consigne vers Core1 via mémoire partagée
    local_data.cmd_speed.vx = vx;
    local_data.cmd_speed.vy = vy;
    local_data.cmd_speed.vt = vt;
    SEND_FIELD(&local_data, cmd_speed);
 
    // --- Armement du timer
    speed_timed_end_time   = Timer_ms1 + duration_ms;
    speed_timed_print_per  = (print_period_ms > 0) ? print_period_ms : 10;
    speed_timed_last_print = Timer_ms1;
    speed_timed_active     = 1;
 
    printf("SPEEDTEST START vx=%.3f vy=%.3f vt=%.3f dur=%lums print=%lums\n",
           (double)vx, (double)vy, (double)vt,
           (unsigned long)duration_ms,
           (unsigned long)speed_timed_print_per);
 
    return 0;
}
 
// À appeler dans le main loop au même niveau que Print_Position_loop()
void Speed_Timed_Loop(void) {
    if (!speed_timed_active) return;
 
    uint32_t now = Timer_ms1;
 
    // Rafraîchissement des données Core1 → Core0 dans local_data
    // (CHECK_FIELD consomme le flag : on ne lit que si une nouvelle valeur est dispo)
    CHECK_FIELD(&local_data, cmd_speed_constrained);
    CHECK_FIELD(&local_data, speed_robot);
 
    // --- Fin de durée : passage en FREE
    if ((int32_t)(now - speed_timed_end_time) >= 0) {
        speed_timed_active = 0;
 
        local_data.asserv_mode = 0; // FREE
        SEND_FIELD(&local_data, asserv_mode);
 
        // Dernier print avec les valeurs courantes
        // printf("ROBOTDATA %.4f %.4f %.4f %.4f %.4f %.4f\n",
        //        (double)local_data.cmd_speed_constrained.vx,
        //        (double)local_data.cmd_speed_constrained.vy,
        //        (double)local_data.cmd_speed_constrained.vt,
        //        (double)local_data.speed_robot.vx,
        //        (double)local_data.speed_robot.vy,
        //        (double)local_data.speed_robot.vt);
 
        printf("SPEEDTEST DONE\n");
        return;
    }
 
    // --- Print périodique
    if ((now - speed_timed_last_print) >= speed_timed_print_per) {
        speed_timed_last_print = now;
 
        // Format : ROBOTDATA vx_cmd vy_cmd vt_cmd vx_meas vy_meas vt_meas
        // printf("ROBOTDATA %.4f %.4f %.4f %.4f %.4f %.4f\n",
        //        (double)local_data.cmd_speed_constrained.vx,  // consigne contrainte X
        //        (double)local_data.cmd_speed_constrained.vy,  // consigne contrainte Y
        //        (double)local_data.cmd_speed_constrained.vt,  // consigne contrainte Theta
        //        (double)local_data.speed_robot.vx,             // vitesse mesurée X
        //        (double)local_data.speed_robot.vy,             // vitesse mesurée Y
        //        (double)local_data.speed_robot.vt);            // vitesse mesurée Theta
    }
}


// Appelée par la commande MOVESEX : enfile une position
uint8_t Move_Seq_Cmd(void) {
    if (AU_state) { printf("INVALID COMMAND : AU\n"); return 0; }

    Position pos;
    if (Get_Param_Float(&pos.x)) return PARAM_ERROR_CODE;
    if (Get_Param_Float(&pos.y)) return PARAM_ERROR_CODE;
    if (Get_Param_Float(&pos.t)) return PARAM_ERROR_CODE;

    if (!move_seq_push(pos)) {
        printf("MOVESEX,FIFO_FULL\n");
        return 1;
    }
    // printf("MOVESEX,queued %d\n", move_seq_fifo.count);
    return 0;
}

// À appeler dans la main loop de Core 0 — dépile et envoie le prochain move
void Move_Seq_Loop(void) {
    CHECK_FIELD(&local_data, asserv_done);   // lit asserv_done depuis Core 1

    if (move_seq_active && local_data.asserv_done) {
        move_seq_active = 0;                 // move terminé, on est prêt pour le suivant
    }

    if (!move_seq_active && move_seq_fifo.count > 0) {
        Position pos;
        if (move_seq_pop(&pos)) {
            local_data.cmd_position = pos;
            SEND_FIELD(&local_data, cmd_position);
            move_seq_active = 1;
            // printf("MOVESEX,start %.4f %.4f %.4f rem:%d\n",
            //        (double)pos.x, (double)pos.y, (double)pos.t,
            //        move_seq_fifo.count);
        }
    }
}

