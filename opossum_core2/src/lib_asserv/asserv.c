#include "../main.h"
#include "lib_asserv.h"


// ******************************    Variables    *******************************
int asserv_mode; // asservissement off par defaut (refer to asserv_init function)
int motion_done;

float blocked_time;

float current_stop_distance;
float default_stop_distance;

Position Wanted_Pos;
Speed Wanted_Speed;

float speed_order_d;

float v_constrained = 0;

float v_max = DEFAULT_CONSTRAINT_V_MAX;
float a_max = DEFAULT_CONSTRAINT_A_MAX;

int emergency_break_requested = 0;

/******************************    Fonctions    *******************************/

// init de tout l'asservissement
void asserv_init(void) {
	// init des autres trucs de la lib
	odo_init();

    // init acccel and speed constrainers
	speed_constrainer_init();
    acceleration_constrainer_init();

    // init PID
	pid_vitesse_init();

    // init kalman
    kalman_init(&kalman_current_state);
    kalman_fifo_init(&kalman_fifo);

	// init des consignes / modes de ce fichier :
    asserv_mode = ASSERV_MODE_OFF;
    motion_done = 0;
	blocked_time = 0;

    Wanted_Pos.x = 0;
    Wanted_Pos.y = 0;
    Wanted_Pos.t = 0;
    
	Wanted_Speed.vx = 0;
	Wanted_Speed.vy = 0;
	Wanted_Speed.vt = 0;

    current_stop_distance = DEFAULT_STOP_DISTANCE;
    default_stop_distance = DEFAULT_STOP_DISTANCE;

    emergency_break_requested = 0;
}


// consignes de deplacements du robot
void motion_block(void) {
    asserv_mode = ASSERV_MODE_BREAK;
}

void motion_off(void) {
    asserv_mode = ASSERV_MODE_OFF;
}

void motion_free(void) {
    asserv_mode = ASSERV_MODE_FREE;
}

void motion_pos(Position pos) {
    if (asserv_mode == ASSERV_MODE_BREAK) {
        pid_vitesse_reset();
        speed_order_constrained.vx = 0.0f;
        speed_order_constrained.vy = 0.0f;
        speed_order_constrained.vt = 0.0f;
    } else {
        // OFF, FREE, SPEED, ABSOLUTE_SPEED, POS : toujours partir de la vitesse physique réelle
        pid_vitesse_reset();
        speed_order_constrained.vx = speed_robot_asserv.vx;
        speed_order_constrained.vy = speed_robot_asserv.vy;
        speed_order_constrained.vt = speed_robot_asserv.vt;
    }
    current_stop_distance = default_stop_distance;
    Wanted_Pos = pos;
    emergency_break_requested = 0;
    asserv_mode = ASSERV_MODE_POS;
}

void motion_speed(Speed speed) {
    if (asserv_mode == ASSERV_MODE_OFF || 
        asserv_mode == ASSERV_MODE_FREE) {
        pid_vitesse_reset();
        speed_order_constrained.vx = speed_robot_asserv.vx;
        speed_order_constrained.vy = speed_robot_asserv.vy;
        speed_order_constrained.vt = speed_robot_asserv.vt;

    } else if (asserv_mode == ASSERV_MODE_BREAK) {
        pid_vitesse_reset();
        speed_order_constrained.vx = 0.0f;
        speed_order_constrained.vy = 0.0f;
        speed_order_constrained.vt = 0.0f;
    }

    emergency_break_requested = 0;  // ← manquait totalement
    Wanted_Speed = speed;
    asserv_mode = ASSERV_MODE_SPEED;
}

void motion_absolute_speed(Speed speed) {
    Wanted_Speed = speed;
    asserv_mode = ASSERV_MODE_ABSOLUTE_SPEED;
}


// effectue un pas d'asservissement
void motion_step(void) {
    // choix en fonction du mode d'asservissement (off, position ou vitesse)
    switch (asserv_mode) {
        // si on est en roue libre
        case ASSERV_MODE_OFF:
            asserv_off_step();
            motion_done = 1;
            break;
        // si on s'arrête mais qu'on ne doit pas freiner (pas de roue libre ni de blocage roues)
        case ASSERV_MODE_FREE:
            asserv_free_step();
            motion_done = 1;
            break;
        // si on est en asservissement en position
        case ASSERV_MODE_POS:
            pos_asserv_step();
            motion_done = 0;
            break;
        // si on est en asservissement en vitesse
        case ASSERV_MODE_SPEED:
            speed_asserv_step();
            motion_done = 0;
            break;
        // si on doit freiner en urgence
        case ASSERV_MODE_BREAK:
            speed_asserv_break_step();
            motion_done = 0;
            break;
        // si on est en asservissement en vitesse absolue
        case ASSERV_MODE_ABSOLUTE_SPEED:
            absolute_speed_asserv_step();
            motion_done = 0;
            break;
		default:
			asserv_off_step();
			break;
    }
}

void asserv_off_step(void) {
	speed_order.vx = 0;
	speed_order.vy = 0;
	speed_order.vt = 0;
    Pid_Speed_En = 0;
    emergency_break_requested = 0;
    motion_off();
}

void asserv_free_step(void)
{   
	speed_order.vx = 0;
	speed_order.vy = 0;
	speed_order.vt = 0;
    Pid_Speed_En = 1;

	if ((fabs(speed_robot_asserv.vx) < DEFAULT_SPEED_LIN_STOP) && 
        (fabs(speed_robot_asserv.vy) < DEFAULT_SPEED_LIN_STOP) && 
        (fabs(speed_robot_asserv.vt) < DEFAULT_SPEED_ROT_STOP)) {
        
            motion_off();
        }
}
	

void speed_asserv_break_step(void) {
// break only if the robot is moving 
    if (fabs(speed_robot_asserv.vx) > DEFAULT_SPEED_LIN_STOP || fabs(speed_robot_asserv.vy) > DEFAULT_SPEED_LIN_STOP || fabs(speed_robot_asserv.vt) > DEFAULT_SPEED_ROT_STOP){
        emergency_break_requested = 1;
        speed_order.vx = 0;
        speed_order.vy = 0;
        speed_order.vt = 0;
        Pid_Speed_En = 1;
    } else {
        emergency_break_requested = 0;
        
        // CORRECTION : Forcer la consigne interne à 0 pour tuer le "fantôme"
        speed_order.vx = 0;
        speed_order.vy = 0;
        speed_order.vt = 0;
        
        motion_free();
        printf("Break,done\n");
        motion_done = 1;
    }
}

float old_angle = 0;

void pos_asserv_step(void) {
    // --- Consignes
    float x_o = Wanted_Pos.x;
    float y_o = Wanted_Pos.y;
    float t_o = Wanted_Pos.t;

    // --- État actuel (Kalman)
    float x = kalman_current_state.x[0];
    float y = kalman_current_state.x[1];
    float t = kalman_current_state.x[2];

    // --- Erreurs
    float rdx = x_o - x;
    float rdy = y_o - y;
    float d   = sqrtf(rdx*rdx + rdy*rdy);
    float dt  = principal_angle(t_o - t);

    float cos_t = cosf(t);
    float sin_t = sinf(t);

    float angle = atan2f(rdy, rdx);

    // -----------------------------------------------------------------------
    // --- Vitesse radiale : profil sqrt clampé par la limite physique de freinage
    // -----------------------------------------------------------------------
    float v_profile = radial_speed_calculation(d);

    // Limite physique dure : vitesse max pour pouvoir s'arrêter sur d avec a_max
    // Si un saut Kalman réduit d brusquement, cette limite protège du dépassement
    // On prend le min des deux : v_cmd ne dépasse jamais ce que la physique permet
    float v_braking_limit = sqrtf(2.0f * robot_a_max * d);
    float v_cmd = fminf(v_profile, fminf(v_braking_limit, v_max));

    // -----------------------------------------------------------------------
    // --- Décomposition directionnelle et transformation repère robot
    // -----------------------------------------------------------------------
    float vx_world = v_cmd * cosf(angle);
    float vy_world = v_cmd * sinf(angle);

    speed_order.vx =  vx_world * cos_t + vy_world * sin_t;
    speed_order.vy = -vx_world * sin_t + vy_world * cos_t;

    // --- Vitesse angulaire (inchangée)
    speed_order.vt = angular_speed_calculation(dt);

    // --- Activation PID vitesse
    Pid_Speed_En = 1;

    // -----------------------------------------------------------------------
    // --- Condition d'arrêt : position + angle + vitesse physique faible
    // -----------------------------------------------------------------------
    float v_now = sqrtf(speed_robot_asserv.vx * speed_robot_asserv.vx +
                        speed_robot_asserv.vy * speed_robot_asserv.vy);

    if ((d         < current_stop_distance)       &&
        (fabsf(dt) < DEFAULT_STOP_ANGLE)          &&
        (v_now     < DEFAULT_SPEED_LIN_STOP * 0.5f)) {
        motion_free();
        printf("Pos,done\n");
    }
}


float radial_speed_calculation(float distance) {
    float v_profile      = sqrtf(2.0f * robot_a_max * distance * 0.85f);
    float v_braking_limit = sqrtf(2.0f * robot_a_max * distance);
    return fminf(v_profile, fminf(v_braking_limit, v_max));
}
float angular_speed_calculation(float angle) {
    float fabs_angle = fabsf(angle);
    int sign = 1;
    if (angle < 0) {
        sign = -1;
    }
    return sign * sqrtf(2.0f * robot_at_max * fabs_angle * 0.95f);
}

void speed_asserv_step(void) {

	speed_order.vx = Wanted_Speed.vx;
	speed_order.vy = Wanted_Speed.vy;
	speed_order.vt = Wanted_Speed.vt;
    Pid_Speed_En = 1;
}

void absolute_speed_asserv_step(void) {
    float cos_t = cos(position_robot_odom.t);
    float sin_t = sin(position_robot_odom.t);
	speed_order.vx =  Wanted_Speed.vx*cos_t + Wanted_Speed.vy*sin_t;
	speed_order.vy = -Wanted_Speed.vx*sin_t + Wanted_Speed.vy*cos_t;
	speed_order.vt = Wanted_Speed.vt;
    Pid_Speed_En = 1;
}


// indique si l'asservissement en cours a termine
int Get_asserv_done(void) {
    if (asserv_mode == ASSERV_MODE_OFF) {
         return 1;
    } else {
        return 0;
    }
}

// verifier qu'on est pas bloque par un obstacle
// si bloque, annule la consigne de vitesse
void asserv_check_blocked(float period) {
    if (   (fabs(speed_robot_asserv.vx - speed_order_constrained.vx) > 0.1) || 
           (fabs(speed_robot_asserv.vy - speed_order_constrained.vy) > 0.1) || 
           (fabs(speed_robot_asserv.vt - speed_order_constrained.vt) > 0.4)    ) {
        if (blocked_time >= ASSERV_BLOCK_TIME_LIMIT) {
			printf("asserv,blocked\n");
            motion_free();
            blocked_time = 0;
        }
        blocked_time += period;
    } else {
        blocked_time = 0;
    }
}

