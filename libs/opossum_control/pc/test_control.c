/**
 * @file test_control.c
 * @brief Test en boucle fermee d'opossum_control sur PC.
 *
 * Le robot est simule simplement : chaque roue suit sa commande ESC avec un
 * premier ordre, la cinematique est celle de l'odometrie (base holonome 4
 * roues). L'asservissement tourne exactement comme sur le Zynq : pas rapide
 * a 1 ms, pas lent a 10 ms, commandes moteur maintenues entre deux pas lents.
 */

#include "opossum_control.h"
#include "control_hal_pc.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define PI_F 3.14159265f

/* --- Modele simple du robot ------------------------------------------- */
#define WHEEL_GAIN   2.0e-4f  /* vitesse roue en regime etabli (m/s) par unite de commande ESC */
#define WHEEL_TAU_S  0.020f   /* constante de temps moteur + roue */
#define RPM_TO_RAD   0.10471975512f

typedef struct {
    float wheel[4];   /* vitesse lineaire des roues (m/s) */
    float x, y, t;    /* pose vraie (repere table) */
    float vx, vy, vt; /* vitesse vraie (repere robot) */
} plant_t;

static void plant_step(plant_t *p, const int16_t motors[4], float dt)
{
    const float sign[4] = {ODO_MOTOR_1_SIGN, ODO_MOTOR_2_SIGN, ODO_MOTOR_3_SIGN, ODO_MOTOR_4_SIGN};
    for (int i = 0; i < 4; i++) {
        const float target = WHEEL_GAIN * (float)motors[i] * sign[i];
        p->wheel[i] += (target - p->wheel[i]) * (dt / WHEEL_TAU_S);
    }
    const float w1 = p->wheel[0], w2 = p->wheel[1], w3 = p->wheel[2], w4 = p->wheel[3];
    const float s = 0.70710678f;
    p->vx = s * (-w1 + w2 + w3 - w4) * 0.5f;
    p->vy = s * (w1 + w2 - w3 - w4) * 0.5f;
    p->vt = -(w1 + w2 + w3 + w4) / (4.0f * (float)DEFAULT_ODO_SPACING);

    const float a = p->t + 0.5f * p->vt * dt;
    p->x += (p->vx * cosf(a) - p->vy * sinf(a)) * dt;
    p->y += (p->vx * sinf(a) + p->vy * cosf(a)) * dt;
    p->t = principal_angle(p->t + p->vt * dt);
}

static void plant_sensors(const plant_t *p, control_inputs_t *in, uint32_t tick)
{
    const float sign[4] = {ODO_MOTOR_1_SIGN, ODO_MOTOR_2_SIGN, ODO_MOTOR_3_SIGN, ODO_MOTOR_4_SIGN};
    memset(in, 0, sizeof(*in));
    for (int i = 0; i < 4; i++) {
        /* rpm moteur = vitesse roue / rayon * reduction, en entier comme l'ESC */
        const float rpm = sign[i] * p->wheel[i] / (float)DEFAULT_WHEEL_RADIUS * ODO_MOTOR_REDUCTION / RPM_TO_RAD;
        in->motor_rpm[i] = (int)rpm;
    }
    in->imu_seq          = tick / 10u;    /* gyro a 100 Hz */
    in->imu_seq_stable   = 1u;
    in->imu_gyro_z       = p->vt;
    in->imu_calib_status = 1u;
    in->imu_yaw_seq      = 0u;            /* pas de cap absolu */
}

/* --- Banc de test ----------------------------------------------------- */
typedef struct {
    plant_t           plant;
    control_outputs_t out;
    uint32_t          tick;   /* ms */
    uint32_t          au;
} bench_t;

static void bench_init(bench_t *b, float x, float y, float t)
{
    memset(b, 0, sizeof(*b));
    ctrl_hal_pc_set_log(NULL);
    control_init(0u);
    b->plant.x = x; b->plant.y = y; b->plant.t = t;

    control_commands_t cmd = {0};
    cmd.has_set_pos = 1u;
    cmd.set_pos = (Position){x, y, t};
    control_apply_commands(&cmd, 0.0f, 0u);
}

static void bench_run_ms(bench_t *b, uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        control_inputs_t in;
        plant_sensors(&b->plant, &in, b->tick);
        in.au_state = b->au;
        if (control_fast_step(&in)) {
            control_slow_step(b->au, &b->out);
        }
        plant_step(&b->plant, b->out.motors, 0.001f);
        b->tick++;
    }
}

static void bench_move(bench_t *b, float x, float y, float t)
{
    control_commands_t cmd = {0};
    (void)b; /* la commande part directement dans opossum_control */
    cmd.has_cmd_position = 1u;
    cmd.cmd_position = (Position){x, y, t};
    control_apply_commands(&cmd, 0.0f, 0u);
}

/* Attend motion_done != MOTION_MOVING ; renvoie la duree en ms ou 0 si timeout. */
static uint32_t bench_wait_done(bench_t *b, uint32_t timeout_ms)
{
    for (uint32_t ms = 0; ms < timeout_ms; ms += 10u) {
        bench_run_ms(b, 10u);
        if (ctrl_hal_pc_motion_done() != MOTION_MOVING) {
            return ms + 10u;
        }
    }
    return 0u;
}

static int failures;
#define CHECK(cond, ...) do { \
    if (!(cond)) { failures++; printf("  ECHEC : "); printf(__VA_ARGS__); printf("\n"); } \
    else         { printf("  ok     : "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void test_move(float x0, float y0, float t0, float x, float y, float t)
{
    bench_t b;
    printf("MOVE (%.2f, %.2f, %.2f) -> (%.2f, %.2f, %.2f)\n", x0, y0, t0, x, y, t);
    bench_init(&b, x0, y0, t0);
    bench_move(&b, x, y, t);
    CHECK(ctrl_hal_pc_motion_done() == MOTION_MOVING, "motion_done = MOVING des la commande");
    const uint32_t ms = bench_wait_done(&b, 8000u);
    const float err = hypotf(b.plant.x - x, b.plant.y - y);
    const float err_t = fabsf(principal_angle(b.plant.t - t));
    CHECK(ms > 0u && ctrl_hal_pc_motion_done() == MOTION_SUCCESS, "arrivee en %u ms", ms);
    CHECK(err < 0.02f, "erreur de position vraie %.1f mm", err * 1000.0f);
    CHECK(err_t < 0.035f, "erreur de cap vraie %.2f deg", err_t * 180.0f / PI_F);
    /* Pas de verification d'immobilite apres l'arrivee : sans frottement dans
     * ce modele, l'integrale du PID accumulee pendant le trajet fait glisser
     * le robot en mode FREE (cf README, "Comportements observes en SIL"). */
    bench_run_ms(&b, 500u);
    printf("  info   : 500 ms apres l'arrivee : %.3f m/s, %.3f rad/s, asserv_mode %d\n",
           hypotf(b.plant.vx, b.plant.vy), b.plant.vt, (int)asserv_mode);
}

static void test_emergency_stop(void)
{
    bench_t b;
    printf("Arret d'urgence pendant un mouvement\n");
    bench_init(&b, 0.5f, 0.5f, 0.0f);
    bench_move(&b, 2.0f, 0.5f, 0.0f);
    bench_run_ms(&b, 600u);
    CHECK(b.out.motors[0] != 0 || b.out.motors[1] != 0, "moteurs commandes avant l'AU");
    b.au = 1u;
    bench_run_ms(&b, 20u);
    CHECK(b.out.motors[0] == 0 && b.out.motors[1] == 0 && b.out.motors[2] == 0 && b.out.motors[3] == 0,
          "commandes moteur nulles sous AU");
}

static void test_block(void)
{
    bench_t b;
    printf("BLOCK pendant un mouvement\n");
    bench_init(&b, 0.5f, 0.5f, 0.0f);
    bench_move(&b, 2.0f, 0.5f, 0.0f);
    bench_run_ms(&b, 600u);
    CHECK(hypotf(b.plant.vx, b.plant.vy) > 0.2f, "en mouvement (%.2f m/s)", hypotf(b.plant.vx, b.plant.vy));
    control_commands_t cmd = {0};
    cmd.has_asserv_mode = 1u;
    cmd.asserv_mode = 4;
    control_apply_commands(&cmd, 0.0f, 0u);
    CHECK(ctrl_hal_pc_motion_done() == MOTION_BRAKED, "motion_done = BRAKED");
    bench_run_ms(&b, 1500u);
    CHECK(hypotf(b.plant.vx, b.plant.vy) < 0.02f, "arrete apres 1.5 s (%.3f m/s)", hypotf(b.plant.vx, b.plant.vy));
}

int main(void)
{
    test_move(0.5f, 0.5f, 0.0f, 1.2f, 0.9f, PI_F / 2.0f);
    test_move(2.5f, 1.5f, PI_F, 1.0f, 0.4f, -PI_F / 4.0f);
    test_move(1.0f, 1.0f, 0.0f, 1.0f, 1.0f, PI_F);   /* rotation pure */
    test_emergency_stop();
    test_block();
    printf(failures ? "\n%d ECHEC(S)\n" : "\nTous les tests passent\n", failures);
    return failures ? 1 : 0;
}
