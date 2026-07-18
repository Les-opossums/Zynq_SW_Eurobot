#include "asserv.h"

#define ODO_PERIOD_MS ODO_EVERY_MS
#define ASSERV_PERIOD_MS (ASSERV_EVERY * ODO_EVERY_MS)
#define ASSERV_TICKS_PER_SLOW (ASSERV_PERIOD_MS / ODO_PERIOD_MS)

typedef struct
{
    Enable_Kalman kalman_enable;
    float lidar_noise[3];
    uint8_t kalman_initialized;
    uint8_t manual_esc_enabled;
    ESC_Command manual_esc;
    uint32_t last_imu_sequence;
} asserv_context_t;

static asserv_context_t ctx;
static uint32_t last_fast_ms;
static uint8_t fast_ticks_since_slow;

static int16_t saturate_motor_command(float command)
{
    if (command > MOTOR_POWER_MAX)
        return MOTOR_POWER_MAX;
    if (command < MOTOR_POWER_MIN)
        return MOTOR_POWER_MIN;
    return (int16_t)command;
}

static void receive_commands(void)
{
    Position position;
    if (IPC_CheckFromOtherCore(&position, sizeof(position), &IPC_DATA->cmd_position,
                               &IPC_DATA->flag_cmd_position_valid, &IPC_DATA->flag_cmd_position_ack))
        motion_set_position(&position);
    Speed speed;
    if (IPC_CheckFromOtherCore(&speed, sizeof(speed), &IPC_DATA->cmd_speed, &IPC_DATA->flag_cmd_speed_valid,
                               &IPC_DATA->flag_cmd_speed_ack))
        motion_set_speed(&speed);
    Enable_Kalman kalman_enable;
    Set_lidar_noise lidar_noise;
    ESC_Command esc;
    int mode;
    float value;
    if (IPC_CheckFromOtherCore(&speed, sizeof(speed), &IPC_DATA->cmd_abs_speed,
                               &IPC_DATA->flag_cmd_abs_speed_valid, &IPC_DATA->flag_cmd_abs_speed_ack))
        motion_set_absolute_speed(&speed);
    if (IPC_CheckFromOtherCore(&mode, sizeof(mode), &IPC_DATA->asserv_mode, &IPC_DATA->flag_asserv_mode_valid,
                               &IPC_DATA->flag_asserv_mode_ack))
    {
        if (mode == 0)
        {
            motion_free();
            ctx.manual_esc_enabled = 0;
        }
        else if (mode == 4)
            motion_block();
    }
    if (IPC_CheckFromOtherCore(&position, sizeof(position), &IPC_DATA->set_pos, &IPC_DATA->flag_set_pos_valid,
                               &IPC_DATA->flag_set_pos_ack))
    {
        odometry_set_position(&position);
        kalman_init_with_lidar(&kalman_fifo, &position);
        ctx.kalman_initialized = 1;
    }
    if (IPC_CheckFromOtherCore(&value, sizeof(value), &IPC_DATA->vmax, &IPC_DATA->flag_vmax_valid,
                               &IPC_DATA->flag_vmax_ack))
        set_Constraint_vitesse_xy_max(value);
    if (IPC_CheckFromOtherCore(&value, sizeof(value), &IPC_DATA->vtmax, &IPC_DATA->flag_vtmax_valid,
                               &IPC_DATA->flag_vtmax_ack))
        set_Constraint_vt_max(value);
    if (IPC_CheckFromOtherCore(&value, sizeof(value), &IPC_DATA->amax, &IPC_DATA->flag_amax_valid,
                               &IPC_DATA->flag_amax_ack))
        set_Constraint_a_xy_max(value);
    if (IPC_CheckFromOtherCore(&value, sizeof(value), &IPC_DATA->odo_spacing,
                               &IPC_DATA->flag_odo_spacing_valid, &IPC_DATA->flag_odo_spacing_ack))
        odometry_set_wheel_distance(value);
    if (IPC_CheckFromOtherCore(&esc, sizeof(esc), &IPC_DATA->cmd_esc, &IPC_DATA->flag_cmd_esc_valid,
                               &IPC_DATA->flag_cmd_esc_ack))
    {
        ctx.manual_esc = esc;
        ctx.manual_esc_enabled = 1;
    }
    if (IPC_CheckFromOtherCore(&kalman_enable, sizeof(kalman_enable), &IPC_DATA->enable_kalman,
                               &IPC_DATA->flag_enable_kalman_valid, &IPC_DATA->flag_enable_kalman_ack))
        ctx.kalman_enable = kalman_enable;
    if (IPC_CheckFromOtherCore(&lidar_noise, sizeof(lidar_noise), &IPC_DATA->kalman_noise_lidar,
                               &IPC_DATA->flag_kalman_noise_lidar_valid,
                               &IPC_DATA->flag_kalman_noise_lidar_ack))
    {
        ctx.lidar_noise[0] = lidar_noise.process_noise_lidar_x * lidar_noise.process_noise_lidar_x;
        ctx.lidar_noise[1] = lidar_noise.process_noise_lidar_y * lidar_noise.process_noise_lidar_y;
        ctx.lidar_noise[2] = lidar_noise.process_noise_lidar_t * lidar_noise.process_noise_lidar_t;
    }
    Set_lidar lidar;
    const uint8_t lidar_received =
        IPC_CheckFromOtherCore(&lidar, sizeof(lidar), &IPC_DATA->set_lidar, &IPC_DATA->flag_set_lidar_valid,
                               &IPC_DATA->flag_set_lidar_ack);
    if (!ctx.kalman_initialized)
    {
        if (lidar_received)
        {
            Position initial = {lidar.lidar_position_x, lidar.lidar_position_y, lidar.lidar_position_t};
            odometry_set_position(&initial);
            kalman_init_with_lidar(&kalman_fifo, &initial);
            ctx.kalman_initialized = 1;
        }
        return;
    }
    int earliest_index = -1, largest_delay = -1;
    if (lidar_received && ctx.kalman_enable.enable_lidar_kalman)
    {
        int index = kalman_fifo_insert_lidar(&kalman_fifo, &lidar, ctx.lidar_noise);
        if (index >= 0)
        {
            earliest_index = index;
            largest_delay = (int)lidar.delay;
        }
    }
    Set_camera camera;
    volatile Set_camera *sources[3] = {&IPC_DATA->set_camera_1, &IPC_DATA->set_camera_2,
                                       &IPC_DATA->set_camera_3};
    volatile uint32_t *valid[3] = {&IPC_DATA->flag_set_camera_1_valid, &IPC_DATA->flag_set_camera_2_valid,
                                   &IPC_DATA->flag_set_camera_3_valid};
    volatile uint32_t *ack[3] = {&IPC_DATA->flag_set_camera_1_ack, &IPC_DATA->flag_set_camera_2_ack,
                                 &IPC_DATA->flag_set_camera_3_ack};
    for (uint8_t i = 0; i < 3; ++i)
        if (IPC_CheckFromOtherCore(&camera, sizeof(camera), sources[i], valid[i], ack[i]) &&
            ctx.kalman_enable.enable_camera_kalman)
        {
            int index = kalman_fifo_insert_camera(&kalman_fifo, &camera, i);
            if (index >= 0 && (int)camera.delay > largest_delay)
            {
                earliest_index = index;
                largest_delay = (int)camera.delay;
            }
        }
    if (earliest_index >= 0)
        kalman_fifo_repropagate_start(&kalman_fifo, earliest_index, ODO_PERIOD_MS * 0.001f, ctx.lidar_noise);
}

static void fast_loop(void)
{
    int motor_rpm[4] = {motor_feedback[0].speed_motor, motor_feedback[1].speed_motor,
                        motor_feedback[2].speed_motor, motor_feedback[3].speed_motor};
    odometry_update_from_motor_rpm(motor_rpm);
    odometry_integrate(ODO_PERIOD_MS * 0.001f);
    kalman_predict(&kalman_current_state, ODO_PERIOD_MS * 0.001f);
    kalman_update_odo(&kalman_current_state, &speed_robot_odom);
    const uint32_t sequence_before = IPC_DATA->imu_seq;
    const float gyro_z = IPC_DATA->imu_gyro_z;
    const uint32_t calibration = IPC_DATA->imu_calib_status;
    const uint8_t imu_available = (sequence_before == IPC_DATA->imu_seq &&
                                   sequence_before != ctx.last_imu_sequence && calibration >= 1U);
    if (imu_available)
    {
        ctx.last_imu_sequence = sequence_before;
        kalman_update_imu(&kalman_current_state, gyro_z);
    }
    kalman_fifo_push(&kalman_fifo, &kalman_current_state, &speed_robot_odom, imu_available, gyro_z);
    if (repropagate_job.active)
        (void)kalman_fifo_repropagate_tick(&kalman_fifo);
}

static void slow_loop(void)
{
    Position current = {kalman_current_state.x[0], kalman_current_state.x[1], kalman_current_state.x[2]};
    odometry_finish_slow(ASSERV_TICKS_PER_SLOW);
    motion_step(&current);
    constrain_speed_order();
    constrain_acceleration_order(ASSERV_PERIOD_MS * 0.001f);
    ESC_Command command;
    Asserv_PWM_calculator(&command);
    if (ctx.manual_esc_enabled)
        command = ctx.manual_esc;
    int16_t motors[4] = {saturate_motor_command(command.command1), saturate_motor_command(command.command2),
                         saturate_motor_command(command.command3), saturate_motor_command(command.command4)};
    if (IPC_DATA->AU_state)
    {
        motors[0] = motors[1] = motors[2] = motors[3] = 0;
        pid_vitesse_reset();
    }
    CAN_transmit_motor(&Can0_Ctx, motors, 4);
    (void)IPC_SendToOtherCore(&current, sizeof(current), &IPC_DATA->kalman_out,
                              &IPC_DATA->flag_kalman_out_valid, &IPC_DATA->flag_kalman_out_ack);
    (void)IPC_SendToOtherCore(&speed_robot_asserv, sizeof(speed_robot_asserv), &IPC_DATA->speed_robot,
                              &IPC_DATA->flag_speed_robot_valid, &IPC_DATA->flag_speed_robot_ack);
    (void)IPC_SendToOtherCore(&speed_order_constrained, sizeof(speed_order_constrained),
                              &IPC_DATA->cmd_speed_constrained, &IPC_DATA->flag_cmd_speed_constrained_valid,
                              &IPC_DATA->flag_cmd_speed_constrained_ack);
}

void asserv_loop_init(void)
{
    ctx = (asserv_context_t){0};
    ctx.kalman_enable.enable_lidar_kalman = 1;
    ctx.lidar_noise[0] = OBS_NOISE_LIDAR_X * OBS_NOISE_LIDAR_X;
    ctx.lidar_noise[1] = OBS_NOISE_LIDAR_Y * OBS_NOISE_LIDAR_Y;
    ctx.lidar_noise[2] = OBS_NOISE_LIDAR_THETA * OBS_NOISE_LIDAR_THETA;
    Init_CAN_MOTOR_variables();
    odometry_init();
    motion_init();
    speed_constrainer_init();
    acceleration_constrainer_init();
    pid_vitesse_init();
    pid_vitesse_reset();
    kalman_fifo_init(&kalman_fifo);
    kalman_init(&kalman_current_state);
    last_fast_ms = (uint32_t)Timer_ms1;
}

void asserv_loop_update(void)
{
    receive_commands();
    const uint32_t now_ms = (uint32_t)Timer_ms1;
    if ((uint32_t)(now_ms - last_fast_ms) < ODO_PERIOD_MS)
        return;
    last_fast_ms = now_ms; /* Never replay stale actuator cycles after an overload. */
    fast_loop();
    if (++fast_ticks_since_slow >= ASSERV_TICKS_PER_SLOW)
    {
        fast_ticks_since_slow = 0;
        slow_loop();
    }
}
