#include <stdio.h>
#include <string.h>

#define main baseline_mission_test_main
#define LX_Change_Mode baseline_LX_Change_Mode
#define FC_Unlock baseline_FC_Unlock
#define OneKey_Land baseline_OneKey_Land
#include "mission_v21_state_sim.c"
#undef OneKey_Land
#undef FC_Unlock
#undef LX_Change_Mode
#undef main

static uint8_t s_allow_mode_command = 1u;
static uint8_t s_allow_unlock_command = 1u;
static uint8_t s_unlock_sets_feedback = 1u;
static uint8_t s_land_command_locks = 1u;
static uint32_t s_land_command_calls;

uint8_t LX_Change_Mode(uint8_t mode)
{
    change_mode_calls++;
    if (s_allow_mode_command == 0u)
    {
        return 0u;
    }
    fc_sta.fc_mode_sta = mode;
    return 1u;
}

uint8_t FC_Unlock(void)
{
    unlock_calls++;
    if (s_allow_unlock_command == 0u)
    {
        return 0u;
    }
    if (s_unlock_sets_feedback != 0u)
    {
        fc_sta.unlock_sta = 1u;
    }
    return 1u;
}

uint8_t OneKey_Land(void)
{
    s_land_command_calls++;
    if (s_land_command_locks != 0u)
    {
        fc_sta.unlock_sta = 0u;
    }
    return 1u;
}

static void setup_healthy_inputs(void)
{
    s_yaw_arrived = 1u;
    rc_in.rc_ch.st_data.ch_[5] = 1900;
    hight = 150;
    UserTask_OneKeyCmd();
    fake_time_ms += 100u; feed_car_pose();
    fake_time_ms += 100u; feed_car_pose();
    fake_time_ms += 100u; feed_health(1u);
}

static void setup_healthy_inputs_without_localization_freshness(void)
{
    rc_in.rc_ch.st_data.ch_[5] = 1900;
    hight = 150;
    UserTask_OneKeyCmd();
    fake_time_ms += 100u; feed_car_pose();
    fake_time_ms += 100u; feed_car_pose();
    fake_time_ms += 100u;
    feed_car_pose();
    feed_camera();
}

static void start_task(uint8_t task_type, uint16_t mission_id)
{
    fake_time_ms += 20u;
    feed_health(1u);
    send_request(task_type, mission_id, 1u);
    UserTask_OneKeyCmd();
    expect(mission_step == MISSION_STAGE_IDLE,
           "request entered PRECHECK before CAMERA_MODE ACK");
    feed_last_camera_ack(CAMERA_TYPE_MODE);
    UserTask_OneKeyCmd();
    expect(mission_step == MISSION_STAGE_PRECHECK, "request did not enter PRECHECK");
    expect(count_start_warnings() == 1u, "request did not send exactly one warning");
}

static void start_task_without_localization_freshness(uint8_t task_type, uint16_t mission_id)
{
    fake_time_ms += 20u;
    feed_car_pose();
    feed_camera();
    send_request(task_type, mission_id, 1u);
    UserTask_OneKeyCmd();
    expect(mission_step == MISSION_STAGE_IDLE,
           "sensor-freshness-free request entered PRECHECK before CAMERA_MODE ACK");
    feed_last_camera_ack(CAMERA_TYPE_MODE);
    UserTask_OneKeyCmd();
    expect(mission_step == MISSION_STAGE_PRECHECK,
           "sensor-freshness-free request was not accepted into PRECHECK");
}

static void run_to_stage(uint8_t stage, uint32_t max_ticks)
{
    uint32_t ticks;
    for (ticks = 0u; ticks < max_ticks && mission_step != stage; ticks++)
    {
        tick_perfect_tracking(1u);
    }
    if (mission_step != stage)
    {
        fprintf(stderr, "wanted stage %u, got %u after %lu ticks\n",
                (unsigned)stage, (unsigned)mission_step, (unsigned long)max_ticks);
        fail("safety regression stage timeout");
    }
}

static uint8_t make_camera_action_result(uint16_t action_id, uint8_t *frame)
{
    uint8_t payload[12] = {0};

    payload[0] = CAMERA_ACTION_DROP;
    payload[1] = 0u;
    put_be16(&payload[2], action_id);
    put_be32(&payload[4], camera_source_time++);
    payload[8] = camera_session_task_type;
    put_be16(&payload[9], camera_session_mission_id);
    payload[11] = camera_session_mode_seq;
    return make_camera_frame(CAMERA_TYPE_ACTION_RESULT, 2u,
                             payload, sizeof(payload), frame);
}

static void feed_action_success(uint16_t action_id)
{
    uint8_t frame[24];
    uint8_t len;

    feed_last_camera_ack(CAMERA_TYPE_ACTION);
    UserTask_OneKeyCmd();
    len = make_camera_action_result(action_id, frame);
    feed_bytes(MY_uart_maixcam_receive, frame, len);
}

static void tick_frozen_height(void)
{
    fake_time_ms += 20u;
    dis_x_slam = dis_x_target;
    dis_y_slam = dis_y_target;
    hight = 20;
    yaw_slam = (int32_t)yaw_target * 10;
    if ((fake_time_ms % 100u) == 0u) feed_health(1u);
    else MY_uart_mark_laser_update(hight, 1u);
    UserTask_OneKeyCmd();
}

static void tick_perfect_tracking_without_localization_freshness(void)
{
    fake_time_ms += 20u;
    dis_x_slam = dis_x_target;
    dis_y_slam = dis_y_target;
    if (height_target > 0) hight = height_target;
    yaw_slam = (int32_t)yaw_target * 10;
    if ((fake_time_ms % 100u) == 0u)
    {
        feed_car_pose();
        feed_camera();
    }
    UserTask_OneKeyCmd();
}

static void feed_car_pose_xy(int32_t x_cm, int32_t y_cm)
{
    uint8_t payload[22] = {0};
    uint8_t frame[40];
    uint8_t len;

    payload[0] = CAR_FRAME_FIELD_GLOBAL;
    payload[1] = CAR_POSE_POSITION_VALID | CAR_POSE_CALIBRATED | CAR_POSE_YAW_VALID;
    put_be16(&payload[2], 1u);
    put_be32(&payload[4], (uint32_t)x_cm);
    put_be32(&payload[8], (uint32_t)y_cm);
    put_be16(&payload[12], 900u);
    put_be16(&payload[14], 0x7FFFu);
    put_be16(&payload[16], 0x7FFFu);
    put_be32(&payload[18], car_source_time++);
    len = make_lx(LX_EXT_CAR_POSE, payload, sizeof(payload), frame);
    feed_bytes(MY_uart_esp_receive, frame, len);
}

static void feed_maintenance_reset(uint16_t reset_id)
{
    uint8_t payload[8] = {0};
    uint8_t frame[24];
    uint8_t len;

    put_be16(&payload[0], reset_id);
    payload[2] = MAINTENANCE_RESET_CLEAR_CALIBRATION;
    put_be32(&payload[4], fake_time_ms);
    len = make_lx(LX_EXT_MAINTENANCE_RESET, payload, sizeof(payload), frame);
    feed_bytes(MY_uart_esp_receive, frame, len);
}

static void feed_mission_abort(void)
{
    uint8_t frame[16];
    uint8_t len = make_lx(LX_EXT_MISSION_ABORT, 0, 0u, frame);
    feed_bytes(MY_uart_esp_receive, frame, len);
}

static void feed_health_xy(int32_t x_cm, int32_t y_cm)
{
    feed_mid360();
    MY_uart_mark_laser_update(hight, 1u);
    feed_car_pose_xy(x_cm, y_cm);
    feed_camera();
}

static void tick_perfect_tracking_xy(int32_t x_cm, int32_t y_cm)
{
    fake_time_ms += 20u;
    dis_x_slam = dis_x_target;
    dis_y_slam = dis_y_target;
    if (height_target > 0) hight = height_target;
    yaw_slam = (int32_t)yaw_target * 10;
    if ((fake_time_ms % 100u) == 0u) feed_health_xy(x_cm, y_cm);
    else MY_uart_mark_laser_update(hight, 1u);
    UserTask_OneKeyCmd();
}

static int scenario_drop_complete(void)
{
    setup_healthy_inputs();
    start_task(LX_TASK_DROP, 10u);
    camera_target_is_valid = 0u;
    run_to_stage(MISSION_STAGE_DROP_ALIGN, 1600u);
    tick_perfect_tracking(1u);
    expect(height_target == 65,
           "DROP_ALIGN did not write the 65 cm target directly to the height loop");
    expect(dis_x_target == 120 && dis_y_target == -65,
           "DROP_ALIGN did not apply the body +X release compensation");
    run_to_stage(MISSION_STAGE_DROP_ACTION, 1600u);
    tick_perfect_tracking(1u);
    feed_action_success(10u);
    UserTask_OneKeyCmd();
    run_to_stage(MISSION_STAGE_IDLE, 1000u);
    puts("[PASS] drop_complete");
    return 0;
}

static int scenario_drop_release_does_not_wait_for_yaw(void)
{
    setup_healthy_inputs();
    start_task(LX_TASK_DROP, 11u);
    run_to_stage(MISSION_STAGE_DROP_ALIGN, 1600u);

    /* Yaw tracking remains commanded, but must not block the vertical release. */
    s_yaw_arrived = 0u;
    run_to_stage(MISSION_STAGE_DROP_ACTION, 1600u);
    expect(mission_step == MISSION_STAGE_DROP_ACTION,
           "DROP_ACTION remained blocked by yaw after drop alignment");
    puts("[PASS] drop_release_does_not_wait_for_yaw");
    return 0;
}

static int scenario_dynamic_complete(void)
{
    setup_healthy_inputs();
    start_task(LX_TASK_DYNAMIC_LAND, 20u);
    run_to_stage(MISSION_STAGE_IDLE, 3500u);
    expect(unlock_calls == 1u, "dynamic task attempted a second unlock after platform hold");
    expect(s_land_command_calls == 1u,
           "dynamic task sent a platform landing command instead of only final landing");
    puts("[PASS] dynamic_complete");
    return 0;
}

static int scenario_platform_remains_armed(void)
{
    setup_healthy_inputs();
    start_task(LX_TASK_DYNAMIC_LAND, 21u);
    run_to_stage(MISSION_STAGE_ON_PLATFORM_5S, 2500u);
    expect(height_target == 14,
           "platform contact did not apply the configured 14 cm hold target");
    expect(fc_sta.unlock_sta != 0u,
           "platform contact unexpectedly locked the flight controller");
    expect(s_land_command_calls == 0u,
           "platform contact sent OneKey_Land instead of holding armed");

    run_to_stage(MISSION_STAGE_PLATFORM_TAKEOFF, 400u);
    tick_perfect_tracking(1u);
    expect(unlock_calls == 1u,
           "platform takeoff issued a redundant second unlock command");
    expect(fc_sta.unlock_sta != 0u && height_target == 135,
           "platform takeoff did not resume armed climb to platform cruise height");
    puts("[PASS] platform_remains_armed");
    return 0;
}

static int scenario_mode_queue_timeout(void)
{
    uint32_t ticks;
    s_allow_mode_command = 0u;
    setup_healthy_inputs();
    start_task(LX_TASK_DROP, 30u);
    for (ticks = 0u; ticks < 750u && mission_step != MISSION_STAGE_IDLE; ticks++)
    {
        tick_perfect_tracking(1u);
    }
    expect(mission_step == MISSION_STAGE_IDLE, "mode queue failure did not abort");
    expect(unlock_calls == 0u, "mode queue failure reached unlock");
    puts("[PASS] mode_queue_timeout");
    return 0;
}

static int scenario_unlock_queue_timeout(void)
{
    uint32_t ticks;
    s_allow_unlock_command = 0u;
    setup_healthy_inputs();
    start_task(LX_TASK_DROP, 40u);
    for (ticks = 0u; ticks < 750u && mission_step != MISSION_STAGE_IDLE; ticks++)
    {
        tick_perfect_tracking(1u);
    }
    expect(mission_step == MISSION_STAGE_IDLE, "unlock queue failure did not abort");
    expect(fc_sta.unlock_sta == 0u, "unlock queue failure left FC armed");
    puts("[PASS] unlock_queue_timeout");
    return 0;
}

static int scenario_takeoff_timeout(void)
{
    uint32_t ticks;
    setup_healthy_inputs();
    start_task(LX_TASK_DROP, 50u);
    run_to_stage(MISSION_STAGE_TAKEOFF, 700u);
    for (ticks = 0u; ticks < 1600u && mission_step != MISSION_STAGE_IDLE; ticks++)
    {
        tick_frozen_height();
    }
    expect(mission_step == MISSION_STAGE_IDLE, "stalled takeoff did not hard-abort");
    expect(fc_sta.unlock_sta == 0u, "stalled takeoff did not finish locked");
    puts("[PASS] takeoff_timeout");
    return 0;
}

static int scenario_return_near_car(void)
{
    uint32_t ticks;

    rc_in.rc_ch.st_data.ch_[5] = 1900;
    hight = 150;
    UserTask_OneKeyCmd();
    fake_time_ms += 100u; feed_car_pose_xy(20, 20);
    fake_time_ms += 100u; feed_car_pose_xy(20, 20);
    fake_time_ms += 100u; feed_health_xy(20, 20);
    fake_time_ms += 20u; feed_health_xy(20, 20);
    send_request(LX_TASK_DROP, 60u, 1u);
    UserTask_OneKeyCmd();
    feed_last_camera_ack(CAMERA_TYPE_MODE);
    UserTask_OneKeyCmd();
    for (ticks = 0u; ticks < 1800u && mission_step != MISSION_STAGE_DROP_ACTION; ticks++)
    {
        tick_perfect_tracking_xy(20, 20);
    }
    expect(mission_step == MISSION_STAGE_DROP_ACTION, "near-car task missed DROP_ACTION");
    tick_perfect_tracking_xy(20, 20);
    feed_action_success(60u);
    UserTask_OneKeyCmd();
    for (ticks = 0u; ticks < 1000u && mission_step != MISSION_STAGE_IDLE; ticks++)
    {
        tick_perfect_tracking_xy(20, 20);
    }
    expect(mission_step == MISSION_STAGE_IDLE, "near-car return remained stuck");
    puts("[PASS] return_near_car");
    return 0;
}

static int scenario_home_land_requires_lock(void)
{
    uint32_t ticks;
    setup_healthy_inputs();
    start_task(LX_TASK_DROP, 70u);
    run_to_stage(MISSION_STAGE_DROP_ACTION, 1600u);
    tick_perfect_tracking(1u);
    feed_action_success(70u);
    UserTask_OneKeyCmd();
    run_to_stage(MISSION_STAGE_HOME_LAND, 1000u);
    s_land_command_locks = 0u;
    for (ticks = 0u; ticks < 1100u; ticks++) tick_perfect_tracking(1u);
    expect(mission_step == MISSION_STAGE_HOME_LAND,
           "HOME_LAND published IDLE without lock feedback");
    expect(fc_sta.unlock_sta != 0u, "test did not preserve armed feedback");
    expect(s_land_command_calls >= 2u, "HOME_LAND did not retry landing command");
    fc_sta.unlock_sta = 0u;
    tick_perfect_tracking(1u);
    expect(mission_step == MISSION_STAGE_IDLE, "HOME_LAND did not finish after lock feedback");
    puts("[PASS] home_land_requires_lock");
    return 0;
}

static int scenario_abort_requires_lock(void)
{
    uint32_t ticks;
    setup_healthy_inputs();
    start_task(LX_TASK_DROP, 80u);
    run_to_stage(MISSION_STAGE_TAKEOFF, 700u);
    s_land_command_locks = 0u;
    rc_in.rc_ch.st_data.ch_[5] = 1500;
    tick_perfect_tracking(1u);
    expect(mission_step == MISSION_STAGE_ABORT, "CH6 stop did not enter ABORT");
    for (ticks = 0u; ticks < 1100u; ticks++) tick_perfect_tracking(1u);
    expect(mission_step == MISSION_STAGE_ABORT, "ABORT published IDLE without lock feedback");
    expect(s_land_command_calls >= 2u, "ABORT did not retry landing command");
    fc_sta.unlock_sta = 0u;
    tick_perfect_tracking(1u);
    expect(mission_step == MISSION_STAGE_IDLE, "ABORT did not finish after lock feedback");
    puts("[PASS] abort_requires_lock");
    return 0;
}

static int scenario_camera_mode_ack_timeout(void)
{
    uint32_t ticks;
    uint8_t mode_seq;
    uint8_t idle_seq;

    setup_healthy_inputs();
    send_request(LX_TASK_DROP, 90u, 1u);
    UserTask_OneKeyCmd();
    expect(camera_track_mode_tx_count == 1u, "initial CAMERA_MODE was not sent once");
    mode_seq = uart3_last[6];

    for (ticks = 0u; ticks < 30u; ticks++) tick_perfect_tracking(0u);
    expect(camera_track_mode_tx_count == 3u,
           "CAMERA_MODE timeout did not stop after three total sends");
    expect(uart3_last[3] == CAMERA_TYPE_MODE && uart3_last[9] == CAMERA_MODE_IDLE,
           "MODE ACK exhaustion did not command IDLE");
    expect(find_response(90u, LX_RESULT_STATE_DENIED, LX_DETAIL_CAMERA_MODE),
           "MODE ACK exhaustion did not reject the car request");

    /* The third tracking send immediately precedes IDLE; its Seq is retained by allocation order. */
    expect((uint8_t)(uart3_last[6] - mode_seq) == 1u,
           "tracking retransmissions consumed new Seq values");
    idle_seq = uart3_last[6];
    for (ticks = 0u; ticks < 30u; ticks++) tick_perfect_tracking(0u);
    expect(camera_idle_mode_tx_count == 3u,
           "IDLE CAMERA_MODE did not use the same three-send reliability rule");
    expect(uart3_last[6] == idle_seq &&
               uart3_last[7] == (V21_FLAG_ACK_REQUIRED | V21_FLAG_RETRANSMISSION),
           "IDLE retransmission changed Seq or omitted RETRANSMISSION");
    puts("[PASS] camera_mode_ack_timeout");
    return 0;
}

static int scenario_camera_action_ack_timeout(void)
{
    uint32_t ticks;
    uint8_t action_seq;

    setup_healthy_inputs();
    start_task(LX_TASK_DROP, 91u);
    run_to_stage(MISSION_STAGE_DROP_ACTION, 1600u);
    tick_perfect_tracking(1u);
    expect(camera_action_tx_count == 1u, "initial CAMERA_ACTION was not sent once");
    action_seq = uart3_last[6];
    for (ticks = 0u; ticks < 30u && mission_step != MISSION_STAGE_ABORT; ticks++)
    {
        tick_perfect_tracking(1u);
    }
    expect(mission_step == MISSION_STAGE_ABORT,
           "ACTION ACK exhaustion did not abort the mission");
    expect(camera_action_tx_count == 3u,
           "CAMERA_ACTION timeout did not stop after three total sends");
    /* request_abort sends IDLE after the last ACTION, so allocation delta proves one ACTION Seq. */
    expect(uart3_last[3] == CAMERA_TYPE_MODE &&
               (uint8_t)(uart3_last[6] - action_seq) == 1u,
           "ACTION retransmissions consumed new Seq values");
    puts("[PASS] camera_action_ack_timeout");
    return 0;
}

static int scenario_legacy_start_cannot_select_task(void)
{
    static const uint8_t legacy_start[8] = {
        0xBBu, 0x10u, 0xF1u, 0x10u, 0x01u, 0xEEu, 0xBBu, 0x96u};

    setup_healthy_inputs();
    feed_bytes(MY_uart_esp_receive, legacy_start, sizeof(legacy_start));
    UserTask_OneKeyCmd();
    expect(mission_step == MISSION_STAGE_IDLE,
           "legacy START selected the DROP task under V2.2");
    expect(camera_track_mode_tx_count == 0u,
           "legacy START created a camera task session");
    puts("[PASS] legacy_start_cannot_select_task");
    return 0;
}

static int scenario_maintenance_reset_aborts(void)
{
    setup_healthy_inputs();
    start_task(LX_TASK_DROP, 100u);
    run_to_stage(MISSION_STAGE_TAKEOFF, 700u);

    feed_maintenance_reset(1u);
    UserTask_OneKeyCmd();
    expect(mission_step == MISSION_STAGE_ABORT,
           "MAINTENANCE_RESET did not abort the active mission");
    expect(uart3_last_len >= 11u && uart3_last[3] == CAMERA_TYPE_MODE &&
               uart3_last[9] == CAMERA_MODE_IDLE,
           "MAINTENANCE_RESET did not invalidate the active camera session");

    send_request(LX_TASK_DROP, 100u, 1u);
    UserTask_OneKeyCmd();
    expect(find_response(100u, LX_RESULT_BUSY, LX_DETAIL_MISSION_RUNNING),
           "MAINTENANCE_RESET left an old ACCEPTED response in the dedup cache");
    puts("[PASS] maintenance_reset_aborts");
    return 0;
}

static int scenario_mission_abort_closes_loop(void)
{
    setup_healthy_inputs();
    start_task(LX_TASK_DROP, 110u);
    run_to_stage(MISSION_STAGE_TAKEOFF, 700u);

    feed_mission_abort();
    UserTask_OneKeyCmd();
    expect(mission_step == MISSION_STAGE_ABORT,
           "MISSION_ABORT did not enter the flight-controller ABORT state");
    expect(count_legacy_command(ESP_EVENT_STAGE_ABORT) == 1u &&
               count_legacy_command(ESP_EVENT_ABORT_REMOTE_COMMAND) == 1u,
           "MISSION_ABORT did not edge-report its ABORT stage/reason");
    expect(find_response_type(0x84u, 0u, LX_RESULT_ACCEPTED, LX_DETAIL_NONE),
           "MISSION_ABORT did not return an accepted MISSION_RESPONSE");
    expect(uart3_last_len >= 11u && uart3_last[3] == CAMERA_TYPE_MODE &&
               uart3_last[9] == CAMERA_MODE_IDLE,
           "MISSION_ABORT did not invalidate the active camera session");
    puts("[PASS] mission_abort_closes_loop");
    return 0;
}

static int scenario_localization_liveness_is_not_a_task_gate(void)
{
    uint32_t ticks;

    setup_healthy_inputs_without_localization_freshness();
    expect(my_slam_flag == 0u && MY_uart_laser_is_valid() == 0u,
           "localization-freshness-free setup unexpectedly received a sensor update");
    start_task_without_localization_freshness(LX_TASK_DROP, 120u);
    expect(find_response(120u, LX_RESULT_ACCEPTED, LX_DETAIL_NONE),
           "localization liveness still rejected an otherwise healthy mission request");
    expect(count_legacy_command(ESP_DIAG_MID360) == 0u &&
               count_legacy_command(ESP_DIAG_LASER_HEIGHT) == 0u,
           "a removed localization liveness diagnostic was emitted");

    for (ticks = 0u; ticks < 700u && mission_step != MISSION_STAGE_TAKEOFF; ticks++)
    {
        tick_perfect_tracking_without_localization_freshness();
    }
    expect(mission_step == MISSION_STAGE_TAKEOFF,
           "localization liveness loss still aborted PRECHECK before takeoff");
    puts("[PASS] localization_liveness_is_not_a_task_gate");
    return 0;
}

static int scenario_precheck_tolerates_single_car_pose_gap(void)
{
    setup_healthy_inputs();

    fake_time_ms += 20u;
    feed_health(1u);
    send_request(LX_TASK_DROP, 130u, 1u);
    UserTask_OneKeyCmd();
    expect(mission_step == MISSION_STAGE_IDLE,
           "request entered PRECHECK before CAMERA_MODE ACK");

    /* The task request was admitted while the pose was <250 ms old.  Model
     * one delayed 10 Hz CAR_POSE period while waiting for CAMERA_MODE ACK. */
    fake_time_ms += 268u;
    feed_last_camera_ack(CAMERA_TYPE_MODE);
    UserTask_OneKeyCmd();
    expect(mission_step == MISSION_STAGE_PRECHECK,
           "a 268 ms post-request CAR_POSE gap aborted PRECHECK");

    fake_time_ms += 260u;
    UserTask_OneKeyCmd();
    expect(mission_step == MISSION_STAGE_ABORT,
           "PRECHECK did not abort after CAR_POSE exceeded 500 ms");
    puts("[PASS] precheck_tolerates_single_car_pose_gap");
    return 0;
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        fprintf(stderr, "scenario required\n");
        return 2;
    }
    if (strcmp(argv[1], "drop_complete") == 0) return scenario_drop_complete();
    if (strcmp(argv[1], "drop_release_does_not_wait_for_yaw") == 0)
        return scenario_drop_release_does_not_wait_for_yaw();
    if (strcmp(argv[1], "dynamic_complete") == 0) return scenario_dynamic_complete();
    if (strcmp(argv[1], "platform_remains_armed") == 0)
        return scenario_platform_remains_armed();
    if (strcmp(argv[1], "mode_queue_timeout") == 0) return scenario_mode_queue_timeout();
    if (strcmp(argv[1], "unlock_queue_timeout") == 0) return scenario_unlock_queue_timeout();
    if (strcmp(argv[1], "takeoff_timeout") == 0) return scenario_takeoff_timeout();
    if (strcmp(argv[1], "return_near_car") == 0) return scenario_return_near_car();
    if (strcmp(argv[1], "home_land_requires_lock") == 0) return scenario_home_land_requires_lock();
    if (strcmp(argv[1], "abort_requires_lock") == 0) return scenario_abort_requires_lock();
    if (strcmp(argv[1], "camera_mode_ack_timeout") == 0) return scenario_camera_mode_ack_timeout();
    if (strcmp(argv[1], "camera_action_ack_timeout") == 0) return scenario_camera_action_ack_timeout();
    if (strcmp(argv[1], "legacy_start_cannot_select_task") == 0) return scenario_legacy_start_cannot_select_task();
    if (strcmp(argv[1], "maintenance_reset_aborts") == 0) return scenario_maintenance_reset_aborts();
    if (strcmp(argv[1], "mission_abort_closes_loop") == 0) return scenario_mission_abort_closes_loop();
    if (strcmp(argv[1], "localization_liveness_is_not_a_task_gate") == 0) return scenario_localization_liveness_is_not_a_task_gate();
    if (strcmp(argv[1], "precheck_tolerates_single_car_pose_gap") == 0) return scenario_precheck_tolerates_single_car_pose_gap();
    fprintf(stderr, "unknown scenario: %s\n", argv[1]);
    return 2;
}
