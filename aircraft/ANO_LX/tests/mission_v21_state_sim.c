#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "User_Task.h"
#include "my_contrl.h"
#include "my_send_test.h"
#include "my_uart.h"

typedef struct { int16_t ch_[10]; } test_rc_channels_t;
typedef union { test_rc_channels_t st_data; } test_rc_channel_union_t;
typedef struct
{
    uint8_t no_signal;
    uint8_t fail_safe;
    test_rc_channel_union_t rc_ch;
} test_rc_input_t;
typedef struct
{
    uint8_t fc_mode_sta;
    uint8_t unlock_sta;
} test_fc_state_t;

test_rc_input_t rc_in;
test_fc_state_t fc_sta;
double yaw;
int32_t hight;
int32_t dis_x_slam, dis_y_slam, yaw_slam;

int my_mode;
int my_give_vel_x, my_give_vel_y, my_give_vel_z, my_give_vel_yaw;
int my_give_vel_rol, my_give_vel_pit;
unsigned int mission_step;
int my_task_time_dly_cnt_ms, time_cnt_ms;
int32_t height_target, dis_x_target, dis_y_target, yaw_target;
int32_t hight_now, dis_x_now, dis_y_now, yaw_now, yaw_base;
double yaw_zero, yaw_fix;
int32_t keep_hight_flag, keep_dis_flag, keep_yaw_flag, keep_cam_flag, keep_radar_flag;
int32_t dis_x_zero, dis_y_zero;
int dis_target_num;
uint8_t yaw_mode;
int dis_x_cam_target, dis_y_cam_target;
uint8_t cam_target_control_flag, cam_target_code_identity_flag;

static uint32_t fake_time_ms = 1000u;
static uint8_t uart1_history[128][80];
static uint8_t uart1_history_len[128];
static uint8_t uart1_history_count;
static uint8_t uart3_last[80];
static uint8_t uart3_last_len;
static uint32_t car_source_time = 1u;
static uint32_t camera_source_time = 1u;
static uint32_t change_mode_calls;
static uint32_t unlock_calls;
static uint8_t camera_session_task_type;
static uint16_t camera_session_mission_id;
static uint8_t camera_session_mode_seq;
static uint32_t camera_track_mode_tx_count;
static uint32_t camera_idle_mode_tx_count;
static uint32_t camera_action_tx_count;
static uint8_t camera_target_is_valid = 1u;
static uint8_t s_yaw_arrived = 1u;

uint32_t GetSysRunTimeMs(void) { return fake_time_ms; }
void DrvUart1SendBuf(unsigned char *data, uint8_t len)
{
    if (uart1_history_count < 128u)
    {
        memcpy(uart1_history[uart1_history_count], data, len);
        uart1_history_len[uart1_history_count] = len;
        uart1_history_count++;
    }
}
void DrvUart2SendBuf(unsigned char *data, uint8_t len) { (void)data; (void)len; }
void DrvUart3SendBuf(unsigned char *data, uint8_t len)
{
    memcpy(uart3_last, data, len);
    uart3_last_len = len;
    if (len >= 11u && data[3] == CAMERA_TYPE_MODE)
    {
        if (data[9] == CAMERA_MODE_IDLE) camera_idle_mode_tx_count++;
        else camera_track_mode_tx_count++;
    }
    else if (len >= 11u && data[3] == CAMERA_TYPE_ACTION)
    {
        camera_action_tx_count++;
    }
}

void mode_select(int mode) { my_mode = mode; }
void set_dis_field_global(void) { dis_x_zero = 0; dis_y_zero = 0; }
int32_t get_corrected_dis_x(int32_t value) { return value - dis_x_zero; }
int32_t get_corrected_dis_y(int32_t value) { return value - dis_y_zero; }
void yaw_set_hold_target(int32_t target_deg) { yaw_target = target_deg; }
uint8_t yaw_arrived(float threshold_deg) { (void)threshold_deg; return s_yaw_arrived; }
void pid_set_radar_xy_limit(int32_t limit) { (void)limit; }
uint8_t LX_Change_Mode(uint8_t mode) { change_mode_calls++; fc_sta.fc_mode_sta = mode; return 1u; }
uint8_t FC_Unlock(void) { unlock_calls++; fc_sta.unlock_sta = 1u; return 1u; }
uint8_t OneKey_Land(void) { fc_sta.unlock_sta = 0u; return 1u; }
double my_sin(double radians) { return sin(radians); }
float my_cos(double radians) { return (float)cos(radians); }

static void fail(const char *message)
{
    fprintf(stderr, "[FAIL] mission V2.2 state simulation: %s\n", message);
    exit(1);
}

static void expect(int condition, const char *message)
{
    if (!condition) fail(message);
}

static void put_be16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value >> 8);
    dst[1] = (uint8_t)value;
}

static void put_be32(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value >> 24);
    dst[1] = (uint8_t)(value >> 16);
    dst[2] = (uint8_t)(value >> 8);
    dst[3] = (uint8_t)value;
}

static uint8_t make_lx(uint8_t subtype, const uint8_t *payload, uint8_t payload_len, uint8_t *frame)
{
    uint8_t count = 0u, sc = 0u, ac = 0u, i;
    frame[count++] = 0xBBu; frame[count++] = 0x10u; frame[count++] = 0xF1u;
    frame[count++] = 0x30u; frame[count++] = subtype; frame[count++] = payload_len;
    for (i = 0u; i < payload_len; i++) frame[count++] = payload[i];
    frame[count++] = 0xEEu;
    for (i = 0u; i < count; i++) { sc = (uint8_t)(sc + frame[i]); ac = (uint8_t)(ac + sc); }
    frame[count++] = sc; frame[count++] = ac;
    return count;
}

static uint8_t make_camera_frame(uint8_t type,
                                 uint8_t seq,
                                 const uint8_t *payload,
                                 uint8_t payload_len,
                                 uint8_t *frame)
{
    uint8_t count = 0u, i;
    uint16_t crc;

    frame[count++] = 0xAAu; frame[count++] = 0x55u; frame[count++] = 0x02u;
    frame[count++] = type; frame[count++] = 0x50u; frame[count++] = 0x21u;
    frame[count++] = seq; frame[count++] = 0u; frame[count++] = payload_len;
    for (i = 0u; i < payload_len; i++) frame[count++] = payload[i];
    crc = my_v21_crc16_ccitt_false(&frame[2], (uint8_t)(7u + payload_len));
    frame[count++] = (uint8_t)(crc >> 8); frame[count++] = (uint8_t)crc;
    return count;
}

static uint8_t make_camera_target(uint8_t *frame)
{
    uint8_t payload[18] = {0};

    payload[0] = (camera_target_is_valid != 0u) ? CAMERA_TARGET_VALID : 0u;
    payload[1] = 90u;
    put_be16(&payload[2], 0u);
    put_be16(&payload[4], 0u);
    put_be16(&payload[6], 300u);
    put_be16(&payload[8], (uint16_t)camera_source_time);
    put_be32(&payload[10], camera_source_time++);
    payload[14] = camera_session_task_type;
    put_be16(&payload[15], camera_session_mission_id);
    payload[17] = camera_session_mode_seq;
    return make_camera_frame(CAMERA_TYPE_TARGET, 1u,
                             payload, sizeof(payload), frame);
}

static void feed_bytes(void (*receiver)(uint8_t), const uint8_t *frame, uint8_t len)
{
    uint8_t i;
    for (i = 0u; i < len; i++) receiver(frame[i]);
}

static void feed_mid360(void)
{
    uint8_t frame[14] = {0};
    frame[0] = 0xAAu;
    put_be16(&frame[1], (uint16_t)dis_x_slam);
    put_be16(&frame[3], (uint16_t)dis_y_slam);
    put_be16(&frame[11], (uint16_t)yaw_slam);
    frame[13] = 0x0Au;
    feed_bytes(MY_uart_radar_receive, frame, sizeof(frame));
}

static void feed_car_pose(void)
{
    uint8_t payload[22] = {0};
    uint8_t frame[40];
    uint8_t len;
    payload[0] = CAR_FRAME_FIELD_GLOBAL;
    payload[1] = CAR_POSE_POSITION_VALID | CAR_POSE_CALIBRATED | CAR_POSE_YAW_VALID;
    put_be16(&payload[2], 1u);
    put_be32(&payload[4], 120u);
    put_be32(&payload[8], (uint32_t)-50);
    put_be16(&payload[12], 900u);
    put_be16(&payload[14], 0x7FFFu);
    put_be16(&payload[16], 0x7FFFu);
    put_be32(&payload[18], car_source_time++);
    len = make_lx(LX_EXT_CAR_POSE, payload, sizeof(payload), frame);
    feed_bytes(MY_uart_esp_receive, frame, len);
}

static void feed_camera(void)
{
    uint8_t frame[32];
    uint8_t len = make_camera_target(frame);
    feed_bytes(MY_uart_maixcam_receive, frame, len);
}

static void feed_last_camera_ack(uint8_t request_type)
{
    uint8_t payload[4];
    uint8_t frame[20];
    uint8_t len;

    expect(uart3_last_len >= 11u && uart3_last[3] == request_type,
           "last camera TX has unexpected request type");
    if (request_type == CAMERA_TYPE_MODE && uart3_last[9] != CAMERA_MODE_IDLE)
    {
        camera_session_mode_seq = uart3_last[6];
        camera_session_task_type = uart3_last[10];
        camera_session_mission_id = (uint16_t)(((uint16_t)uart3_last[11] << 8) |
                                               uart3_last[12]);
    }
    payload[0] = request_type;
    payload[1] = uart3_last[6];
    payload[2] = V21_ACK_ACCEPTED;
    payload[3] = 0u;
    len = make_camera_frame(0x11u, 0x70u, payload, sizeof(payload), frame);
    feed_bytes(MY_uart_maixcam_receive, frame, len);
}

static void feed_health(uint8_t include_camera)
{
    feed_mid360();
    MY_uart_mark_laser_update(hight, 1u);
    feed_car_pose();
    if (include_camera != 0u) feed_camera();
}

static void send_request(uint8_t task_type, uint16_t mission_id, uint16_t calibration_id)
{
    uint8_t payload[12] = {0};
    uint8_t frame[32];
    uint8_t len;
    payload[0] = task_type;
    payload[1] = 0x01u;
    put_be16(&payload[2], mission_id);
    put_be16(&payload[4], calibration_id);
    put_be32(&payload[8], mission_id);
    len = make_lx(LX_EXT_MISSION_REQUEST, payload, sizeof(payload), frame);
    feed_bytes(MY_uart_esp_receive, frame, len);
}

static int find_response_type(uint8_t request_type, uint16_t mission_id,
                              uint8_t result, uint8_t detail)
{
    uint8_t i;
    for (i = 0u; i < uart1_history_count; i++)
    {
        const uint8_t *f = uart1_history[i];
        if (uart1_history_len[i] == 15u && f[3] == 0x30u &&
            f[4] == LX_EXT_MISSION_RESPONSE && f[6] == request_type &&
            f[7] == (uint8_t)(mission_id >> 8) &&
            f[8] == (uint8_t)mission_id && f[9] == result && f[10] == detail)
        {
            return 1;
        }
    }
    return 0;
}

static int find_response(uint16_t mission_id, uint8_t result, uint8_t detail)
{
    return find_response_type(0x81u, mission_id, result, detail);
}

static uint8_t count_legacy_command(uint16_t command)
{
    uint8_t i;
    uint8_t count = 0u;
    for (i = 0u; i < uart1_history_count; i++)
    {
        const uint8_t *f = uart1_history[i];
        if (uart1_history_len[i] == 9u && f[0] == 0xBBu && f[1] == 0x10u &&
            f[2] == 0xF1u && f[3] == 0x02u &&
            f[4] == (uint8_t)command && f[5] == (uint8_t)(command >> 8) &&
            f[6] == 0xEEu)
        {
            count++;
        }
    }
    return count;
}

static uint8_t count_start_warnings(void)
{
    return count_legacy_command(ESP_DIAG_START_WARNING);
}

static void tick_perfect_tracking(uint8_t include_camera)
{
    fake_time_ms += 20u;
    dis_x_slam = dis_x_target;
    dis_y_slam = dis_y_target;
    if (height_target > 0)
    {
        hight = height_target;
    }
    yaw_slam = (int32_t)yaw_target * 10;
    if ((fake_time_ms % 100u) == 0u) feed_health(include_camera);
    else MY_uart_mark_laser_update(hight, 1u);
    UserTask_OneKeyCmd();
}

static void tick_without_car(void)
{
    fake_time_ms += 20u;
    dis_x_slam = dis_x_target;
    dis_y_slam = dis_y_target;
    if (height_target > 0) hight = height_target;
    yaw_slam = (int32_t)yaw_target * 10;
    if ((fake_time_ms % 100u) == 0u)
    {
        feed_mid360();
        feed_camera();
    }
    MY_uart_mark_laser_update(hight, 1u);
    UserTask_OneKeyCmd();
}

int main(void)
{
    uint32_t ticks;
    int32_t stopped_range;
    uint8_t car_loss_abort_seen = 0u;

    rc_in.rc_ch.st_data.ch_[5] = 1900;
    hight = 150;
    UserTask_OneKeyCmd();
    expect(count_legacy_command(ESP_EVENT_STAGE_IDLE) == 1u,
           "state-machine initialization did not report IDLE exactly once");
    fake_time_ms += 100u; feed_car_pose();
    fake_time_ms += 100u; feed_car_pose();
    fake_time_ms += 100u; feed_health(1u);

    send_request(LX_TASK_DYNAMIC_LAND, 1u, 2u);
    UserTask_OneKeyCmd();
    expect(find_response(1u, LX_RESULT_STATE_DENIED, LX_DETAIL_CALIBRATION),
           "CalibrationId mismatch was not rejected by the FC");

    rc_in.rc_ch.st_data.ch_[5] = 1500;
    send_request(LX_TASK_DYNAMIC_LAND, 2u, 1u);
    UserTask_OneKeyCmd();
    expect(find_response(2u, LX_RESULT_STATE_DENIED, LX_DETAIL_CH6),
           "CH6-off request was not rejected by the FC");
    expect(count_legacy_command(ESP_DIAG_RC_OR_CH6) == 1u,
           "CH6-off request did not report the RC/CH6 diagnostic code");

    rc_in.rc_ch.st_data.ch_[5] = 1900;
    fake_time_ms += 20u;
    feed_health(1u);
    send_request(LX_TASK_DYNAMIC_LAND, 3u, 1u);
    UserTask_OneKeyCmd();
    expect(!find_response(3u, LX_RESULT_ACCEPTED, LX_DETAIL_NONE),
           "task was accepted before matching CAMERA_MODE ACK");
    feed_last_camera_ack(CAMERA_TYPE_MODE);
    UserTask_OneKeyCmd();
    expect(find_response(3u, LX_RESULT_ACCEPTED, LX_DETAIL_NONE),
           "valid request was not accepted by the FC");
    expect(mission_step == MISSION_STAGE_PRECHECK,
           "accepted request did not enter PRECHECK");
    expect(count_legacy_command(ESP_EVENT_CAMERA_MODE_ACK_CONFIRMED) == 1u &&
               count_legacy_command(ESP_EVENT_STAGE_PRECHECK) == 1u,
           "CAMERA_MODE ACK or PRECHECK transition was not edge-reported once");
    expect(count_start_warnings() == 1u,
           "only the accepted request should send one ESP32 0x10 start warning");
    expect(change_mode_calls == 0u && unlock_calls == 0u && fc_sta.unlock_sta == 0u,
           "accepted request started flight control before the warning delay");

    send_request(LX_TASK_DYNAMIC_LAND, 3u, 1u);
    UserTask_OneKeyCmd();
    expect(find_response(3u, LX_RESULT_DUP_ACCEPTED, LX_DETAIL_NONE),
           "accepted duplicate did not return duplicate-accepted");
    expect(count_start_warnings() == 1u,
           "duplicate request sent the ESP32 start warning more than once");

    for (ticks = 0u; ticks < 499u; ticks++) tick_perfect_tracking(1u);
    expect(mission_step == MISSION_STAGE_PRECHECK &&
           change_mode_calls == 0u && unlock_calls == 0u && fc_sta.unlock_sta == 0u,
           "flight control started before the 10 s warning delay elapsed");
    tick_perfect_tracking(1u);
    expect(change_mode_calls == 1u && unlock_calls == 0u && fc_sta.unlock_sta == 0u,
           "flight mode was not started at the end of the 10 s warning delay");
    tick_perfect_tracking(1u);
    tick_perfect_tracking(1u);
    expect(unlock_calls == 1u && fc_sta.unlock_sta != 0u,
           "FC unlock did not follow the completed warning delay and mode confirmation");

    /* A live camera without a recognized target must no longer block dynamic landing. */
    camera_target_is_valid = 0u;
    for (ticks = 0u; ticks < 800u && mission_step != MISSION_STAGE_DESCEND; ticks++)
    {
        tick_perfect_tracking(1u);
    }
    expect(mission_step == MISSION_STAGE_DESCEND,
           "dynamic-land task did not reach DESCEND under stable inputs");

    for (ticks = 0u; ticks < 20u; ticks++) tick_perfect_tracking(0u);
    expect(mission_step == MISSION_STAGE_DESCEND,
           "camera age below 1 s prematurely left DESCEND");
    stopped_range = height_target;
    for (ticks = 0u; ticks < 20u; ticks++) tick_perfect_tracking(0u);
    expect(height_target == stopped_range,
           "camera age above 200 ms did not stop descent");
    for (ticks = 0u; ticks < 20u && mission_step == MISSION_STAGE_DESCEND; ticks++)
    {
        tick_perfect_tracking(0u);
    }
    expect(mission_step == MISSION_STAGE_LAND_ALIGN,
           "camera loss above 1 s did not return to high LAND_ALIGN");

    for (ticks = 0u; ticks < 500u && mission_step != MISSION_STAGE_DESCEND; ticks++)
    {
        tick_perfect_tracking(1u);
    }
    expect(mission_step == MISSION_STAGE_DESCEND,
           "camera recovery did not allow a new stabilized descent");
    for (ticks = 0u; ticks < 15u; ticks++) tick_without_car();
    stopped_range = height_target;
    for (ticks = 0u; ticks < 20u; ticks++)
    {
        tick_without_car();
        if (count_legacy_command(ESP_EVENT_ABORT_CAR_POSE_LOST) != 0u)
        {
            car_loss_abort_seen = 1u;
            break;
        }
    }
    expect(car_loss_abort_seen != 0u,
           "CAR_POSE loss above 500 ms during descent did not request ABORT");
    expect(count_legacy_command(ESP_EVENT_FC_MODE_2_CONFIRMED) == 1u &&
               count_legacy_command(ESP_EVENT_FC_UNLOCK_CONFIRMED) == 1u,
           "mode/unlock confirmation events were not edge-reported once");
    expect(count_legacy_command(ESP_EVENT_STAGE_ABORT) == 1u &&
               count_legacy_command(ESP_EVENT_ABORT_CAR_POSE_LOST) == 1u,
           "CAR_POSE ABORT transition/reason was not edge-reported once");
    expect(height_target >= stopped_range,
           "CAR_POSE loss during descent commanded a lower range target");

    puts("[PASS] mission V2.2 state simulation: MODE ACK gate, warning delay, dedup, flow, camera/car failsafe");
    return 0;
}
