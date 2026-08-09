#include "User_Task.h"
#ifndef UNIT_TEST
#include "Drv_RcIn.h"
#include "Drv_Sys.h"
#include "LX_FC_Fun.h"
#include "LX_FC_State.h"
#include "Ano_Math.h"
#endif

#include "mission_command.h"
#include "my_contrl.h"
#include "my_pid.h"
#include "my_send_test.h"
#include "my_uart.h"

#ifndef UNIT_TEST
#include "my_get_data.h"
#else
#define RAD_PER_DEG 0.017453293f
#define ch_6_aux2 5
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
extern test_rc_input_t rc_in;
extern test_fc_state_t fc_sta;
extern double yaw;
extern int32_t hight, dis_x_slam, dis_y_slam, yaw_slam;
extern uint32_t GetSysRunTimeMs(void);
extern uint8_t LX_Change_Mode(uint8_t mode);
extern uint8_t FC_Unlock(void);
extern uint8_t OneKey_Land(void);
extern double my_sin(double radians);
extern float my_cos(double radians);
#endif

#define TASK_PERIOD_MS                    20u
#define START_WARNING_DELAY_MS            10000u

#define FIELD_X_MIN_CM                    (-400)
#define FIELD_X_MAX_CM                    400
#define FIELD_Y_MIN_CM                    (-500)
#define FIELD_Y_MAX_CM                    500

#define GROUND_CRUISE_RANGE_CM            150
#define PLATFORM_CRUISE_RANGE_CM          135
#define DROP_PLATFORM_RANGE_CM             65
#define DROP_RELEASE_BODY_X_ADVANCE_CM     15
#define PLATFORM_HEIGHT_COMP_CM           15
#define LANDING_CONTACT_RANGE_CM          16
#define LANDING_RANGE_MARGIN_CM           2
#define LANDING_TERMINAL_RANGE_CM         (LANDING_CONTACT_RANGE_CM + LANDING_RANGE_MARGIN_CM)
#define PLATFORM_CONTACT_HOLD_RANGE_CM    14

#define PRECHECK_SENSOR_FRESH_MS          250u
#define CAR_POSE_WARNING_MS               280u
#define CAR_POSE_INVALID_MS               500u
#define CAMERA_DESCEND_STOP_MS            200u
#define CAMERA_INVALID_MS                 1000u
#define CAMERA_LINK_FRESH_MS              1000u
#define CAMERA_PRECHECK_WAIT_MS           1000u
#define CAMERA_CONTROL_ACK_TIMEOUT_MS      200u
#define CAMERA_CONTROL_MAX_ATTEMPTS        3u
#define REQUEST_DEDUP_MS                  5000u

#define CAMERA_QUALITY_MIN                60u
#define CAMERA_ALIGN_ERR_CM               8
#define VISUAL_CORRECTION_LIMIT_CM        25
#define VISUAL_FILTER_DIV                 4

#define HEIGHT_ARRIVE_CM                  10
#define INTERCEPT_ARRIVE_CM               25
#define FOLLOW_ARRIVE_CM                  20
#define TERMINAL_POS_ARRIVE_CM            12
#define DROP_TERMINAL_POS_ARRIVE_CM       18
#define DROP_HEIGHT_ARRIVE_CM             15
#define HOME_ARRIVE_CM                    7
#define YAW_ARRIVE_DEG                    5.0f

#define TAKEOFF_STABLE_MS                 2000u
#define INTERCEPT_STABLE_MS               1000u
#define FOLLOW_STABLE_MS                  1000u
#define DROP_ALIGN_STABLE_MS               300u
#define LAND_ALIGN_STABLE_MS               500u
#define LAND_CONTACT_STABLE_MS            500u
#define PLATFORM_HOLD_MS                  5000u
#define PLATFORM_TAKEOFF_STABLE_MS        3000u
#define HOME_STABLE_MS                    1500u

#define PRECHECK_COMMAND_TIMEOUT_MS       3000u
#define TAKEOFF_TIMEOUT_MS                30000u
#define INTERCEPT_TIMEOUT_MS              45000u
#define FOLLOW_TIMEOUT_MS                 30000u
#define DROP_ACTION_TIMEOUT_MS            5000u
#define ALIGN_TIMEOUT_MS                  30000u
#define DESCENT_TIMEOUT_MS                60000u
#define RETURN_HOME_TIMEOUT_MS            45000u
#define LAND_COMMAND_TIMEOUT_MS           20000u

#define INTERCEPT_SPEED_CMPS              40
#define FOLLOW_SPEED_CMPS                 30
#define TERMINAL_SPEED_CMPS               17
#define RETURN_SPEED_CMPS                 40

#define PLATFORM_NEAR_RADIUS_CM           70
#define CAR_EXTRAPOLATION_MAX_MS          250u
#define DESCENT_STEP_INTERVAL_MS          100u

#define STATUS_RUNNING                    0x0001u
#define STATUS_CAR_FRESH                  0x0002u
#define STATUS_CAMERA_FRESH               0x0004u
#define STATUS_CALIBRATION_OK             0x0008u
#define STATUS_CAMERA_LINK                0x0010u
#define STATUS_CAR_WARNING                0x0040u
#define STATUS_DESCENT_INHIBITED          0x0080u

#define ERROR_NONE                        0u
#define ERROR_PRECHECK                    1u
#define ERROR_CAR_LOST                    2u
#define ERROR_CAMERA_LOST                 3u
#define ERROR_CALIBRATION                 4u
#define ERROR_CAMERA_ACTION               7u
#define ERROR_TIMEOUT                     8u
#define ERROR_CH6_OR_RC                    9u
#define ERROR_REMOTE_ABORT                 10u

typedef struct
{
    uint8_t valid;
    uint8_t task_type;
    uint16_t mission_id;
    uint8_t result;
    uint8_t detail;
    uint32_t cached_ms;
} mission_response_cache_t;

static mission_command_state_t s_command_state;
static uint8_t s_initialized;
static uint8_t s_active;
static uint8_t s_task_type;
static uint8_t s_stage;
static uint8_t s_error_code;
static uint16_t s_mission_id;
static uint16_t s_calibration_id;
static uint32_t s_stage_enter_ms;
static uint32_t s_stable_ms;
static uint8_t s_stage_substate;
static uint32_t s_substate_enter_ms;

static int32_t s_home_x_cm;
static int32_t s_home_y_cm;
static int32_t s_last_car_x_cm;
static int32_t s_last_car_y_cm;
static int32_t s_abort_hold_x_cm;
static int32_t s_abort_hold_y_cm;
static int32_t s_range_target_cm;
static uint16_t s_descent_step_ms;

static int32_t s_visual_corr_x_cm;
static int32_t s_visual_corr_y_cm;
static uint8_t s_visual_filter_initialized;
static uint8_t s_descent_inhibited;

static uint8_t s_abort_hard_land;
static uint8_t s_abort_return_started;
static uint8_t s_land_command_queued;
static uint16_t s_action_id;
static uint32_t s_action_result_start_counter;
static uint8_t s_action_seq;
static uint8_t s_action_tx_attempts;
static uint8_t s_action_acknowledged;
static uint32_t s_action_last_tx_ms;
static uint32_t s_action_ack_rx_ms;

static uint8_t s_camera_mode_seq;
static uint8_t s_camera_mode_tx_attempts;
static uint32_t s_camera_mode_last_tx_ms;
static uint8_t s_camera_idle_pending;
static uint8_t s_camera_idle_seq;
static uint8_t s_camera_idle_tx_attempts;
static uint32_t s_camera_idle_last_tx_ms;

static lx_car_pose_t s_pose;
static camera_target_t s_camera_target;
static camera_action_result_t s_camera_result;
static uint32_t s_last_processed_pose_counter;

static uint8_t s_pending_request_valid;
static lx_mission_request_t s_pending_request;
static uint32_t s_pending_request_started_ms;
static mission_response_cache_t s_response_cache;
static uint16_t s_last_precheck_diagnostic;
static uint8_t s_abort_reason_reported;
static uint8_t s_terminal_align_blockers;

static void report_mission_trace(uint8_t event);

static int32_t abs_i32(int32_t value)
{
    return (value >= 0) ? value : -value;
}

static int32_t clamp_i32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static int32_t round_f64_to_i32(double value)
{
    return (value >= 0.0) ? (int32_t)(value + 0.5) : (int32_t)(value - 0.5);
}

static uint8_t elapsed_fresh(uint32_t now, uint32_t timestamp, uint32_t limit_ms)
{
    return ((uint32_t)(now - timestamp) <= limit_ms) ? 1u : 0u;
}

/*
 * Keep the ESP32 console useful during a 20 ms task loop: report a failed
 * precheck once, then only report again after the failure type changes.  A
 * healthy precheck clears the latch, so a later recurrence is visible too.
 */
static void report_precheck_diagnostic(uint16_t diagnostic)
{
    if (diagnostic == 0u)
    {
        s_last_precheck_diagnostic = 0u;
        return;
    }
    if (s_last_precheck_diagnostic != diagnostic)
    {
        my_send_esp_1_test((int)diagnostic);
        s_last_precheck_diagnostic = diagnostic;
    }
}

static uint16_t stage_event_code(uint8_t stage)
{
    switch (stage)
    {
    case MISSION_STAGE_IDLE: return ESP_EVENT_STAGE_IDLE;
    case MISSION_STAGE_PRECHECK: return ESP_EVENT_STAGE_PRECHECK;
    case MISSION_STAGE_TAKEOFF: return ESP_EVENT_STAGE_TAKEOFF;
    case MISSION_STAGE_INTERCEPT: return ESP_EVENT_STAGE_INTERCEPT;
    case MISSION_STAGE_FOLLOW: return ESP_EVENT_STAGE_FOLLOW;
    case MISSION_STAGE_DROP_ALIGN: return ESP_EVENT_STAGE_DROP_ALIGN;
    case MISSION_STAGE_DROP_ACTION: return ESP_EVENT_STAGE_DROP_ACTION;
    case MISSION_STAGE_LAND_ALIGN: return ESP_EVENT_STAGE_LAND_ALIGN;
    case MISSION_STAGE_DESCEND: return ESP_EVENT_STAGE_DESCEND;
    case MISSION_STAGE_ON_PLATFORM_5S: return ESP_EVENT_STAGE_ON_PLATFORM_HOLD;
    case MISSION_STAGE_PLATFORM_TAKEOFF: return ESP_EVENT_STAGE_PLATFORM_TAKEOFF;
    case MISSION_STAGE_RETURN_HOME: return ESP_EVENT_STAGE_RETURN_HOME;
    case MISSION_STAGE_HOME_LAND: return ESP_EVENT_STAGE_HOME_LAND;
    case MISSION_STAGE_ABORT: return ESP_EVENT_STAGE_ABORT;
    default: return 0u;
    }
}

static uint16_t abort_event_code(uint8_t error_code)
{
    switch (error_code)
    {
    case ERROR_PRECHECK: return ESP_EVENT_ABORT_PRECHECK;
    case ERROR_CAR_LOST: return ESP_EVENT_ABORT_CAR_POSE_LOST;
    case ERROR_CAMERA_LOST: return ESP_EVENT_ABORT_CAMERA_LOST;
    case ERROR_CALIBRATION: return ESP_EVENT_ABORT_CALIBRATION;
    case ERROR_CAMERA_ACTION: return ESP_EVENT_ABORT_CAMERA_ACTION;
    case ERROR_TIMEOUT: return ESP_EVENT_ABORT_TIMEOUT;
    case ERROR_CH6_OR_RC: return ESP_EVENT_ABORT_RC_STOP;
    case ERROR_REMOTE_ABORT: return ESP_EVENT_ABORT_REMOTE_COMMAND;
    default: return ESP_EVENT_ABORT_INVALID_STAGE;
    }
}

static void report_mission_event(uint16_t event)
{
    if (event != 0u)
    {
        my_send_esp_1_test((int)event);
    }
}

static void get_air_xy(int32_t *x_cm, int32_t *y_cm)
{
    *x_cm = get_corrected_dis_x(dis_x_slam);
    *y_cm = get_corrected_dis_y(dis_y_slam);
}

static uint8_t car_pose_fresh(uint32_t now, uint32_t limit_ms)
{
    return (s_pose.rx_counter != 0u && s_pose.usable != 0u &&
            elapsed_fresh(now, s_pose.local_rx_ms, limit_ms) != 0u)
               ? 1u
               : 0u;
}

static uint8_t camera_link_fresh_within(uint32_t now, uint32_t limit_ms)
{
    return (s_camera_target.rx_counter != 0u &&
            elapsed_fresh(now, MY_uart_camera_last_rx_ms(), limit_ms) != 0u)
               ? 1u
               : 0u;
}

static uint8_t camera_link_fresh(uint32_t now)
{
    return camera_link_fresh_within(now, CAMERA_LINK_FRESH_MS);
}

static uint8_t camera_target_fresh_and_valid(uint32_t now, uint32_t limit_ms)
{
    return (s_camera_target.rx_counter != 0u &&
            MY_uart_camera_session_ready(s_task_type,
                                         s_mission_id,
                                         s_camera_mode_seq) != 0u &&
            elapsed_fresh(now, s_camera_target.local_rx_ms, limit_ms) != 0u &&
            s_camera_target.task_type == s_task_type &&
            s_camera_target.mission_id == s_mission_id &&
            s_camera_target.mode_seq == s_camera_mode_seq &&
            (s_camera_target.flags & CAMERA_TARGET_VALID) != 0u &&
            s_camera_target.quality >= CAMERA_QUALITY_MIN &&
            s_camera_target.outer_diameter_px != 0u)
               ? 1u
               : 0u;
}

static uint16_t mission_status_flags(uint32_t now)
{
    uint16_t flags = 0u;

    if (s_active != 0u)
    {
        flags |= STATUS_RUNNING;
    }
    if (car_pose_fresh(now, CAR_POSE_INVALID_MS) != 0u)
    {
        flags |= STATUS_CAR_FRESH;
    }
    if (camera_target_fresh_and_valid(now, CAMERA_INVALID_MS) != 0u)
    {
        flags |= STATUS_CAMERA_FRESH;
    }
    if (camera_link_fresh(now) != 0u)
    {
        flags |= STATUS_CAMERA_LINK;
    }
    if (s_calibration_id != 0u && s_pose.calibration_id == s_calibration_id &&
        (s_pose.pose_flags & CAR_POSE_CALIBRATED) != 0u)
    {
        flags |= STATUS_CALIBRATION_OK;
    }
    if (s_pose.rx_counter != 0u &&
        (uint32_t)(now - s_pose.local_rx_ms) > CAR_POSE_WARNING_MS)
    {
        flags |= STATUS_CAR_WARNING;
    }
    if (s_descent_inhibited != 0u)
    {
        flags |= STATUS_DESCENT_INHIBITED;
    }
    return flags;
}

static void send_status(uint8_t stage)
{
    uint32_t now = GetSysRunTimeMs();

    my_send_esp_mission_status(s_task_type,
                               stage,
                               mission_status_flags(now),
                               s_mission_id,
                               s_error_code,
                               now);
}

static void set_stage(uint8_t stage)
{
    if (s_stage == stage)
    {
        return;
    }
    s_stage = stage;
    mission_step = stage;
    s_stage_enter_ms = GetSysRunTimeMs();
    s_substate_enter_ms = s_stage_enter_ms;
    s_stage_substate = 0u;
    s_stable_ms = 0u;
    s_descent_step_ms = 0u;
    report_mission_event(stage_event_code(stage));
    send_status(stage);
    report_mission_trace(MISSION_TRACE_STAGE_ENTER);
}

static uint8_t stable_for(uint8_t condition, uint32_t required_ms)
{
    if (condition != 0u)
    {
        if (s_stable_ms < required_ms)
        {
            s_stable_ms += TASK_PERIOD_MS;
        }
    }
    else
    {
        s_stable_ms = 0u;
    }
    return (s_stable_ms >= required_ms) ? 1u : 0u;
}

static void set_radar_target(int32_t x_cm, int32_t y_cm, int32_t range_cm, int32_t speed_cmps)
{
    dis_x_target = clamp_i32(x_cm, FIELD_X_MIN_CM, FIELD_X_MAX_CM);
    dis_y_target = clamp_i32(y_cm, FIELD_Y_MIN_CM, FIELD_Y_MAX_CM);
    height_target = range_cm;
    pid_set_radar_xy_limit(speed_cmps);
}

static uint8_t position_arrived(int32_t target_x_cm, int32_t target_y_cm, int32_t threshold_cm)
{
    int32_t air_x;
    int32_t air_y;

    get_air_xy(&air_x, &air_y);
    return (abs_i32(target_x_cm - air_x) <= threshold_cm &&
            abs_i32(target_y_cm - air_y) <= threshold_cm)
               ? 1u
               : 0u;
}

static uint8_t height_arrived(int32_t target_cm)
{
    return (abs_i32(target_cm - hight) <= HEIGHT_ARRIVE_CM) ? 1u : 0u;
}

static uint8_t height_arrived_within(int32_t target_cm, int32_t threshold_cm)
{
    return (abs_i32(target_cm - hight) <= threshold_cm) ? 1u : 0u;
}

static int32_t car_yaw_deg(void)
{
    int32_t value = s_pose.yaw_0p1deg;

    if (value >= 0)
    {
        return (value + 5) / 10;
    }
    return (value - 5) / 10;
}

static void align_yaw_world_frame(void)
{
    /* MID360 yaw and V2.2 CAR yaw are both 0.1 degree, CCW-positive. */
    yaw_zero = yaw - ((double)yaw_slam * 0.1);
}

static void car_target(uint32_t now, uint8_t use_visual, int32_t *target_x, int32_t *target_y)
{
    uint32_t age_ms = now - s_pose.local_rx_ms;
    int32_t car_x = s_pose.x_cm;
    int32_t car_y = s_pose.y_cm;

    if (age_ms > CAR_EXTRAPOLATION_MAX_MS)
    {
        age_ms = CAR_EXTRAPOLATION_MAX_MS;
    }
    if ((s_pose.pose_flags & CAR_POSE_VELOCITY_VALID) != 0u &&
        s_pose.vx_cmps != (int16_t)0x7FFF && s_pose.vy_cmps != (int16_t)0x7FFF)
    {
        car_x += ((int32_t)s_pose.vx_cmps * (int32_t)age_ms) / 1000;
        car_y += ((int32_t)s_pose.vy_cmps * (int32_t)age_ms) / 1000;
    }

    s_last_car_x_cm = car_x;
    s_last_car_y_cm = car_y;
    *target_x = car_x;
    *target_y = car_y;

    if (use_visual != 0u &&
        camera_target_fresh_and_valid(now, CAMERA_DESCEND_STOP_MS) != 0u)
    {
        int32_t air_x;
        int32_t air_y;
        int32_t measured_x;
        int32_t measured_y;
        int32_t correction_x;
        int32_t correction_y;
        double yaw_rad = ((double)yaw_slam * 0.1) * (double)RAD_PER_DEG;
        double sin_yaw = my_sin(yaw_rad);
        double cos_yaw = my_cos(yaw_rad);
        int32_t world_dx = (int32_t)(s_camera_target.err_x_cm * cos_yaw -
                                     s_camera_target.err_y_cm * sin_yaw);
        int32_t world_dy = (int32_t)(s_camera_target.err_x_cm * sin_yaw +
                                     s_camera_target.err_y_cm * cos_yaw);

        get_air_xy(&air_x, &air_y);
        measured_x = air_x + world_dx;
        measured_y = air_y + world_dy;
        correction_x = clamp_i32(measured_x - car_x,
                                 -VISUAL_CORRECTION_LIMIT_CM,
                                 VISUAL_CORRECTION_LIMIT_CM);
        correction_y = clamp_i32(measured_y - car_y,
                                 -VISUAL_CORRECTION_LIMIT_CM,
                                 VISUAL_CORRECTION_LIMIT_CM);

        if (s_visual_filter_initialized == 0u)
        {
            s_visual_corr_x_cm = correction_x;
            s_visual_corr_y_cm = correction_y;
            s_visual_filter_initialized = 1u;
        }
        else
        {
            s_visual_corr_x_cm += (correction_x - s_visual_corr_x_cm) / VISUAL_FILTER_DIV;
            s_visual_corr_y_cm += (correction_y - s_visual_corr_y_cm) / VISUAL_FILTER_DIV;
        }
        *target_x = car_x + s_visual_corr_x_cm;
        *target_y = car_y + s_visual_corr_y_cm;
    }

    *target_x = clamp_i32(*target_x, FIELD_X_MIN_CM, FIELD_X_MAX_CM);
    *target_y = clamp_i32(*target_y, FIELD_Y_MIN_CM, FIELD_Y_MAX_CM);
}

static void apply_drop_release_body_x_advance(int32_t *target_x, int32_t *target_y)
{
    double yaw_rad = ((double)yaw_slam * 0.1) * (double)RAD_PER_DEG;
    int32_t world_x = round_f64_to_i32((double)DROP_RELEASE_BODY_X_ADVANCE_CM *
                                       (double)my_cos(yaw_rad));
    int32_t world_y = round_f64_to_i32((double)DROP_RELEASE_BODY_X_ADVANCE_CM *
                                       my_sin(yaw_rad));

    /* The released object lands about 15 cm toward body -X, so pre-position +X. */
    *target_x = clamp_i32(*target_x + world_x, FIELD_X_MIN_CM, FIELD_X_MAX_CM);
    *target_y = clamp_i32(*target_y + world_y, FIELD_Y_MIN_CM, FIELD_Y_MAX_CM);
}

static uint8_t camera_centered(uint32_t now)
{
    return (camera_target_fresh_and_valid(now, CAMERA_DESCEND_STOP_MS) != 0u &&
            abs_i32(s_camera_target.err_x_cm) <= CAMERA_ALIGN_ERR_CM &&
            abs_i32(s_camera_target.err_y_cm) <= CAMERA_ALIGN_ERR_CM)
               ? 1u
               : 0u;
}

static uint8_t terminal_alignment_blockers(uint32_t now,
                                           int32_t target_x_cm,
                                           int32_t target_y_cm,
                                           int32_t range_cm,
                                           int32_t position_threshold_cm,
                                           int32_t height_threshold_cm,
                                           uint8_t require_height,
                                           uint8_t require_yaw)
{
    uint8_t blockers = 0u;

    if (car_pose_fresh(now, CAR_POSE_INVALID_MS) == 0u)
    {
        blockers |= MISSION_ALIGN_BLOCK_CAR_POSE;
    }
    if (camera_link_fresh(now) == 0u)
    {
        blockers |= MISSION_ALIGN_BLOCK_CAMERA_LINK;
    }
    if (position_arrived(target_x_cm, target_y_cm, position_threshold_cm) == 0u)
    {
        blockers |= MISSION_ALIGN_BLOCK_POSITION;
    }
    if (require_height != 0u &&
        height_arrived_within(range_cm, height_threshold_cm) == 0u)
    {
        blockers |= MISSION_ALIGN_BLOCK_HEIGHT;
    }
    if (require_yaw != 0u && yaw_arrived(YAW_ARRIVE_DEG) == 0u)
    {
        blockers |= MISSION_ALIGN_BLOCK_YAW;
    }
    return blockers;
}

static int16_t trace_i16(int32_t value)
{
    if (value > 32767)
    {
        return 32767;
    }
    if (value < -32768)
    {
        return -32768;
    }
    return (int16_t)value;
}

static uint16_t trace_camera_age_ms(uint32_t now)
{
    uint32_t age_ms;

    if (s_camera_target.rx_counter == 0u)
    {
        return 0xFFFFu;
    }
    age_ms = now - s_camera_target.local_rx_ms;
    return (age_ms > 65534u) ? 65534u : (uint16_t)age_ms;
}

static void report_mission_trace(uint8_t event)
{
    mission_trace_t trace;
    uint32_t now = GetSysRunTimeMs();
    int32_t air_x;
    int32_t air_y;

    get_air_xy(&air_x, &air_y);
    trace.event = event;
    trace.stage = s_stage;
    trace.error_code = s_error_code;
    trace.flags = 0u;
    if (s_abort_hard_land != 0u) trace.flags |= MISSION_TRACE_FLAG_HARD_LAND;
    if (s_abort_return_started != 0u) trace.flags |= MISSION_TRACE_FLAG_RETURN_STARTED;
    if (camera_link_fresh(now) != 0u) trace.flags |= MISSION_TRACE_FLAG_CAMERA_LINK;
    if (camera_target_fresh_and_valid(now, CAMERA_DESCEND_STOP_MS) != 0u)
        trace.flags |= MISSION_TRACE_FLAG_CAMERA_VALID;
    if (camera_centered(now) != 0u) trace.flags |= MISSION_TRACE_FLAG_CAMERA_CENTERED;
    if (car_pose_fresh(now, CAR_POSE_INVALID_MS) != 0u)
        trace.flags |= MISSION_TRACE_FLAG_CAR_FRESH;
    if (fc_sta.unlock_sta != 0u) trace.flags |= MISSION_TRACE_FLAG_FC_UNLOCKED;
    trace.mission_id = s_mission_id;
    trace.fc_time_ms = now;
    trace.stage_elapsed_ms = now - s_stage_enter_ms;
    trace.air_x_cm = trace_i16(air_x);
    trace.air_y_cm = trace_i16(air_y);
    trace.home_x_cm = trace_i16(s_home_x_cm);
    trace.home_y_cm = trace_i16(s_home_y_cm);
    trace.height_cm = trace_i16(hight);
    trace.range_target_cm = trace_i16(s_range_target_cm);
    trace.camera_flags = s_camera_target.flags;
    trace.camera_quality = s_camera_target.quality;
    trace.camera_err_x_cm = s_camera_target.err_x_cm;
    trace.camera_err_y_cm = s_camera_target.err_y_cm;
    trace.camera_age_ms = trace_camera_age_ms(now);
    trace.align_blockers = s_terminal_align_blockers;
    my_send_esp_mission_trace(&trace);
}

static uint8_t camera_mode_for_task(uint8_t task_type)
{
    return (task_type == LX_TASK_DROP)
               ? CAMERA_MODE_TRACK_DROP
               : CAMERA_MODE_TRACK_LAND;
}

static void transmit_camera_mode(uint8_t retransmission)
{
    uint8_t header_flags = V21_FLAG_ACK_REQUIRED;

    if (retransmission != 0u)
    {
        header_flags |= V21_FLAG_RETRANSMISSION;
    }
    my_send_camera_mode_v22(s_camera_mode_seq,
                            header_flags,
                            camera_mode_for_task(s_task_type),
                            s_task_type,
                            s_mission_id,
                            0u);
    s_camera_mode_last_tx_ms = GetSysRunTimeMs();
}

static void begin_camera_session(uint8_t task_type, uint16_t mission_id)
{
    s_task_type = task_type;
    s_mission_id = mission_id;
    s_camera_mode_seq = my_camera_allocate_seq();
    s_camera_mode_tx_attempts = 1u;
    s_camera_idle_pending = 0u;
    MY_uart_camera_begin_session(task_type, mission_id, s_camera_mode_seq);
    MY_uart_camera_expect_ack(CAMERA_TYPE_MODE, s_camera_mode_seq);
    transmit_camera_mode(0u);
}

/* 0=waiting, 1=ready, 2=rejected/retry exhausted. */
static uint8_t service_camera_mode_handshake(uint32_t now)
{
    camera_ack_t ack;

    if (MY_uart_camera_session_ready(s_task_type,
                                     s_mission_id,
                                     s_camera_mode_seq) != 0u)
    {
        return 1u;
    }
    if (MY_uart_camera_get_expected_ack(CAMERA_TYPE_MODE,
                                        s_camera_mode_seq,
                                        &ack) != 0u &&
        ack.result != V21_ACK_ACCEPTED &&
        ack.result != V21_ACK_DUP_ACCEPTED)
    {
        return 2u;
    }
    if ((uint32_t)(now - s_camera_mode_last_tx_ms) < CAMERA_CONTROL_ACK_TIMEOUT_MS)
    {
        return 0u;
    }
    if (s_camera_mode_tx_attempts >= CAMERA_CONTROL_MAX_ATTEMPTS)
    {
        return 2u;
    }
    s_camera_mode_tx_attempts++;
    transmit_camera_mode(1u);
    return 0u;
}

static void transmit_camera_idle(uint8_t retransmission)
{
    uint8_t header_flags = V21_FLAG_ACK_REQUIRED;

    if (retransmission != 0u)
    {
        header_flags |= V21_FLAG_RETRANSMISSION;
    }
    my_send_camera_mode_v22(s_camera_idle_seq,
                            header_flags,
                            CAMERA_MODE_IDLE,
                            s_task_type,
                            s_mission_id,
                            0u);
    s_camera_idle_last_tx_ms = GetSysRunTimeMs();
}

static void send_camera_idle_and_invalidate(void)
{
    MY_uart_camera_invalidate_session();
    s_visual_filter_initialized = 0u;
    s_camera_mode_tx_attempts = 0u;
    s_camera_idle_seq = my_camera_allocate_seq();
    s_camera_idle_tx_attempts = 1u;
    s_camera_idle_pending = 1u;
    MY_uart_camera_expect_ack(CAMERA_TYPE_MODE, s_camera_idle_seq);
    transmit_camera_idle(0u);
}

static void service_camera_idle_handshake(uint32_t now)
{
    camera_ack_t ack;

    if (s_camera_idle_pending == 0u)
    {
        return;
    }
    if (MY_uart_camera_get_expected_ack(CAMERA_TYPE_MODE,
                                        s_camera_idle_seq,
                                        &ack) != 0u)
    {
        s_camera_idle_pending = 0u;
        MY_uart_camera_cancel_expected_ack();
        return;
    }
    if ((uint32_t)(now - s_camera_idle_last_tx_ms) < CAMERA_CONTROL_ACK_TIMEOUT_MS)
    {
        return;
    }
    if (s_camera_idle_tx_attempts >= CAMERA_CONTROL_MAX_ATTEMPTS)
    {
        s_camera_idle_pending = 0u;
        MY_uart_camera_cancel_expected_ack();
        return;
    }
    s_camera_idle_tx_attempts++;
    transmit_camera_idle(1u);
}

static void transmit_camera_action(uint8_t retransmission)
{
    uint8_t header_flags = V21_FLAG_ACK_REQUIRED;

    if (retransmission != 0u)
    {
        header_flags |= V21_FLAG_RETRANSMISSION;
    }
    my_send_camera_action_v22(s_action_seq,
                              header_flags,
                              CAMERA_ACTION_DROP,
                              0u,
                              s_action_id,
                              s_task_type,
                              s_mission_id,
                              s_camera_mode_seq);
    s_action_last_tx_ms = GetSysRunTimeMs();
}

static void begin_camera_action(void)
{
    s_action_seq = my_camera_allocate_seq();
    s_action_tx_attempts = 1u;
    s_action_acknowledged = 0u;
    s_action_ack_rx_ms = 0u;
    MY_uart_camera_begin_action(s_action_id, s_action_seq);
    MY_uart_camera_expect_ack(CAMERA_TYPE_ACTION, s_action_seq);
    transmit_camera_action(0u);
}

/* 0=waiting, 1=ACK accepted, 2=ACK rejected/retry exhausted. */
static uint8_t service_camera_action_ack(uint32_t now)
{
    camera_ack_t ack;

    if (s_action_acknowledged != 0u)
    {
        return 1u;
    }
    if (MY_uart_camera_get_expected_ack(CAMERA_TYPE_ACTION,
                                        s_action_seq,
                                        &ack) != 0u)
    {
        if (ack.result == V21_ACK_ACCEPTED ||
            ack.result == V21_ACK_DUP_ACCEPTED)
        {
            s_action_acknowledged = 1u;
            s_action_ack_rx_ms = ack.local_rx_ms;
            report_mission_event(ESP_EVENT_CAMERA_ACTION_ACK_CONFIRMED);
            return 1u;
        }
        return 2u;
    }
    if ((uint32_t)(now - s_action_last_tx_ms) < CAMERA_CONTROL_ACK_TIMEOUT_MS)
    {
        return 0u;
    }
    if (s_action_tx_attempts >= CAMERA_CONTROL_MAX_ATTEMPTS)
    {
        return 2u;
    }
    s_action_tx_attempts++;
    transmit_camera_action(1u);
    return 0u;
}

static void cache_and_send_response(const lx_mission_request_t *request,
                                    uint8_t result,
                                    uint8_t detail)
{
    s_response_cache.valid = 1u;
    s_response_cache.task_type = request->task_type;
    s_response_cache.mission_id = request->mission_id;
    s_response_cache.result = result;
    s_response_cache.detail = detail;
    s_response_cache.cached_ms = GetSysRunTimeMs();
    my_send_esp_mission_response(0x81u, request->mission_id, result, detail);
}

static uint8_t request_matches_cache(const lx_mission_request_t *request, uint32_t now)
{
    return (s_response_cache.valid != 0u &&
            (uint32_t)(now - s_response_cache.cached_ms) <= REQUEST_DEDUP_MS &&
            s_response_cache.task_type == request->task_type &&
            s_response_cache.mission_id == request->mission_id)
               ? 1u
               : 0u;
}

static uint8_t request_base_precheck(const lx_mission_request_t *request,
                                      uint32_t now,
                                      uint32_t car_pose_limit_ms,
                                      uint8_t *result,
                                      uint8_t *detail)
{
    if (request->task_type != LX_TASK_DROP && request->task_type != LX_TASK_DYNAMIC_LAND)
    {
        *result = LX_RESULT_INVALID_PARAM;
        *detail = LX_DETAIL_TASK_TYPE;
        return 0u;
    }
    if (request->request_flags != 0x01u || request->reserved != 0u)
    {
        *result = LX_RESULT_INVALID_PARAM;
        *detail = LX_DETAIL_NONE;
        return 0u;
    }
    if (s_active != 0u)
    {
        *result = LX_RESULT_BUSY;
        *detail = LX_DETAIL_MISSION_RUNNING;
        return 0u;
    }
    if (mission_command_ch6_allows_start((rc_in.no_signal == 0u && rc_in.fail_safe == 0u) ? 1u : 0u,
                                         rc_in.rc_ch.st_data.ch_[ch_6_aux2]) == 0u)
    {
        report_precheck_diagnostic(ESP_DIAG_RC_OR_CH6);
        *result = LX_RESULT_STATE_DENIED;
        *detail = LX_DETAIL_CH6;
        return 0u;
    }
    if (car_pose_fresh(now, car_pose_limit_ms) == 0u || s_pose.valid_streak < 3u)
    {
        report_precheck_diagnostic(ESP_DIAG_CAR_POSE);
        *result = LX_RESULT_STATE_DENIED;
        *detail = LX_DETAIL_POSE_STALE;
        return 0u;
    }
    if (request->calibration_id == 0u ||
        request->calibration_id != s_pose.calibration_id ||
        (s_pose.pose_flags & CAR_POSE_CALIBRATED) == 0u)
    {
        report_precheck_diagnostic(ESP_DIAG_CALIBRATION);
        *result = LX_RESULT_STATE_DENIED;
        *detail = LX_DETAIL_CALIBRATION;
        return 0u;
    }
    report_precheck_diagnostic(0u);
    *result = LX_RESULT_ACCEPTED;
    *detail = LX_DETAIL_NONE;
    return 1u;
}

static void accept_request(const lx_mission_request_t *request)
{
    int32_t air_x;
    int32_t air_y;

    get_air_xy(&air_x, &air_y);
    s_active = 1u;
    my_task_flag = 1;
    s_task_type = request->task_type;
    s_mission_id = request->mission_id;
    s_calibration_id = request->calibration_id;
    s_error_code = ERROR_NONE;
    s_abort_reason_reported = 0u;
    s_home_x_cm = clamp_i32(air_x, FIELD_X_MIN_CM, FIELD_X_MAX_CM);
    s_home_y_cm = clamp_i32(air_y, FIELD_Y_MIN_CM, FIELD_Y_MAX_CM);
    s_last_car_x_cm = s_pose.x_cm;
    s_last_car_y_cm = s_pose.y_cm;
    s_range_target_cm = GROUND_CRUISE_RANGE_CM;
    s_visual_filter_initialized = 0u;
    s_descent_inhibited = 0u;
    s_terminal_align_blockers = 0u;
    s_abort_hard_land = 0u;
    s_abort_return_started = 0u;
    s_land_command_queued = 0u;
    s_last_processed_pose_counter = s_pose.rx_counter;
    set_dis_field_global();
    align_yaw_world_frame();
    cache_and_send_response(request, LX_RESULT_ACCEPTED, LX_DETAIL_NONE);
    my_send_esp_1_test(ESP_DIAG_START_WARNING);
    set_stage(MISSION_STAGE_PRECHECK);
}

static void handle_request(const lx_mission_request_t *request)
{
    uint32_t now = GetSysRunTimeMs();
    uint8_t result;
    uint8_t detail;

    if (request_matches_cache(request, now) != 0u)
    {
        result = (s_response_cache.result == LX_RESULT_ACCEPTED)
                     ? LX_RESULT_DUP_ACCEPTED
                     : s_response_cache.result;
        my_send_esp_mission_response(0x81u, request->mission_id, result, s_response_cache.detail);
        return;
    }
    if (s_active != 0u && s_stage != MISSION_STAGE_ABORT &&
        request->mission_id == s_mission_id && request->task_type == s_task_type)
    {
        my_send_esp_mission_response(0x81u, request->mission_id,
                                     LX_RESULT_DUP_ACCEPTED, LX_DETAIL_NONE);
        return;
    }
    /* At the initial task request, require a 250 ms locally received pose. */
    if (request_base_precheck(request, now, PRECHECK_SENSOR_FRESH_MS,
                              &result, &detail) == 0u)
    {
        cache_and_send_response(request, result, detail);
        return;
    }

    if (s_pending_request_valid != 0u)
    {
        if (s_pending_request.mission_id != request->mission_id ||
            s_pending_request.task_type != request->task_type)
        {
            cache_and_send_response(request, LX_RESULT_BUSY, LX_DETAIL_MISSION_RUNNING);
        }
        return;
    }

    /*
     * Do not accept the car task merely because an old camera target is fresh.
     * First bind a new visual session and wait for CAMERA_MODE ACK.
     */
    s_pending_request = *request;
    s_pending_request_valid = 1u;
    s_pending_request_started_ms = now;
    begin_camera_session(request->task_type, request->mission_id);
}

static void process_pending_request(void)
{
    uint32_t now;
    uint8_t camera_mode_state;
    uint8_t result;
    uint8_t detail;

    if (s_pending_request_valid == 0u)
    {
        return;
    }
    now = GetSysRunTimeMs();
    /* The initial 250 ms admission has already passed.  During the camera
     * MODE ACK wait, use the same 500 ms loss threshold as PRECHECK/flight. */
    if (request_base_precheck(&s_pending_request, now, CAR_POSE_INVALID_MS,
                              &result, &detail) == 0u)
    {
        cache_and_send_response(&s_pending_request, result, detail);
        s_pending_request_valid = 0u;
        send_camera_idle_and_invalidate();
        return;
    }
    camera_mode_state = service_camera_mode_handshake(now);
    if (camera_mode_state == 1u)
    {
        lx_mission_request_t request = s_pending_request;
        s_pending_request_valid = 0u;
        report_mission_event(ESP_EVENT_CAMERA_MODE_ACK_CONFIRMED);
        accept_request(&request);
        return;
    }
    if (camera_mode_state == 2u)
    {
        report_precheck_diagnostic(ESP_DIAG_CAMERA_MODE);
        cache_and_send_response(&s_pending_request,
                                LX_RESULT_STATE_DENIED,
                                LX_DETAIL_CAMERA_MODE);
        s_pending_request_valid = 0u;
        send_camera_idle_and_invalidate();
        return;
    }
    if ((uint32_t)(now - s_pending_request_started_ms) >= CAMERA_PRECHECK_WAIT_MS)
    {
        report_precheck_diagnostic(ESP_DIAG_CAMERA_MODE);
        cache_and_send_response(&s_pending_request,
                                LX_RESULT_STATE_DENIED,
                                LX_DETAIL_CAMERA_MODE);
        s_pending_request_valid = 0u;
        send_camera_idle_and_invalidate();
    }
}

static void finish_mission(void)
{
    /* Never publish IDLE while the external flight controller is still armed. */
    if (fc_sta.unlock_sta != 0u)
    {
        return;
    }
    s_active = 0u;
    my_task_flag = 0;
    s_stage = MISSION_STAGE_IDLE;
    mission_step = MISSION_STAGE_IDLE;
    send_camera_idle_and_invalidate();
    report_mission_event(ESP_EVENT_STAGE_IDLE);
    send_status(MISSION_STAGE_IDLE);
    mode_select(0);
}

static void request_abort(uint8_t error_code, uint8_t hard_land)
{
    if (s_active == 0u)
    {
        return;
    }
    if (s_stage == MISSION_STAGE_ABORT)
    {
        if (hard_land != 0u)
        {
            s_abort_hard_land = 1u;
        }
        return;
    }

    s_error_code = error_code;
    report_mission_trace(MISSION_TRACE_ABORT_REQUEST);
    if (s_abort_reason_reported == 0u)
    {
        report_mission_event(abort_event_code(error_code));
        s_abort_reason_reported = 1u;
    }
    s_abort_hard_land = hard_land;
    s_abort_return_started = 0u;
    s_land_command_queued = 0u;
    get_air_xy(&s_abort_hold_x_cm, &s_abort_hold_y_cm);
    s_range_target_cm = clamp_i32(hight, LANDING_TERMINAL_RANGE_CM, GROUND_CRUISE_RANGE_CM);
    send_camera_idle_and_invalidate();
    set_stage(MISSION_STAGE_ABORT);
}

static void handle_maintenance_reset(void)
{
    lx_maintenance_reset_t reset;
    uint8_t pending_request;

    if (MY_uart_esp_take_maintenance_reset(&reset) == 0u)
    {
        return;
    }

    (void)reset;
    pending_request = s_pending_request_valid;
    s_pending_request_valid = 0u;
    s_response_cache.valid = 0u;
    s_response_cache.task_type = 0u;
    s_response_cache.mission_id = 0u;
    s_response_cache.result = 0u;
    s_response_cache.detail = 0u;
    s_response_cache.cached_ms = 0u;
    s_calibration_id = 0u;
    s_visual_filter_initialized = 0u;

    if (s_active != 0u)
    {
        request_abort(ERROR_CALIBRATION, 0u);
    }
    else if (pending_request != 0u)
    {
        /* A MODE request may already be in flight while the task is pending. */
        send_camera_idle_and_invalidate();
    }
    else
    {
        MY_uart_camera_invalidate_session();
        s_camera_idle_pending = 0u;
        s_camera_idle_tx_attempts = 0u;
    }
}

static void handle_mission_abort(void)
{
    uint8_t cancelled_pending_request = 0u;

    if (MY_uart_esp_take_mission_abort() == 0u)
    {
        return;
    }

    if (s_pending_request_valid != 0u)
    {
        /* Close a task-request handshake that lost the race with the abort. */
        cache_and_send_response(&s_pending_request,
                                LX_RESULT_STATE_DENIED,
                                LX_DETAIL_NONE);
        s_pending_request_valid = 0u;
        cancelled_pending_request = 1u;
    }

    if (s_active != 0u)
    {
        /* Healthy navigation returns to H; existing sensor failsafes may
         * escalate the same ABORT state to a direct landing. */
        request_abort(ERROR_REMOTE_ABORT, 0u);
    }
    else if (cancelled_pending_request != 0u)
    {
        send_camera_idle_and_invalidate();
    }

    /* MISSION_ABORT has no wireless MissionId field in V2.3. Internal
     * MISSION_RESPONSE therefore uses the reserved correlation value zero. */
    my_send_esp_mission_response(0x84u, 0u,
                                 LX_RESULT_ACCEPTED, LX_DETAIL_NONE);
}

static void update_input_snapshots(void)
{
    (void)MY_uart_esp_get_car_pose(&s_pose);
    (void)MY_uart_camera_get_target(&s_camera_target);
    (void)MY_uart_camera_get_action_result(&s_camera_result);

    if (s_active != 0u && s_pose.rx_counter != 0u &&
        s_pose.rx_counter != s_last_processed_pose_counter)
    {
        s_last_processed_pose_counter = s_pose.rx_counter;
        if (s_pose.coordinate_frame != CAR_FRAME_FIELD_GLOBAL ||
            (s_pose.pose_flags & CAR_POSE_CALIBRATED) == 0u ||
            s_pose.calibration_id == 0u ||
            s_pose.calibration_id != s_calibration_id)
        {
            request_abort(ERROR_CALIBRATION, 0u);
        }
    }

}

static uint8_t update_precheck(uint32_t now)
{
    uint16_t diagnostic = 0u;

    if (mission_command_ch6_allows_start((rc_in.no_signal == 0u && rc_in.fail_safe == 0u) ? 1u : 0u,
                                         rc_in.rc_ch.st_data.ch_[ch_6_aux2]) == 0u)
    {
        diagnostic = ESP_DIAG_RC_OR_CH6;
    }
    /* The request itself was admitted with the stricter 250 ms snapshot.
     * During the MODE-ACK/clearance delay, use the established 500 ms loss
     * threshold so a single missed 10 Hz LoRa period cannot abort PRECHECK. */
    else if (car_pose_fresh(now, CAR_POSE_INVALID_MS) == 0u ||
             s_pose.valid_streak < 3u)
    {
        diagnostic = ESP_DIAG_CAR_POSE;
    }
    else if (s_pose.calibration_id != s_calibration_id)
    {
        diagnostic = ESP_DIAG_CALIBRATION;
    }
    else if (MY_uart_camera_session_ready(s_task_type,
                                           s_mission_id,
                                           s_camera_mode_seq) == 0u)
    {
        diagnostic = ESP_DIAG_CAMERA_MODE;
    }

    if (diagnostic != 0u)
    {
        report_precheck_diagnostic(diagnostic);
        request_abort(ERROR_PRECHECK, 1u);
        return 0u;
    }
    report_precheck_diagnostic(0u);

    if (s_stage_substate == 0u)
    {
        if ((uint32_t)(now - s_stage_enter_ms) < START_WARNING_DELAY_MS)
        {
            return 0u;
        }
        s_stage_substate = 1u;
        s_substate_enter_ms = now;
    }

    if (s_stage_substate == 1u)
    {
        if (LX_Change_Mode(2u) != 0u)
        {
            s_stage_substate = 2u;
            s_substate_enter_ms = now;
        }
        else if ((uint32_t)(now - s_substate_enter_ms) > PRECHECK_COMMAND_TIMEOUT_MS)
        {
            report_precheck_diagnostic(ESP_DIAG_FC_MODE);
            request_abort(ERROR_TIMEOUT, 1u);
        }
    }
    else if (s_stage_substate == 2u)
    {
        if (fc_sta.fc_mode_sta == 2u)
        {
            s_stage_substate = 3u;
            s_substate_enter_ms = now;
            report_mission_event(ESP_EVENT_FC_MODE_2_CONFIRMED);
        }
        else if ((uint32_t)(now - s_substate_enter_ms) > PRECHECK_COMMAND_TIMEOUT_MS)
        {
            report_precheck_diagnostic(ESP_DIAG_FC_MODE);
            request_abort(ERROR_TIMEOUT, 1u);
        }
    }
    else if (s_stage_substate == 3u)
    {
        if (FC_Unlock() != 0u)
        {
            s_stage_substate = 4u;
            s_substate_enter_ms = now;
        }
        else if ((uint32_t)(now - s_substate_enter_ms) > PRECHECK_COMMAND_TIMEOUT_MS)
        {
            report_precheck_diagnostic(ESP_DIAG_FC_UNLOCK);
            request_abort(ERROR_TIMEOUT, 1u);
        }
    }
    else
    {
        if (fc_sta.unlock_sta != 0u)
        {
            mode_select(4);
            s_range_target_cm = GROUND_CRUISE_RANGE_CM;
            report_mission_event(ESP_EVENT_FC_UNLOCK_CONFIRMED);
            set_stage(MISSION_STAGE_TAKEOFF);
            return 1u;
        }
        if ((uint32_t)(now - s_substate_enter_ms) > PRECHECK_COMMAND_TIMEOUT_MS)
        {
            report_precheck_diagnostic(ESP_DIAG_FC_UNLOCK);
            request_abort(ERROR_TIMEOUT, 1u);
        }
    }
    return 0u;
}

static uint8_t car_tracking_allowed(uint32_t now)
{
    if (car_pose_fresh(now, CAR_POSE_INVALID_MS) == 0u ||
        s_pose.calibration_id != s_calibration_id)
    {
        request_abort(ERROR_CAR_LOST, 0u);
        return 0u;
    }
    return 1u;
}

static void update_abort(uint32_t now)
{
    int32_t air_x;
    int32_t air_y;
    int32_t desired_range;

    if (s_abort_hard_land != 0u)
    {
        if (s_land_command_queued == 0u)
        {
            s_land_command_queued = OneKey_Land();
            if (s_land_command_queued != 0u)
            {
                report_mission_event(ESP_EVENT_FC_LAND_COMMAND_ACCEPTED);
                report_mission_trace(MISSION_TRACE_ABORT_LAND_ACCEPTED);
                mission_command_stop_accepted(&s_command_state);
                mode_select(0);
                s_substate_enter_ms = now;
            }
        }
        if (fc_sta.unlock_sta == 0u)
        {
            finish_mission();
        }
        else if (s_land_command_queued != 0u &&
                 (uint32_t)(now - s_substate_enter_ms) > LAND_COMMAND_TIMEOUT_MS)
        {
            /* Command acceptance is not lock confirmation; retry without leaving ABORT. */
            s_land_command_queued = 0u;
            s_substate_enter_ms = now;
        }
        return;
    }

    mode_select(4);
    get_air_xy(&air_x, &air_y);
    desired_range = (abs_i32(air_x - s_last_car_x_cm) <= PLATFORM_NEAR_RADIUS_CM &&
                     abs_i32(air_y - s_last_car_y_cm) <= PLATFORM_NEAR_RADIUS_CM)
                        ? PLATFORM_CRUISE_RANGE_CM
                        : GROUND_CRUISE_RANGE_CM;
    s_range_target_cm = desired_range;

    if (s_abort_return_started == 0u)
    {
        set_radar_target(s_abort_hold_x_cm, s_abort_hold_y_cm,
                         s_range_target_cm, TERMINAL_SPEED_CMPS);
        if (height_arrived(desired_range) != 0u)
        {
            s_abort_return_started = 1u;
            report_mission_trace(MISSION_TRACE_ABORT_RETURN_START);
        }
    }
    else
    {
        set_radar_target(s_home_x_cm, s_home_y_cm,
                         s_range_target_cm, RETURN_SPEED_CMPS);
        if (position_arrived(s_home_x_cm, s_home_y_cm, HOME_ARRIVE_CM) != 0u &&
            height_arrived(desired_range) != 0u)
        {
            s_abort_hard_land = 1u;
            s_land_command_queued = 0u;
            report_mission_trace(MISSION_TRACE_ABORT_HOME_REACHED);
        }
    }
    if ((uint32_t)(now - s_stage_enter_ms) > RETURN_HOME_TIMEOUT_MS)
    {
        s_abort_hard_land = 1u;
        s_land_command_queued = 0u;
        s_substate_enter_ms = now;
        report_mission_trace(MISSION_TRACE_ABORT_RETURN_TIMEOUT);
    }
}

void UserTask_OneKeyCmd(void)
{
    uint32_t now = GetSysRunTimeMs();
    uint8_t legacy_requests;
    uint8_t land_accepted;
    mission_command_output_t command;
    lx_mission_request_t request;
    int32_t target_x;
    int32_t target_y;
    int32_t air_x;
    int32_t air_y;
    int32_t desired_range;
    uint8_t aligned;
    uint8_t action_ack_state;

    if (s_initialized == 0u)
    {
        mission_command_init(&s_command_state);
        set_dis_field_global();
        mission_step = MISSION_STAGE_IDLE;
        s_stage = MISSION_STAGE_IDLE;
        report_mission_event(ESP_EVENT_STAGE_IDLE);
        MY_uart_camera_invalidate_session();
        s_initialized = 1u;
    }

    update_input_snapshots();
    handle_maintenance_reset();
    service_camera_idle_handshake(now);
    legacy_requests = MY_uart_esp_take_mission_requests();
    mission_command_update(&s_command_state,
                           (rc_in.no_signal == 0u && rc_in.fail_safe == 0u) ? 1u : 0u,
                           rc_in.rc_ch.st_data.ch_[ch_6_aux2],
                           0u, /* V2.2: only TaskType+MissionId may select/start a task. */
                           (legacy_requests & ESP_MISSION_REQUEST_STOP) ? 1u : 0u,
                           &command);

    if (MY_uart_esp_take_mission_request(&request) != 0u)
    {
        handle_request(&request);
    }
    process_pending_request();
    handle_mission_abort();

    if (command.stop != 0u)
    {
        if (s_pending_request_valid != 0u)
        {
            send_camera_idle_and_invalidate();
        }
        s_pending_request_valid = 0u;
        if (s_active != 0u)
        {
            request_abort(ERROR_CH6_OR_RC, 1u);
        }
        else
        {
            land_accepted = OneKey_Land();
            if (land_accepted != 0u)
            {
                mission_command_stop_accepted(&s_command_state);
            }
        }
    }

    if (s_active == 0u)
    {
        return;
    }

    if (s_stage == MISSION_STAGE_ABORT)
    {
        update_abort(now);
        return;
    }

    switch (s_stage)
    {
    case MISSION_STAGE_PRECHECK:
        (void)update_precheck(now);
        break;

    case MISSION_STAGE_TAKEOFF:
        if (car_tracking_allowed(now) == 0u)
        {
            break;
        }
        car_target(now, 0u, &target_x, &target_y);
        yaw_set_hold_target(car_yaw_deg());
        set_radar_target(s_home_x_cm, s_home_y_cm,
                         GROUND_CRUISE_RANGE_CM, TERMINAL_SPEED_CMPS);
        if (stable_for((uint8_t)(position_arrived(s_home_x_cm, s_home_y_cm, INTERCEPT_ARRIVE_CM) &&
                                 height_arrived(GROUND_CRUISE_RANGE_CM) &&
                                 yaw_arrived(YAW_ARRIVE_DEG)),
                       TAKEOFF_STABLE_MS) != 0u)
        {
            set_stage(MISSION_STAGE_INTERCEPT);
        }
        else if ((uint32_t)(now - s_stage_enter_ms) > TAKEOFF_TIMEOUT_MS)
        {
            request_abort(ERROR_TIMEOUT, 1u);
        }
        break;

    case MISSION_STAGE_INTERCEPT:
        if (car_tracking_allowed(now) == 0u)
        {
            break;
        }
        car_target(now, 0u, &target_x, &target_y);
        get_air_xy(&air_x, &air_y);
        desired_range = (abs_i32(air_x - target_x) <= PLATFORM_NEAR_RADIUS_CM &&
                         abs_i32(air_y - target_y) <= PLATFORM_NEAR_RADIUS_CM)
                            ? PLATFORM_CRUISE_RANGE_CM
                            : GROUND_CRUISE_RANGE_CM;
        s_range_target_cm = desired_range;
        yaw_set_hold_target(car_yaw_deg());
        set_radar_target(target_x, target_y, s_range_target_cm, INTERCEPT_SPEED_CMPS);
        if (stable_for((uint8_t)(position_arrived(target_x, target_y, INTERCEPT_ARRIVE_CM) &&
                                 height_arrived(desired_range) && yaw_arrived(YAW_ARRIVE_DEG)),
                       INTERCEPT_STABLE_MS) != 0u)
        {
            set_stage(MISSION_STAGE_FOLLOW);
        }
        else if ((uint32_t)(now - s_stage_enter_ms) > INTERCEPT_TIMEOUT_MS)
        {
            request_abort(ERROR_TIMEOUT, 0u);
        }
        break;

    case MISSION_STAGE_FOLLOW:
        if (car_tracking_allowed(now) == 0u)
        {
            break;
        }
        car_target(now, 0u, &target_x, &target_y);
        s_range_target_cm = PLATFORM_CRUISE_RANGE_CM;
        yaw_set_hold_target(car_yaw_deg());
        set_radar_target(target_x, target_y, s_range_target_cm, FOLLOW_SPEED_CMPS);
        if (stable_for((uint8_t)(position_arrived(target_x, target_y, FOLLOW_ARRIVE_CM) &&
                                 height_arrived(PLATFORM_CRUISE_RANGE_CM) &&
                                 yaw_arrived(YAW_ARRIVE_DEG)),
                       FOLLOW_STABLE_MS) != 0u)
        {
            set_stage((s_task_type == LX_TASK_DROP)
                          ? MISSION_STAGE_DROP_ALIGN
                          : MISSION_STAGE_LAND_ALIGN);
        }
        else if ((uint32_t)(now - s_stage_enter_ms) > FOLLOW_TIMEOUT_MS)
        {
            request_abort(ERROR_TIMEOUT, 0u);
        }
        break;

    case MISSION_STAGE_DROP_ALIGN:
    case MISSION_STAGE_LAND_ALIGN:
        if (car_tracking_allowed(now) == 0u)
        {
            break;
        }
        car_target(now, 1u, &target_x, &target_y);
        if (s_stage == MISSION_STAGE_DROP_ALIGN)
        {
            apply_drop_release_body_x_advance(&target_x, &target_y);
        }
        desired_range = (s_stage == MISSION_STAGE_DROP_ALIGN)
                            ? DROP_PLATFORM_RANGE_CM
                            : PLATFORM_CRUISE_RANGE_CM;
        s_range_target_cm = desired_range;
        yaw_set_hold_target(car_yaw_deg());
        set_radar_target(target_x, target_y, s_range_target_cm, TERMINAL_SPEED_CMPS);
        if (s_stage == MISSION_STAGE_DROP_ALIGN)
        {
            /* Keep yaw tracking active, but do not make a vertical release wait for it. */
            s_terminal_align_blockers = terminal_alignment_blockers(
                now, target_x, target_y, desired_range,
                DROP_TERMINAL_POS_ARRIVE_CM, DROP_HEIGHT_ARRIVE_CM, 1u, 0u);
        }
        else
        {
            s_terminal_align_blockers = terminal_alignment_blockers(
                now, target_x, target_y, desired_range,
                TERMINAL_POS_ARRIVE_CM, HEIGHT_ARRIVE_CM, 1u, 1u);
        }
        aligned = (s_terminal_align_blockers == 0u) ? 1u : 0u;
        if (stable_for(aligned,
                       (s_stage == MISSION_STAGE_DROP_ALIGN)
                           ? DROP_ALIGN_STABLE_MS
                           : LAND_ALIGN_STABLE_MS) != 0u)
        {
            if (s_stage == MISSION_STAGE_DROP_ALIGN)
            {
                set_stage(MISSION_STAGE_DROP_ACTION);
            }
            else
            {
                s_range_target_cm = PLATFORM_CRUISE_RANGE_CM;
                set_stage(MISSION_STAGE_DESCEND);
            }
        }
        else if ((uint32_t)(now - s_stage_enter_ms) > ALIGN_TIMEOUT_MS)
        {
            request_abort(camera_link_fresh(now) ? ERROR_TIMEOUT : ERROR_CAMERA_LOST, 0u);
        }
        break;

    case MISSION_STAGE_DROP_ACTION:
        if (car_tracking_allowed(now) == 0u)
        {
            break;
        }
        car_target(now, 1u, &target_x, &target_y);
        apply_drop_release_body_x_advance(&target_x, &target_y);
        yaw_set_hold_target(car_yaw_deg());
        set_radar_target(target_x, target_y, DROP_PLATFORM_RANGE_CM, TERMINAL_SPEED_CMPS);
        if (s_stage_substate == 0u)
        {
            s_action_id = s_mission_id;
            s_action_result_start_counter = s_camera_result.rx_counter;
            begin_camera_action();
            s_stage_substate = 1u;
        }
        action_ack_state = service_camera_action_ack(now);
        if (action_ack_state == 2u)
        {
            request_abort(ERROR_CAMERA_ACTION, 0u);
            break;
        }
        if (action_ack_state == 1u &&
            s_camera_result.rx_counter != s_action_result_start_counter &&
            s_camera_result.action == CAMERA_ACTION_DROP &&
            s_camera_result.action_id == s_action_id &&
            s_camera_result.task_type == s_task_type &&
            s_camera_result.mission_id == s_mission_id &&
            s_camera_result.mode_seq == s_camera_mode_seq &&
            (int32_t)(s_camera_result.local_rx_ms - s_action_ack_rx_ms) >= 0)
        {
            s_action_result_start_counter = s_camera_result.rx_counter;
            if (s_camera_result.result == 0u)
            {
                report_mission_event(ESP_EVENT_CAMERA_DROP_COMPLETED);
                send_camera_idle_and_invalidate();
                set_stage(MISSION_STAGE_RETURN_HOME);
            }
            else if (s_camera_result.result == 2u || s_camera_result.result == 3u)
            {
                request_abort(ERROR_CAMERA_ACTION, 0u);
            }
        }
        if ((uint32_t)(now - s_stage_enter_ms) > DROP_ACTION_TIMEOUT_MS)
        {
            request_abort(ERROR_CAMERA_ACTION, 0u);
        }
        break;

    case MISSION_STAGE_DESCEND:
        if (car_tracking_allowed(now) == 0u)
        {
            break;
        }
        car_target(now, 1u, &target_x, &target_y);
        yaw_set_hold_target(car_yaw_deg());
        s_terminal_align_blockers = terminal_alignment_blockers(
            now, target_x, target_y, s_range_target_cm,
            TERMINAL_POS_ARRIVE_CM, HEIGHT_ARRIVE_CM, 0u, 1u);
        aligned = (s_terminal_align_blockers == 0u) ? 1u : 0u;

        if ((uint32_t)(now - s_pose.local_rx_ms) > CAR_POSE_WARNING_MS)
        {
            /* During descent, the warning threshold already inhibits vertical motion. */
            s_descent_inhibited = 1u;
            s_range_target_cm = clamp_i32(hight,
                                          LANDING_TERMINAL_RANGE_CM,
                                          PLATFORM_CRUISE_RANGE_CM);
            s_descent_step_ms = 0u;
        }
        else if (camera_link_fresh_within(now, CAMERA_DESCEND_STOP_MS) == 0u)
        {
            s_descent_inhibited = 1u;
            s_range_target_cm = clamp_i32(hight,
                                          LANDING_TERMINAL_RANGE_CM,
                                          PLATFORM_CRUISE_RANGE_CM);
            s_descent_step_ms = 0u;
            if (camera_link_fresh(now) == 0u)
            {
                set_stage(MISSION_STAGE_LAND_ALIGN);
            }
        }
        else
        {
            s_descent_inhibited = 0u;
            if (aligned != 0u)
            {
                s_descent_step_ms = (uint16_t)(s_descent_step_ms + TASK_PERIOD_MS);
                if (s_descent_step_ms >= DESCENT_STEP_INTERVAL_MS)
                {
                    s_descent_step_ms = 0u;
                    if (s_range_target_cm > LANDING_TERMINAL_RANGE_CM)
                    {
                        s_range_target_cm--;
                    }
                }
            }
            else
            {
                s_descent_step_ms = 0u;
            }
        }
        set_radar_target(target_x, target_y, s_range_target_cm, TERMINAL_SPEED_CMPS);
        if (stable_for((uint8_t)(aligned &&
                                 hight <= (LANDING_TERMINAL_RANGE_CM + 2)),
                       LAND_CONTACT_STABLE_MS) != 0u)
        {
            /* Keep the controller armed on the platform for the second takeoff. */
            s_range_target_cm = PLATFORM_CONTACT_HOLD_RANGE_CM;
            set_radar_target(target_x, target_y, s_range_target_cm, TERMINAL_SPEED_CMPS);
            set_stage(MISSION_STAGE_ON_PLATFORM_5S);
        }
        if (s_stage == MISSION_STAGE_DESCEND &&
            (uint32_t)(now - s_stage_enter_ms) > DESCENT_TIMEOUT_MS)
        {
            request_abort(ERROR_TIMEOUT, 0u);
        }
        break;

    case MISSION_STAGE_ON_PLATFORM_5S:
        if (car_pose_fresh(now, CAR_POSE_INVALID_MS) == 0u ||
            camera_link_fresh(now) == 0u)
        {
            request_abort(ERROR_CAR_LOST, 1u);
            break;
        }
        if (fc_sta.unlock_sta == 0u)
        {
            request_abort(ERROR_TIMEOUT, 1u);
            break;
        }
        if (stable_for((uint8_t)(hight <= (LANDING_TERMINAL_RANGE_CM + 5) &&
                                 fc_sta.unlock_sta != 0u),
                       PLATFORM_HOLD_MS) != 0u)
        {
            set_stage(MISSION_STAGE_PLATFORM_TAKEOFF);
        }
        break;

    case MISSION_STAGE_PLATFORM_TAKEOFF:
        if (car_tracking_allowed(now) == 0u)
        {
            break;
        }
        if (s_stage_substate == 0u)
        {
            if (fc_sta.unlock_sta == 0u)
            {
                request_abort(ERROR_TIMEOUT, 1u);
                break;
            }
            mode_select(4);
            s_stage_substate = 2u;
            s_range_target_cm = PLATFORM_CRUISE_RANGE_CM;
        }
        if (s_stage_substate == 2u)
        {
            car_target(now, 0u, &target_x, &target_y);
            yaw_set_hold_target(car_yaw_deg());
            set_radar_target(target_x, target_y, PLATFORM_CRUISE_RANGE_CM, FOLLOW_SPEED_CMPS);
            if (stable_for((uint8_t)(position_arrived(target_x, target_y, FOLLOW_ARRIVE_CM) &&
                                     height_arrived(PLATFORM_CRUISE_RANGE_CM) &&
                                     yaw_arrived(YAW_ARRIVE_DEG)),
                           PLATFORM_TAKEOFF_STABLE_MS) != 0u)
            {
                send_camera_idle_and_invalidate();
                set_stage(MISSION_STAGE_RETURN_HOME);
            }
        }
        if ((uint32_t)(now - s_stage_enter_ms) > LAND_COMMAND_TIMEOUT_MS)
        {
            request_abort(ERROR_TIMEOUT, 1u);
        }
        break;

    case MISSION_STAGE_RETURN_HOME:
        mode_select(4);
        get_air_xy(&air_x, &air_y);
        desired_range = (abs_i32(air_x - s_last_car_x_cm) <= PLATFORM_NEAR_RADIUS_CM &&
                         abs_i32(air_y - s_last_car_y_cm) <= PLATFORM_NEAR_RADIUS_CM)
                            ? PLATFORM_CRUISE_RANGE_CM
                            : GROUND_CRUISE_RANGE_CM;
        s_range_target_cm = desired_range;
        set_radar_target(s_home_x_cm, s_home_y_cm, s_range_target_cm, RETURN_SPEED_CMPS);
        if (stable_for((uint8_t)(position_arrived(s_home_x_cm, s_home_y_cm, HOME_ARRIVE_CM) &&
                                 height_arrived(desired_range)),
                       HOME_STABLE_MS) != 0u)
        {
            set_stage(MISSION_STAGE_HOME_LAND);
        }
        else if ((uint32_t)(now - s_stage_enter_ms) > RETURN_HOME_TIMEOUT_MS)
        {
            request_abort(ERROR_TIMEOUT, 1u);
        }
        break;

    case MISSION_STAGE_HOME_LAND:
        if (s_land_command_queued == 0u)
        {
            s_land_command_queued = OneKey_Land();
            if (s_land_command_queued != 0u)
            {
                report_mission_event(ESP_EVENT_FC_LAND_COMMAND_ACCEPTED);
                mode_select(0);
                s_substate_enter_ms = now;
            }
        }
        if (fc_sta.unlock_sta == 0u)
        {
            finish_mission();
        }
        else if (s_land_command_queued != 0u &&
                 (uint32_t)(now - s_substate_enter_ms) > LAND_COMMAND_TIMEOUT_MS)
        {
            s_land_command_queued = 0u;
            s_substate_enter_ms = now;
            if (s_error_code != ERROR_TIMEOUT)
            {
                s_error_code = ERROR_TIMEOUT;
                send_status(MISSION_STAGE_HOME_LAND);
            }
        }
        break;

    default:
        request_abort(ERROR_TIMEOUT, 0u);
        break;
    }
}
