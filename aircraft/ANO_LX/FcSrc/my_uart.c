#include "my_uart.h"
#include "my_send_test.h"

#ifndef UNIT_TEST
#include "Drv_Uart.h"
#include "Drv_Sys.h"
#include "my_get_data.h"
#else
extern uint32_t GetSysRunTimeMs(void);
extern void DrvUart2SendBuf(unsigned char *data, uint8_t len);
extern void DrvUart3SendBuf(unsigned char *data, uint8_t len);
extern int32_t dis_x_slam, dis_y_slam, yaw_slam;
static uint32_t test_primask;
#define __get_PRIMASK() (test_primask)
#define __disable_irq() ((void)0)
#define __set_PRIMASK(value) ((void)(value))
#endif

#define ESP_FRAME_HEAD 0xBBu
#define ESP_FRAME_ADDR 0x10u
#define ESP_FRAME_FIXED 0xF1u
#define ESP_FRAME_END 0xEEu
#define ESP_EXT_MARKER 0x30u
#define ESP_FUNC_MISSION_CMD 0x10u
#define ESP_RX_TIMEOUT_MS 100u
#define ESP_MAX_FRAME_LEN 73u

#define V21_HEAD_0 0xAAu
#define V21_HEAD_1 0x55u
#define V21_VERSION 0x02u
#define V21_ADDR_FC 0x21u
#define V21_ADDR_CAMERA 0x50u
#define V21_MAX_PAYLOAD 64u
#define V21_MAX_FRAME_LEN 75u
#define V21_RX_TIMEOUT_MS 100u

#define RADAR_FRAME_LEN 14u
#define RADAR_FRAME_HEAD 0xAAu
#define RADAR_FRAME_END 0x0Au

#define ENABLE_LEGACY_CAMERA_BENCH 0

int my_task_flag;
uint8_t my_slam_flag;
int16_t OpenMV_data_0, OpenMV_data_1, OpenMV_data_2;

static volatile uint8_t s_esp_mission_request_flags;
static volatile uint8_t s_mission_request_pending;
static volatile lx_mission_request_t s_mission_request;
static volatile uint8_t s_mission_abort_pending;
static volatile uint8_t s_maintenance_reset_pending;
static volatile lx_maintenance_reset_t s_maintenance_reset;
static uint16_t s_last_maintenance_reset_id;
static volatile lx_car_pose_t s_car_pose;
static volatile camera_target_t s_camera_target;
static volatile camera_action_result_t s_camera_action_result;
static volatile uint32_t s_camera_last_rx_ms;
static volatile uint32_t s_mid360_last_rx_ms;
static volatile uint32_t s_laser_last_rx_ms;
static volatile uint8_t s_laser_valid;

typedef struct
{
    uint8_t valid;
    uint8_t acknowledged;
    uint8_t task_type;
    uint16_t mission_id;
    uint8_t mode_seq;
    uint32_t ack_rx_ms;
    uint8_t action_pending;
    uint8_t action_acknowledged;
    uint16_t action_id;
    uint8_t action_seq;
    uint32_t action_ack_rx_ms;
} camera_session_t;

static volatile camera_session_t s_camera_session;

typedef struct
{
    uint8_t active;
    uint8_t received;
    uint8_t request_type;
    uint8_t request_seq;
    camera_ack_t ack;
} camera_expected_ack_t;

static volatile camera_expected_ack_t s_camera_expected_ack;

static uint16_t read_be16(const uint8_t *src)
{
    return (uint16_t)(((uint16_t)src[0] << 8) | src[1]);
}

static uint32_t read_be32(const uint8_t *src)
{
    return ((uint32_t)src[0] << 24) |
           ((uint32_t)src[1] << 16) |
           ((uint32_t)src[2] << 8) |
           (uint32_t)src[3];
}

static uint8_t source_time_is_newer(uint32_t newer, uint32_t older)
{
    return ((int32_t)(newer - older) > 0) ? 1u : 0u;
}

static uint8_t car_pose_semantically_usable(const lx_car_pose_t *pose)
{
    uint8_t required = CAR_POSE_POSITION_VALID |
                       CAR_POSE_CALIBRATED |
                       CAR_POSE_YAW_VALID;

    return (pose->coordinate_frame == CAR_FRAME_FIELD_GLOBAL &&
            (pose->pose_flags & required) == required &&
            pose->calibration_id != 0u)
               ? 1u
               : 0u;
}

static void invalidate_car_pose(uint32_t now)
{
    lx_car_pose_t pose = s_car_pose;

    pose.coordinate_frame = 0u;
    pose.pose_flags = 0u;
    pose.calibration_id = 0u;
    pose.x_cm = 0;
    pose.y_cm = 0;
    pose.yaw_0p1deg = 0;
    pose.vx_cmps = 0;
    pose.vy_cmps = 0;
    /* Accept the first SourceTimeMs after the Pi starts a new uptime epoch. */
    pose.source_time_ms = 0xFFFFFFFFu;
    pose.local_rx_ms = now;
    pose.rx_counter++;
    pose.valid_streak = 0u;
    pose.usable = 0u;
    s_car_pose = pose;
}

uint8_t MY_uart_esp_take_mission_requests(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t requests;

    __disable_irq();
    requests = s_esp_mission_request_flags;
    s_esp_mission_request_flags = 0u;
    __set_PRIMASK(primask);
    return requests;
}

uint8_t MY_uart_esp_take_mission_request(lx_mission_request_t *request)
{
    uint32_t primask;

    if (request == 0)
    {
        return 0u;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (s_mission_request_pending == 0u)
    {
        __set_PRIMASK(primask);
        return 0u;
    }
    *request = s_mission_request;
    s_mission_request_pending = 0u;
    __set_PRIMASK(primask);
    return 1u;
}

uint8_t MY_uart_esp_take_mission_abort(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t pending;

    __disable_irq();
    pending = s_mission_abort_pending;
    s_mission_abort_pending = 0u;
    __set_PRIMASK(primask);
    return pending;
}

uint8_t MY_uart_esp_take_maintenance_reset(lx_maintenance_reset_t *reset)
{
    uint32_t primask;

    if (reset == 0)
    {
        return 0u;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (s_maintenance_reset_pending == 0u)
    {
        __set_PRIMASK(primask);
        return 0u;
    }
    *reset = s_maintenance_reset;
    s_maintenance_reset_pending = 0u;
    __set_PRIMASK(primask);
    return 1u;
}

uint8_t MY_uart_esp_get_car_pose(lx_car_pose_t *pose)
{
    uint32_t primask;

    if (pose == 0)
    {
        return 0u;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *pose = s_car_pose;
    __set_PRIMASK(primask);
    return (pose->rx_counter != 0u) ? 1u : 0u;
}

uint8_t MY_uart_camera_get_target(camera_target_t *target)
{
    uint32_t primask;

    if (target == 0)
    {
        return 0u;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *target = s_camera_target;
    __set_PRIMASK(primask);
    return (target->rx_counter != 0u) ? 1u : 0u;
}

uint8_t MY_uart_camera_get_action_result(camera_action_result_t *result)
{
    uint32_t primask;

    if (result == 0)
    {
        return 0u;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *result = s_camera_action_result;
    __set_PRIMASK(primask);
    return (result->rx_counter != 0u) ? 1u : 0u;
}

void MY_uart_camera_expect_ack(uint8_t request_type, uint8_t request_seq)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_camera_expected_ack.active = 1u;
    s_camera_expected_ack.received = 0u;
    s_camera_expected_ack.request_type = request_type;
    s_camera_expected_ack.request_seq = request_seq;
    s_camera_expected_ack.ack.request_type = 0u;
    s_camera_expected_ack.ack.request_seq = 0u;
    s_camera_expected_ack.ack.result = 0u;
    s_camera_expected_ack.ack.detail = 0u;
    s_camera_expected_ack.ack.local_rx_ms = 0u;
    s_camera_expected_ack.ack.rx_counter = 0u;
    __set_PRIMASK(primask);
}

void MY_uart_camera_cancel_expected_ack(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_camera_expected_ack.active = 0u;
    s_camera_expected_ack.received = 0u;
    s_camera_expected_ack.request_type = 0u;
    s_camera_expected_ack.request_seq = 0u;
    __set_PRIMASK(primask);
}

uint8_t MY_uart_camera_get_expected_ack(uint8_t request_type,
                                        uint8_t request_seq,
                                        camera_ack_t *ack)
{
    uint32_t primask;
    uint8_t received;

    if (ack == 0)
    {
        return 0u;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    received = (s_camera_expected_ack.active != 0u &&
                s_camera_expected_ack.received != 0u &&
                s_camera_expected_ack.request_type == request_type &&
                s_camera_expected_ack.request_seq == request_seq)
                   ? 1u
                   : 0u;
    if (received != 0u)
    {
        *ack = s_camera_expected_ack.ack;
    }
    __set_PRIMASK(primask);
    return received;
}

uint32_t MY_uart_camera_last_rx_ms(void)
{
    return s_camera_last_rx_ms;
}

uint32_t MY_uart_mid360_last_rx_ms(void)
{
    return s_mid360_last_rx_ms;
}

void MY_uart_mark_laser_update(int32_t range_cm, uint8_t sensor_state)
{
    (void)sensor_state;
    s_laser_last_rx_ms = GetSysRunTimeMs();
    s_laser_valid = (range_cm > 0 && range_cm <= 600) ? 1u : 0u;
}

uint32_t MY_uart_laser_last_rx_ms(void)
{
    return s_laser_last_rx_ms;
}

uint8_t MY_uart_laser_is_valid(void)
{
    return s_laser_valid;
}

static uint8_t lx_checksum_valid(const uint8_t *data, uint8_t len)
{
    uint8_t sc = 0;
    uint8_t ac = 0;
    uint8_t i;

    if (len < 3u)
    {
        return 0u;
    }
    for (i = 0; i < (uint8_t)(len - 2u); i++)
    {
        sc = (uint8_t)(sc + data[i]);
        ac = (uint8_t)(ac + sc);
    }
    return (sc == data[len - 2u] && ac == data[len - 1u]) ? 1u : 0u;
}

void MY_uart_esp_receive(uint8_t data)
{
    static uint8_t frame[ESP_MAX_FRAME_LEN];
    static uint8_t index;
    static uint8_t expected_len;
    static uint32_t last_byte_ms;
    uint32_t now = GetSysRunTimeMs();

    if (index != 0u && (uint32_t)(now - last_byte_ms) > ESP_RX_TIMEOUT_MS)
    {
        index = 0u;
        expected_len = 0u;
    }
    last_byte_ms = now;

    if (index == 0u)
    {
        if (data == ESP_FRAME_HEAD)
        {
            frame[index++] = data;
        }
        return;
    }

    if (index == 1u && data != ESP_FRAME_ADDR)
    {
        index = (data == ESP_FRAME_HEAD) ? 1u : 0u;
        frame[0] = data;
        return;
    }
    if (index == 2u && data != ESP_FRAME_FIXED)
    {
        index = (data == ESP_FRAME_HEAD) ? 1u : 0u;
        frame[0] = data;
        return;
    }

    frame[index++] = data;

    if (index == 4u && frame[3] != ESP_EXT_MARKER)
    {
        expected_len = 8u;
    }
    else if (index == 6u && frame[3] == ESP_EXT_MARKER)
    {
        if (frame[5] > 64u)
        {
            index = 0u;
            expected_len = 0u;
            return;
        }
        expected_len = (uint8_t)(9u + frame[5]);
    }

    if (expected_len != 0u && index >= expected_len)
    {
        if (frame[expected_len - 3u] == ESP_FRAME_END &&
            lx_checksum_valid(frame, expected_len))
        {
            if (frame[3] == ESP_EXT_MARKER)
            {
                (void)MY_uart_esp_extended_anl(frame, expected_len);
            }
            else
            {
                (void)MY_uart_esp_anl(frame, expected_len);
            }
        }
        index = 0u;
        expected_len = 0u;
    }
}

int MY_uart_esp_anl(uint8_t *data, uint8_t len)
{
    if (len != 8u || data == 0 ||
        data[0] != ESP_FRAME_HEAD || data[1] != ESP_FRAME_ADDR ||
        data[2] != ESP_FRAME_FIXED || data[5] != ESP_FRAME_END ||
        lx_checksum_valid(data, len) == 0u)
    {
        return -1;
    }

    if (data[3] != ESP_FUNC_MISSION_CMD)
    {
        return -1;
    }
    if (data[4] == 0x00u)
    {
        s_esp_mission_request_flags |= ESP_MISSION_REQUEST_STOP;
        return 0;
    }
    if (data[4] == 0x01u)
    {
        s_esp_mission_request_flags |= ESP_MISSION_REQUEST_START;
        return 1;
    }
    return -1;
}

int MY_uart_esp_extended_anl(const uint8_t *data, uint8_t len)
{
    const uint8_t *payload;
    uint8_t payload_len;
    uint32_t now;

    if (data == 0 || len < 9u ||
        data[0] != ESP_FRAME_HEAD || data[1] != ESP_FRAME_ADDR ||
        data[2] != ESP_FRAME_FIXED || data[3] != ESP_EXT_MARKER ||
        data[len - 3u] != ESP_FRAME_END ||
        data[5] != (uint8_t)(len - 9u) ||
        lx_checksum_valid(data, len) == 0u)
    {
        return -1;
    }

    payload_len = data[5];
    payload = &data[6];
    now = GetSysRunTimeMs();

    if (data[4] == LX_EXT_CAR_POSE)
    {
        lx_car_pose_t pose;
        uint8_t previous_streak = s_car_pose.valid_streak;
        uint16_t previous_calibration = s_car_pose.calibration_id;
        uint32_t previous_source_time = s_car_pose.source_time_ms;
        uint32_t previous_counter = s_car_pose.rx_counter;

        if (payload_len != 22u)
        {
            return -1;
        }

        pose.coordinate_frame = payload[0];
        pose.pose_flags = payload[1];
        pose.calibration_id = read_be16(&payload[2]);
        pose.x_cm = (int32_t)read_be32(&payload[4]);
        pose.y_cm = (int32_t)read_be32(&payload[8]);
        /* The car reports clockwise-positive yaw; the flight controller uses CCW-positive. */
        pose.yaw_0p1deg = (int16_t)(-(int32_t)(int16_t)read_be16(&payload[12]));
        pose.vx_cmps = (int16_t)read_be16(&payload[14]);
        pose.vy_cmps = (int16_t)read_be16(&payload[16]);
        pose.source_time_ms = read_be32(&payload[18]);

        if (previous_counter != 0u &&
            source_time_is_newer(pose.source_time_ms, previous_source_time) == 0u)
        {
            if (pose.source_time_ms != previous_source_time &&
                ((pose.pose_flags & CAR_POSE_CALIBRATED) == 0u ||
                 pose.calibration_id == 0u))
            {
                /* Pi reboot: its new uptime starts lower and calibration is invalid. */
                pose.local_rx_ms = now;
                pose.rx_counter = previous_counter + 1u;
                pose.usable = 0u;
                pose.valid_streak = 0u;
                s_car_pose = pose;
                return 1;
            }
            return 0; /* Upstream duplicate/reorder: do not refresh control state. */
        }

        pose.local_rx_ms = now;
        pose.rx_counter = previous_counter + 1u;
        pose.usable = car_pose_semantically_usable(&pose);
        if (pose.usable != 0u)
        {
            if (previous_streak != 0u && previous_calibration == pose.calibration_id)
            {
                pose.valid_streak = (previous_streak < 255u)
                                        ? (uint8_t)(previous_streak + 1u)
                                        : 255u;
            }
            else
            {
                pose.valid_streak = 1u;
            }
        }
        else
        {
            pose.valid_streak = 0u;
        }
        s_car_pose = pose;
        return 1;
    }

    if (data[4] == LX_EXT_MISSION_REQUEST)
    {
        lx_mission_request_t request;

        if (payload_len != 12u)
        {
            return -1;
        }
        request.task_type = payload[0];
        request.request_flags = payload[1];
        request.mission_id = read_be16(&payload[2]);
        request.calibration_id = read_be16(&payload[4]);
        request.reserved = read_be16(&payload[6]);
        request.source_time_ms = read_be32(&payload[8]);
        request.local_rx_ms = now;
        request.rx_counter = s_mission_request.rx_counter + 1u;
        s_mission_request = request;
        s_mission_request_pending = 1u;
        return 1;
    }

    if (data[4] == LX_EXT_MISSION_ABORT)
    {
        if (payload_len != 0u)
        {
            return -1;
        }
        s_mission_abort_pending = 1u;
        return 1;
    }

    if (data[4] == LX_EXT_MAINTENANCE_RESET)
    {
        lx_maintenance_reset_t reset;

        if (payload_len != 8u)
        {
            return -1;
        }
        reset.reset_id = read_be16(&payload[0]);
        reset.reset_flags = payload[2];
        reset.source_time_ms = read_be32(&payload[4]);
        if (reset.reset_id == 0u ||
            reset.reset_flags != MAINTENANCE_RESET_CLEAR_CALIBRATION ||
            payload[3] != 0u)
        {
            return -1;
        }
        if (reset.reset_id == s_last_maintenance_reset_id)
        {
            return 0; /* The second and third LoRa retransmissions are idempotent. */
        }

        s_last_maintenance_reset_id = reset.reset_id;
        reset.local_rx_ms = now;
        reset.rx_counter = s_maintenance_reset.rx_counter + 1u;
        s_maintenance_reset = reset;
        s_maintenance_reset_pending = 1u;
        invalidate_car_pose(now);
        return 1;
    }

    return -1;
}

void MY_uart_radar_receive(uint8_t data)
{
    static uint8_t frame[RADAR_FRAME_LEN];
    static uint8_t index;
    static uint32_t last_byte_ms;
    uint32_t now = GetSysRunTimeMs();

    if (index != 0u && (uint32_t)(now - last_byte_ms) > V21_RX_TIMEOUT_MS)
    {
        index = 0u;
    }
    last_byte_ms = now;

    if (index == 0u)
    {
        if (data == RADAR_FRAME_HEAD)
        {
            frame[index++] = data;
        }
        return;
    }

    frame[index++] = data;
    if (index >= RADAR_FRAME_LEN)
    {
        if (MY_uart_radar_anl(frame, RADAR_FRAME_LEN) == 0)
        {
            my_slam_flag = 1u;
            s_mid360_last_rx_ms = now;
        }
        else
        {
            my_slam_flag = 0u;
        }
        index = 0u;
    }
}

int MY_uart_radar_anl(uint8_t *data, uint8_t len)
{
    if (data == 0 || len != RADAR_FRAME_LEN ||
        data[0] != RADAR_FRAME_HEAD || data[13] != RADAR_FRAME_END)
    {
        return -1;
    }

    dis_x_slam = (int16_t)read_be16(&data[1]);
    dis_y_slam = (int16_t)read_be16(&data[3]);
    yaw_slam = (int16_t)read_be16(&data[11]);
    return 0;
}

void MY_uart_radar_send(uint8_t data)
{
    uint8_t frame[2] = {data, 0x0Au};
    DrvUart2SendBuf(frame, sizeof(frame));
}

void MY_uart_K230_receive(uint8_t data)
{
    static uint8_t count;
    static uint8_t bytes[5];

    if (count == 0u)
    {
        if (data == 0x0Au)
        {
            count = 1u;
        }
        return;
    }

    bytes[count - 1u] = data;
    count++;
    if (count > 5u)
    {
        OpenMV_data_0 = bytes[0];
        if (OpenMV_data_0 == 1)
        {
            OpenMV_data_1 = (int16_t)(((uint16_t)bytes[2] << 8) | bytes[1]);
            OpenMV_data_2 = (int16_t)(((uint16_t)bytes[4] << 8) | bytes[3]);
            OpenMV_data_1 = (int16_t)(OpenMV_data_1 * 0.75f);
            OpenMV_data_2 = (int16_t)(OpenMV_data_2 * 0.75f);
        }
        else
        {
            OpenMV_data_1 = 0;
            OpenMV_data_2 = 0;
        }
        count = 0u;
    }
}

void MY_uart_K230_anl(uint8_t data)
{
    MY_uart_K230_receive(data);
}

void MY_uart_K230_send(uint8_t data)
{
    uint8_t frame[2] = {data, 0x0Au};
    DrvUart3SendBuf(frame, sizeof(frame));
}

void MY_uart_camera_invalidate_session(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_camera_session.valid = 0u;
    s_camera_session.acknowledged = 0u;
    s_camera_session.task_type = 0u;
    s_camera_session.mission_id = 0u;
    s_camera_session.mode_seq = 0u;
    s_camera_session.ack_rx_ms = 0u;
    s_camera_session.action_pending = 0u;
    s_camera_session.action_acknowledged = 0u;
    s_camera_session.action_id = 0u;
    s_camera_session.action_seq = 0u;
    s_camera_session.action_ack_rx_ms = 0u;

    s_camera_target.flags = 0u;
    s_camera_target.quality = 0u;
    s_camera_target.err_x_cm = 0;
    s_camera_target.err_y_cm = 0;
    s_camera_target.outer_diameter_px = 0u;
    s_camera_target.frame_counter = 0u;
    s_camera_target.source_time_ms = 0u;
    s_camera_target.task_type = 0u;
    s_camera_target.mission_id = 0u;
    s_camera_target.mode_seq = 0u;
    s_camera_target.local_rx_ms = 0u;
    s_camera_target.rx_counter = 0u;

    s_camera_action_result.action = 0u;
    s_camera_action_result.result = 0u;
    s_camera_action_result.action_id = 0u;
    s_camera_action_result.source_time_ms = 0u;
    s_camera_action_result.task_type = 0u;
    s_camera_action_result.mission_id = 0u;
    s_camera_action_result.mode_seq = 0u;
    s_camera_action_result.local_rx_ms = 0u;
    s_camera_action_result.rx_counter = 0u;

    s_camera_expected_ack.active = 0u;
    s_camera_expected_ack.received = 0u;
    s_camera_expected_ack.request_type = 0u;
    s_camera_expected_ack.request_seq = 0u;
    s_camera_expected_ack.ack.request_type = 0u;
    s_camera_expected_ack.ack.request_seq = 0u;
    s_camera_expected_ack.ack.result = 0u;
    s_camera_expected_ack.ack.detail = 0u;
    s_camera_expected_ack.ack.local_rx_ms = 0u;
    s_camera_expected_ack.ack.rx_counter = 0u;

    s_camera_last_rx_ms = 0u;
    __set_PRIMASK(primask);
}

void MY_uart_camera_begin_action(uint16_t action_id, uint8_t action_seq)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (s_camera_session.valid != 0u &&
        s_camera_session.acknowledged != 0u)
    {
        s_camera_session.action_pending = 1u;
        s_camera_session.action_acknowledged = 0u;
        s_camera_session.action_id = action_id;
        s_camera_session.action_seq = action_seq;
        s_camera_session.action_ack_rx_ms = 0u;
        s_camera_action_result.action = 0u;
        s_camera_action_result.result = 0u;
        s_camera_action_result.action_id = 0u;
        s_camera_action_result.source_time_ms = 0u;
        s_camera_action_result.task_type = 0u;
        s_camera_action_result.mission_id = 0u;
        s_camera_action_result.mode_seq = 0u;
        s_camera_action_result.local_rx_ms = 0u;
        s_camera_action_result.rx_counter = 0u;
    }
    __set_PRIMASK(primask);
}

void MY_uart_maixcam_clear_state(void)
{
    MY_uart_camera_invalidate_session();
}

void MY_uart_camera_begin_session(uint8_t task_type,
                                  uint16_t mission_id,
                                  uint8_t mode_seq)
{
    uint32_t primask;

    MY_uart_camera_invalidate_session();
    primask = __get_PRIMASK();
    __disable_irq();
    s_camera_session.valid = 1u;
    s_camera_session.task_type = task_type;
    s_camera_session.mission_id = mission_id;
    s_camera_session.mode_seq = mode_seq;
    __set_PRIMASK(primask);
}

uint8_t MY_uart_camera_session_ready(uint8_t task_type,
                                     uint16_t mission_id,
                                     uint8_t mode_seq)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t ready;

    __disable_irq();
    ready = (s_camera_session.valid != 0u &&
             s_camera_session.acknowledged != 0u &&
             s_camera_session.task_type == task_type &&
             s_camera_session.mission_id == mission_id &&
             s_camera_session.mode_seq == mode_seq)
                ? 1u
                : 0u;
    __set_PRIMASK(primask);
    return ready;
}

void MY_uart_maixcam_send(uint8_t code_type)
{
#if ENABLE_LEGACY_CAMERA_BENCH
    my_send_maixcam(code_type);
#else
    (void)code_type;
#endif
}

static void camera_stream_resync(uint8_t *frame,
                                 uint8_t *index,
                                 uint8_t *expected_len)
{
    uint8_t old_index = *index;
    uint8_t start;
    uint8_t i;

    *expected_len = 0u;
    for (start = 1u; (uint8_t)(start + 1u) < old_index; start++)
    {
        if (frame[start] == V21_HEAD_0 && frame[start + 1u] == V21_HEAD_1)
        {
            for (i = 0u; (uint8_t)(start + i) < old_index; i++)
            {
                frame[i] = frame[start + i];
            }
            *index = (uint8_t)(old_index - start);
            return;
        }
    }
    if (old_index != 0u && frame[old_index - 1u] == V21_HEAD_0)
    {
        frame[0] = V21_HEAD_0;
        *index = 1u;
        return;
    }
    *index = 0u;
}

void MY_uart_maixcam_receive(uint8_t data)
{
    static uint8_t frame[V21_MAX_FRAME_LEN];
    static uint8_t index;
    static uint8_t expected_len;
    static uint32_t last_byte_ms;
    uint32_t now = GetSysRunTimeMs();

    if (index != 0u && (uint32_t)(now - last_byte_ms) > V21_RX_TIMEOUT_MS)
    {
        index = 0u;
        expected_len = 0u;
    }
    last_byte_ms = now;

    if (index == 0u)
    {
        if (data == V21_HEAD_0)
        {
            frame[index++] = data;
        }
        return;
    }
    if (index == 1u)
    {
        if (data == V21_HEAD_1)
        {
            frame[index++] = data;
        }
        else if (data == V21_HEAD_0)
        {
            frame[0] = data;
            index = 1u;
        }
        else
        {
            index = 0u;
        }
        return;
    }

    frame[index++] = data;
    for (;;)
    {
        if (index >= 9u && expected_len == 0u)
        {
            if (frame[2] != V21_VERSION || frame[8] > V21_MAX_PAYLOAD)
            {
                camera_stream_resync(frame, &index, &expected_len);
                if (index < 9u)
                {
                    return;
                }
                continue;
            }
            expected_len = (uint8_t)(11u + frame[8]);
        }

        if (expected_len == 0u || index < expected_len)
        {
            return;
        }
        if (my_v21_crc16_ccitt_false(&frame[2], (uint8_t)(7u + frame[8])) ==
            read_be16(&frame[9u + frame[8]]))
        {
            (void)MY_uart_v21_camera_anl(frame, expected_len);
            index = 0u;
            expected_len = 0u;
            return;
        }
        camera_stream_resync(frame, &index, &expected_len);
        if (index < 9u)
        {
            return;
        }
    }
}

int MY_uart_v21_camera_anl(const uint8_t *data, uint8_t len)
{
    const uint8_t *payload;
    uint8_t payload_len;
    uint16_t expected_crc;
    uint16_t received_crc;
    uint32_t now;

    if (data == 0 || len < 11u ||
        data[0] != V21_HEAD_0 || data[1] != V21_HEAD_1 ||
        data[2] != V21_VERSION || data[4] != V21_ADDR_CAMERA ||
        data[5] != V21_ADDR_FC || data[8] > V21_MAX_PAYLOAD ||
        (data[7] & (uint8_t)~(V21_FLAG_ACK_REQUIRED |
                              V21_FLAG_RETRANSMISSION |
                              V21_FLAG_URGENT)) != 0u ||
        len != (uint8_t)(11u + data[8]))
    {
        return -1;
    }

    payload_len = data[8];
    expected_crc = my_v21_crc16_ccitt_false(&data[2], (uint8_t)(7u + payload_len));
    received_crc = read_be16(&data[9u + payload_len]);
    if (expected_crc != received_crc)
    {
        return -1;
    }

    payload = &data[9];
    now = GetSysRunTimeMs();

    if (data[3] == 0x11u)
    {
        camera_ack_t ack;

        if (payload_len != 4u ||
            (data[7] & V21_FLAG_ACK_REQUIRED) != 0u ||
            (payload[0] != CAMERA_TYPE_MODE && payload[0] != CAMERA_TYPE_ACTION) ||
            payload[2] > V21_ACK_INTERNAL_ERROR)
        {
            return -1;
        }
        ack.request_type = payload[0];
        ack.request_seq = payload[1];
        ack.result = payload[2];
        ack.detail = payload[3];
        ack.local_rx_ms = now;
        ack.rx_counter = s_camera_expected_ack.ack.rx_counter + 1u;

        if (s_camera_expected_ack.active != 0u &&
            s_camera_expected_ack.received == 0u &&
            ack.request_type == s_camera_expected_ack.request_type &&
            ack.request_seq == s_camera_expected_ack.request_seq)
        {
            s_camera_expected_ack.ack = ack;
            s_camera_expected_ack.received = 1u;
        }

        if (s_camera_session.valid != 0u &&
            ack.request_type == CAMERA_TYPE_MODE &&
            ack.request_seq == s_camera_session.mode_seq &&
            (ack.result == V21_ACK_ACCEPTED ||
             ack.result == V21_ACK_DUP_ACCEPTED))
        {
            s_camera_session.acknowledged = 1u;
            s_camera_session.ack_rx_ms = now;
        }
        if (s_camera_session.valid != 0u &&
            s_camera_session.acknowledged != 0u &&
            s_camera_session.action_pending != 0u &&
            ack.request_type == CAMERA_TYPE_ACTION &&
            ack.request_seq == s_camera_session.action_seq &&
            (ack.result == V21_ACK_ACCEPTED ||
             ack.result == V21_ACK_DUP_ACCEPTED))
        {
            s_camera_session.action_acknowledged = 1u;
            s_camera_session.action_ack_rx_ms = now;
        }
        return 1;
    }

    if (data[3] == CAMERA_TYPE_TARGET)
    {
        camera_target_t target;
        uint32_t previous_source_time = s_camera_target.source_time_ms;
        uint32_t previous_counter = s_camera_target.rx_counter;

        if (payload_len != 18u ||
            s_camera_session.valid == 0u ||
            s_camera_session.acknowledged == 0u)
        {
            return -1;
        }
        target.flags = payload[0];
        target.quality = payload[1];
        target.err_x_cm = (int16_t)read_be16(&payload[2]);
        target.err_y_cm = (int16_t)read_be16(&payload[4]);
        target.outer_diameter_px = read_be16(&payload[6]);
        target.frame_counter = read_be16(&payload[8]);
        target.source_time_ms = read_be32(&payload[10]);
        target.task_type = payload[14];
        target.mission_id = read_be16(&payload[15]);
        target.mode_seq = payload[17];

        if (target.task_type != s_camera_session.task_type ||
            target.mission_id != s_camera_session.mission_id ||
            target.mode_seq != s_camera_session.mode_seq ||
            (int32_t)(now - s_camera_session.ack_rx_ms) < 0)
        {
            return 0; /* Stale/wrong-session target must not refresh freshness. */
        }

        if (previous_counter != 0u &&
            source_time_is_newer(target.source_time_ms, previous_source_time) == 0u)
        {
            return 0;
        }
        target.local_rx_ms = now;
        target.rx_counter = previous_counter + 1u;
        s_camera_target = target;
        s_camera_last_rx_ms = now;
        return 1;
    }

    if (data[3] == CAMERA_TYPE_ACTION_RESULT)
    {
        camera_action_result_t result;
        uint32_t previous_source_time = s_camera_action_result.source_time_ms;
        uint32_t previous_counter = s_camera_action_result.rx_counter;

        if (payload_len != 12u ||
            s_camera_session.valid == 0u ||
            s_camera_session.acknowledged == 0u ||
            s_camera_session.action_pending == 0u ||
            s_camera_session.action_acknowledged == 0u)
        {
            return -1;
        }
        result.action = payload[0];
        result.result = payload[1];
        result.action_id = read_be16(&payload[2]);
        result.source_time_ms = read_be32(&payload[4]);
        result.task_type = payload[8];
        result.mission_id = read_be16(&payload[9]);
        result.mode_seq = payload[11];
        if (result.action != CAMERA_ACTION_DROP ||
            result.result > 3u ||
            result.action_id != s_camera_session.action_id ||
            result.task_type != s_camera_session.task_type ||
            result.mission_id != s_camera_session.mission_id ||
            result.mode_seq != s_camera_session.mode_seq ||
            (int32_t)(now - s_camera_session.action_ack_rx_ms) < 0)
        {
            return 0; /* Wrong-session result is ignored without refreshing link state. */
        }
        if (previous_counter != 0u &&
            source_time_is_newer(result.source_time_ms, previous_source_time) == 0u)
        {
            return 0;
        }
        result.local_rx_ms = now;
        result.rx_counter = previous_counter + 1u;
        s_camera_action_result = result;
        s_camera_last_rx_ms = now;
        return 1;
    }

    return -1;
}

int MY_uart_maixcam_anl(uint8_t *data, uint8_t len)
{
    return MY_uart_v21_camera_anl(data, len);
}
