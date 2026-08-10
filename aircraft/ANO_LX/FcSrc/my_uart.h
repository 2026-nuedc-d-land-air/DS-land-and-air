#ifndef __MT_UART_H
#define __MT_UART_H

#include <stdint.h>

#define ESP_MISSION_REQUEST_START 0x01u
#define ESP_MISSION_REQUEST_STOP  0x02u

#define LX_EXT_CAR_POSE           0x01u
#define LX_EXT_MISSION_REQUEST    0x02u
#define LX_EXT_MISSION_RESPONSE   0x03u
#define LX_EXT_MISSION_STATUS     0x04u
#define LX_EXT_MAINTENANCE_RESET  0x05u
#define LX_EXT_MISSION_ABORT      0x06u
#define LX_EXT_MISSION_TRACE      0x07u

#define LX_TASK_DROP              0x01u
#define LX_TASK_DYNAMIC_LAND      0x02u

#define LX_RESULT_ACCEPTED        0x00u
#define LX_RESULT_DUP_ACCEPTED    0x01u
#define LX_RESULT_BUSY            0x02u
#define LX_RESULT_STATE_DENIED    0x03u
#define LX_RESULT_UNSUPPORTED     0x04u
#define LX_RESULT_INVALID_PARAM   0x05u
#define LX_RESULT_INTERNAL_ERROR  0x06u

#define LX_DETAIL_NONE            0x00u
#define LX_DETAIL_CH6             0x01u
#define LX_DETAIL_POSE_STALE      0x02u
#define LX_DETAIL_CALIBRATION     0x03u
#define LX_DETAIL_MISSION_RUNNING 0x04u
#define LX_DETAIL_FC_UART_TIMEOUT 0x05u
#define LX_DETAIL_TASK_TYPE       0x06u
#define LX_DETAIL_CAMERA_MODE     0x07u

#define CAR_FRAME_FIELD_GLOBAL    0x01u
#define CAR_POSE_POSITION_VALID   0x01u
#define CAR_POSE_CALIBRATED       0x02u
#define CAR_POSE_VELOCITY_VALID   0x04u
#define CAR_POSE_YAW_VALID        0x08u
#define CAR_POSE_RUNNING          0x10u

#define MAINTENANCE_RESET_CLEAR_CALIBRATION 0x01u

#define CAMERA_TYPE_MODE          0x90u
#define CAMERA_TYPE_TARGET        0x91u
#define CAMERA_TYPE_ACTION        0x92u
#define CAMERA_TYPE_ACTION_RESULT 0x93u

#define V21_FLAG_ACK_REQUIRED     0x01u
#define V21_FLAG_RETRANSMISSION   0x02u
#define V21_FLAG_URGENT           0x04u

#define V21_ACK_ACCEPTED          0x00u
#define V21_ACK_DUP_ACCEPTED      0x01u
#define V21_ACK_BUSY              0x02u
#define V21_ACK_STATE_DENIED      0x03u
#define V21_ACK_UNSUPPORTED       0x04u
#define V21_ACK_INVALID_PARAM     0x05u
#define V21_ACK_INTERNAL_ERROR    0x06u

#define CAMERA_MODE_IDLE          0x00u
#define CAMERA_MODE_TRACK_DROP    0x01u
#define CAMERA_MODE_TRACK_LAND    0x02u
#define CAMERA_TARGET_VALID       0x01u
#define CAMERA_ACTION_DROP        0x01u
#define CAMERA_ACTION_RESET       0x02u

typedef struct
{
    uint8_t coordinate_frame;
    uint8_t pose_flags;
    uint16_t calibration_id;
    int32_t x_cm;
    int32_t y_cm;
    int16_t yaw_0p1deg;
    int16_t vx_cmps;
    int16_t vy_cmps;
    uint32_t source_time_ms;
    uint32_t local_rx_ms;
    uint32_t rx_counter;       /* Local LX sample counter; 6.2 payload has no upstream Seq. */
    uint8_t valid_streak;
    uint8_t usable;
} lx_car_pose_t;

typedef struct
{
    uint8_t task_type;
    uint8_t request_flags;
    uint16_t mission_id;
    uint16_t calibration_id;
    uint16_t reserved;
    uint32_t source_time_ms;
    uint32_t local_rx_ms;
    uint32_t rx_counter;
} lx_mission_request_t;

typedef struct
{
    uint16_t reset_id;
    uint8_t reset_flags;
    uint32_t source_time_ms;
    uint32_t local_rx_ms;
    uint32_t rx_counter;
} lx_maintenance_reset_t;

typedef struct
{
    uint8_t flags;
    uint8_t quality;
    int16_t err_x_cm;
    int16_t err_y_cm;
    uint16_t outer_diameter_px;
    uint16_t frame_counter;
    uint32_t source_time_ms;
    uint8_t task_type;
    uint16_t mission_id;
    uint8_t mode_seq;
    uint32_t local_rx_ms;
    uint32_t rx_counter;
} camera_target_t;

typedef struct
{
    uint8_t action;
    uint8_t result;
    uint16_t action_id;
    uint32_t source_time_ms;
    uint8_t task_type;
    uint16_t mission_id;
    uint8_t mode_seq;
    uint32_t local_rx_ms;
    uint32_t rx_counter;
} camera_action_result_t;

typedef struct
{
    uint8_t request_type;
    uint8_t request_seq;
    uint8_t result;
    uint8_t detail;
    uint32_t local_rx_ms;
    uint32_t rx_counter;
} camera_ack_t;

extern int16_t OpenMV_data_0, OpenMV_data_1, OpenMV_data_2;
extern int my_task_flag;
extern uint8_t my_slam_flag;

void MY_uart_esp_receive(uint8_t data);
int MY_uart_esp_anl(uint8_t *data, uint8_t len);
int MY_uart_esp_extended_anl(const uint8_t *data, uint8_t len);
uint8_t MY_uart_esp_take_mission_requests(void);
uint8_t MY_uart_esp_take_mission_request(lx_mission_request_t *request);
uint8_t MY_uart_esp_take_mission_abort(void);
uint8_t MY_uart_esp_take_maintenance_reset(lx_maintenance_reset_t *reset);
uint8_t MY_uart_esp_get_car_pose(lx_car_pose_t *pose);

void MY_uart_K230_receive(uint8_t data);
void MY_uart_K230_anl(uint8_t data);
void MY_uart_K230_send(uint8_t data);

void MY_uart_radar_receive(uint8_t data);
int MY_uart_radar_anl(uint8_t *data, uint8_t len);
void MY_uart_radar_send(uint8_t data);
uint32_t MY_uart_mid360_last_rx_ms(void);

void MY_uart_mark_laser_update(int32_t range_cm, uint8_t sensor_state);
uint32_t MY_uart_laser_last_rx_ms(void);
uint8_t MY_uart_laser_is_valid(void);

void MY_uart_maixcam_clear_state(void);
void MY_uart_maixcam_send(uint8_t code_type); /* Legacy bench hook; disabled by default. */
void MY_uart_maixcam_receive(uint8_t data);
int MY_uart_maixcam_anl(uint8_t *data, uint8_t len);
int MY_uart_v21_camera_anl(const uint8_t *data, uint8_t len);
void MY_uart_camera_begin_session(uint8_t task_type,
                                  uint16_t mission_id,
                                  uint8_t mode_seq);
void MY_uart_camera_begin_action(uint16_t action_id, uint8_t action_seq);
void MY_uart_camera_invalidate_session(void);
uint8_t MY_uart_camera_session_ready(uint8_t task_type,
                                     uint16_t mission_id,
                                     uint8_t mode_seq);
void MY_uart_camera_expect_ack(uint8_t request_type, uint8_t request_seq);
void MY_uart_camera_cancel_expected_ack(void);
uint8_t MY_uart_camera_get_expected_ack(uint8_t request_type,
                                        uint8_t request_seq,
                                        camera_ack_t *ack);
uint8_t MY_uart_camera_get_target(camera_target_t *target);
uint8_t MY_uart_camera_get_action_result(camera_action_result_t *result);
uint32_t MY_uart_camera_last_rx_ms(void);

#endif
