#ifndef __MY_SEND_TEST_H
#define __MY_SEND_TEST_H

#include <stdint.h>

#define SEND_ESP_HEAD 0xBBu
#define SEND_ESP_ADDR 0x10u
#define SEND_ESP_END  0xEEu

/*
 * Legacy 16-bit diagnostic/event commands transported by my_send_esp_1_test().
 * ESP32 prints every value below except 0x0010, which remains the audible
 * personnel-clearance warning.
 */
#define ESP_DIAG_START_WARNING       0x0010u
#define ESP_DIAG_RC_OR_CH6           0x0020u
#define ESP_DIAG_CAR_POSE            0x0030u
#define ESP_DIAG_MID360              0x0031u
#define ESP_DIAG_LASER_HEIGHT        0x0032u
#define ESP_DIAG_CALIBRATION         0x0033u
#define ESP_DIAG_CAMERA_MODE         0x0034u
#define ESP_DIAG_FC_MODE             0x0040u
#define ESP_DIAG_FC_UNLOCK           0x0041u

/* 0x50xx: autonomous mission stage entered. */
#define ESP_EVENT_STAGE_IDLE                 0x5000u
#define ESP_EVENT_STAGE_PRECHECK             0x5001u
#define ESP_EVENT_STAGE_TAKEOFF              0x5002u
#define ESP_EVENT_STAGE_INTERCEPT            0x5003u
#define ESP_EVENT_STAGE_FOLLOW               0x5004u
#define ESP_EVENT_STAGE_DROP_ALIGN           0x5005u
#define ESP_EVENT_STAGE_DROP_ACTION          0x5006u
#define ESP_EVENT_STAGE_LAND_ALIGN           0x5007u
#define ESP_EVENT_STAGE_DESCEND              0x5008u
#define ESP_EVENT_STAGE_ON_PLATFORM_HOLD     0x5009u
#define ESP_EVENT_STAGE_PLATFORM_TAKEOFF     0x500Au
#define ESP_EVENT_STAGE_RETURN_HOME          0x500Bu
#define ESP_EVENT_STAGE_HOME_LAND            0x500Cu
#define ESP_EVENT_STAGE_ABORT                0x500Du

/* 0x51xx: edge-triggered critical feedback confirmations. */
#define ESP_EVENT_CAMERA_MODE_ACK_CONFIRMED   0x5100u
#define ESP_EVENT_FC_MODE_2_CONFIRMED         0x5101u
#define ESP_EVENT_FC_UNLOCK_CONFIRMED         0x5102u
#define ESP_EVENT_CAMERA_ACTION_ACK_CONFIRMED 0x5103u
#define ESP_EVENT_CAMERA_DROP_COMPLETED       0x5104u
#define ESP_EVENT_FC_LAND_COMMAND_ACCEPTED    0x5105u

/* 0x52xx: reason recorded when entering ABORT. */
#define ESP_EVENT_ABORT_PRECHECK             0x5200u
#define ESP_EVENT_ABORT_CAR_POSE_LOST        0x5201u
#define ESP_EVENT_ABORT_CAMERA_LOST          0x5202u
#define ESP_EVENT_ABORT_CALIBRATION          0x5203u
#define ESP_EVENT_ABORT_MID360_LOST          0x5204u
#define ESP_EVENT_ABORT_LASER_HEIGHT_LOST    0x5205u
#define ESP_EVENT_ABORT_CAMERA_ACTION        0x5206u
#define ESP_EVENT_ABORT_TIMEOUT              0x5207u
#define ESP_EVENT_ABORT_RC_STOP              0x5208u
#define ESP_EVENT_ABORT_REMOTE_COMMAND       0x5209u
#define ESP_EVENT_ABORT_INVALID_STAGE        0x520Au

#define MISSION_TRACE_STAGE_ENTER             0x01u
#define MISSION_TRACE_ABORT_REQUEST           0x02u
#define MISSION_TRACE_ABORT_RETURN_START      0x03u
#define MISSION_TRACE_ABORT_HOME_REACHED      0x04u
#define MISSION_TRACE_ABORT_RETURN_TIMEOUT    0x05u
#define MISSION_TRACE_ABORT_LAND_ACCEPTED     0x06u

#define MISSION_TRACE_FLAG_HARD_LAND          0x01u
#define MISSION_TRACE_FLAG_RETURN_STARTED     0x02u
#define MISSION_TRACE_FLAG_CAMERA_LINK        0x04u
#define MISSION_TRACE_FLAG_CAMERA_VALID       0x08u
#define MISSION_TRACE_FLAG_CAMERA_CENTERED    0x10u
#define MISSION_TRACE_FLAG_CAR_FRESH          0x20u
#define MISSION_TRACE_FLAG_FC_UNLOCKED        0x40u

#define MISSION_ALIGN_BLOCK_CAR_POSE          0x01u
#define MISSION_ALIGN_BLOCK_CAMERA_LINK       0x02u
#define MISSION_ALIGN_BLOCK_POSITION          0x04u
#define MISSION_ALIGN_BLOCK_HEIGHT            0x08u
#define MISSION_ALIGN_BLOCK_YAW               0x10u

typedef struct
{
    uint8_t event;
    uint8_t stage;
    uint8_t error_code;
    uint8_t flags;
    uint16_t mission_id;
    uint32_t fc_time_ms;
    uint32_t stage_elapsed_ms;
    int16_t air_x_cm;
    int16_t air_y_cm;
    int16_t home_x_cm;
    int16_t home_y_cm;
    int16_t height_cm;
    int16_t range_target_cm;
    uint8_t camera_flags;
    uint8_t camera_quality;
    int16_t camera_err_x_cm;
    int16_t camera_err_y_cm;
    uint16_t camera_age_ms;
    uint8_t align_blockers;
} mission_trace_t;

uint16_t my_v21_crc16_ccitt_false(const uint8_t *data, uint8_t len);

void my_send_esp_4_test(int Data_A, int Data_B, int Data_C, int Data_D);
void my_send_esp_1_test(int Data_A);
void my_send_lx_extended(uint8_t subtype, const uint8_t *payload, uint8_t payload_len);
void my_send_esp_mission_response(uint8_t request_type,
                                  uint16_t mission_id,
                                  uint8_t result,
                                  uint8_t detail);
void my_send_esp_mission_status(uint8_t task_type,
                                 uint8_t stage,
                                 uint16_t status_flags,
                                 uint16_t mission_id,
                                 uint8_t error_code,
                                 uint32_t source_time_ms);
void my_send_esp_mission_trace(const mission_trace_t *trace);

uint8_t my_camera_allocate_seq(void);
void my_send_camera_mode_v22(uint8_t seq,
                             uint8_t header_flags,
                             uint8_t mode,
                             uint8_t task_type,
                             uint16_t mission_id,
                             uint8_t mode_flags);
void my_send_camera_action_v22(uint8_t seq,
                               uint8_t header_flags,
                               uint8_t action,
                               uint8_t action_flags,
                               uint16_t action_id,
                               uint8_t task_type,
                               uint16_t mission_id,
                               uint8_t mode_seq);

void my_send_maixcam(uint8_t code_type); /* Legacy bench hook; disabled by default. */

#endif
