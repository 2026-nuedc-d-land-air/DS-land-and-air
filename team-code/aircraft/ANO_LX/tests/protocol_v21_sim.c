#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "my_send_test.h"
#include "my_uart.h"

static uint32_t fake_time_ms = 1000u;
static uint8_t uart1_tx[128];
static uint8_t uart1_tx_len;
static uint8_t uart3_tx[128];
static uint8_t uart3_tx_len;

int32_t dis_x_slam;
int32_t dis_y_slam;
int32_t yaw_slam;

uint32_t GetSysRunTimeMs(void)
{
    return fake_time_ms;
}

void DrvUart1SendBuf(unsigned char *data, uint8_t len)
{
    memcpy(uart1_tx, data, len);
    uart1_tx_len = len;
}

void DrvUart2SendBuf(unsigned char *data, uint8_t len)
{
    (void)data;
    (void)len;
}

void DrvUart3SendBuf(unsigned char *data, uint8_t len)
{
    memcpy(uart3_tx, data, len);
    uart3_tx_len = len;
}

static void fail(const char *test, const char *message)
{
    fprintf(stderr, "[FAIL] %s: %s\n", test, message);
    exit(1);
}

static void expect(const char *test, int condition, const char *message)
{
    if (!condition)
    {
        fail(test, message);
    }
}

static uint8_t make_lx_extended(uint8_t subtype,
                                const uint8_t *payload,
                                uint8_t payload_len,
                                uint8_t *frame)
{
    uint8_t count = 0;
    uint8_t sc = 0;
    uint8_t ac = 0;
    uint8_t i;

    frame[count++] = 0xBBu;
    frame[count++] = 0x10u;
    frame[count++] = 0xF1u;
    frame[count++] = 0x30u;
    frame[count++] = subtype;
    frame[count++] = payload_len;
    for (i = 0; i < payload_len; i++)
    {
        frame[count++] = payload[i];
    }
    frame[count++] = 0xEEu;
    for (i = 0; i < count; i++)
    {
        sc = (uint8_t)(sc + frame[i]);
        ac = (uint8_t)(ac + sc);
    }
    frame[count++] = sc;
    frame[count++] = ac;
    return count;
}

static uint8_t make_camera_v22(uint8_t type,
                               uint8_t seq,
                               uint8_t flags,
                               const uint8_t *payload,
                               uint8_t payload_len,
                               uint8_t *frame)
{
    uint8_t count = 0;
    uint8_t i;
    uint16_t crc;

    frame[count++] = 0xAAu;
    frame[count++] = 0x55u;
    frame[count++] = 0x02u;
    frame[count++] = type;
    frame[count++] = 0x50u;
    frame[count++] = 0x21u;
    frame[count++] = seq;
    frame[count++] = flags;
    frame[count++] = payload_len;
    for (i = 0; i < payload_len; i++)
    {
        frame[count++] = payload[i];
    }
    crc = my_v21_crc16_ccitt_false(&frame[2], (uint8_t)(7u + payload_len));
    frame[count++] = (uint8_t)(crc >> 8);
    frame[count++] = (uint8_t)crc;
    return count;
}

static void feed_esp(const uint8_t *frame, uint8_t len)
{
    uint8_t i;
    for (i = 0; i < len; i++)
    {
        MY_uart_esp_receive(frame[i]);
    }
}

static void feed_camera(const uint8_t *frame, uint8_t len)
{
    uint8_t i;
    for (i = 0; i < len; i++)
    {
        MY_uart_maixcam_receive(frame[i]);
    }
}

static void test_crc_vectors(void)
{
    static const uint8_t standard[] = "123456789";
    static const uint8_t car_pose[] = {
        0xAA, 0x55, 0x02, 0x80, 0x30, 0x10, 0x42, 0x00, 0x16,
        0x01, 0x0F, 0x00, 0x01, 0x00, 0x00, 0x00, 0x78, 0xFF, 0xFF,
        0xFF, 0xCE, 0x03, 0x84, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00,
        0x04, 0xD2, 0xB2, 0x2D};
    static const uint8_t task[] = {
        0xAA, 0x55, 0x02, 0x81, 0x30, 0x20, 0x43, 0x01, 0x0C,
        0x01, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
        0x04, 0xD2, 0x7F, 0x78};
    static const uint8_t calibration[] = {
        0xAA, 0x55, 0x02, 0x83, 0x40, 0x30, 0x16, 0x01, 0x0C,
        0x00, 0x00, 0x00, 0x64, 0xFF, 0xFF, 0xFF, 0xCE, 0x00, 0x01,
        0x01, 0x00, 0xF1, 0x24};
    static const uint8_t maintenance_reset[] = {
        0xAA, 0x55, 0x02, 0x85, 0x30, 0x10, 0x52, 0x00, 0x08,
        0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x04, 0xD2, 0x9C, 0xB2};
    uint16_t crc;

    expect("crc_vectors",
           my_v21_crc16_ccitt_false(standard, 9u) == 0x29B1u,
           "CCITT-FALSE standard vector mismatch");

    crc = my_v21_crc16_ccitt_false(&car_pose[2], (uint8_t)(sizeof(car_pose) - 4u));
    expect("crc_vectors", crc == 0xB22Du, "CAR_POSE appendix CRC mismatch");
    crc = my_v21_crc16_ccitt_false(&task[2], (uint8_t)(sizeof(task) - 4u));
    expect("crc_vectors", crc == 0x7F78u, "CAR_TASK_REQUEST appendix CRC mismatch");
    crc = my_v21_crc16_ccitt_false(&calibration[2], (uint8_t)(sizeof(calibration) - 4u));
    expect("crc_vectors", crc == 0xF124u, "CALIBRATION_SET appendix CRC mismatch");
    crc = my_v21_crc16_ccitt_false(&maintenance_reset[2],
                                   (uint8_t)(sizeof(maintenance_reset) - 4u));
    expect("crc_vectors", crc == 0x9CB2u, "MAINTENANCE_RESET appendix CRC mismatch");
}

static void test_extended_lx_receive(void)
{
    uint8_t payload[22] = {
        0x01, 0x0F, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x78,
        0xFF, 0xFF, 0xFF, 0xCE,
        0x03, 0x84, 0x00, 0x0A,
        0x00, 0x00, 0x00, 0x00, 0x04, 0xD2};
    uint8_t frame[80];
    uint8_t len;
    uint8_t sample;
    lx_car_pose_t pose;

    for (sample = 0u; sample < 3u; sample++)
    {
        payload[21] = (uint8_t)(0xD2u + sample);
        len = make_lx_extended(LX_EXT_CAR_POSE, payload, sizeof(payload), frame);
        fake_time_ms += 100u;
        feed_esp(frame, len);
    }
    expect("extended_lx_receive", MY_uart_esp_get_car_pose(&pose) != 0u,
           "CAR_POSE was not published");
    expect("extended_lx_receive",
           pose.x_cm == 120 && pose.y_cm == -50 && pose.yaw_0p1deg == -900,
           "CAR_POSE fields decoded or yaw direction converted incorrectly");
    expect("extended_lx_receive", pose.usable != 0u && pose.valid_streak == 3u,
           "three valid calibrated poses did not form a precheck streak");

    frame[10] ^= 0x01u;
    feed_esp(frame, len);
    expect("extended_lx_receive", MY_uart_esp_get_car_pose(&pose) != 0u && pose.rx_counter == 3u,
           "bad LX checksum updated control state");

    payload[1] &= (uint8_t)~CAR_POSE_CALIBRATED;
    payload[21] = 0xD6u;
    len = make_lx_extended(LX_EXT_CAR_POSE, payload, sizeof(payload), frame);
    fake_time_ms += 100u;
    feed_esp(frame, len);
    MY_uart_esp_get_car_pose(&pose);
    expect("extended_lx_receive", pose.usable == 0u && pose.valid_streak == 0u,
           "cleared calibration was still usable");
}

static void test_mission_and_legacy_receive(void)
{
    uint8_t payload[12] = {
        0x02, 0x01, 0x12, 0x34, 0x00, 0x22,
        0x00, 0x00, 0x00, 0x00, 0x04, 0xD2};
    uint8_t frame[32];
    uint8_t len;
    uint8_t legacy[8] = {0xBB, 0x10, 0xF1, 0x10, 0x01, 0xEE, 0, 0};
    uint8_t sc = 0;
    uint8_t ac = 0;
    uint8_t i;
    lx_mission_request_t request;

    len = make_lx_extended(LX_EXT_MISSION_REQUEST, payload, sizeof(payload), frame);
    feed_esp(frame, len);
    expect("mission_receive", MY_uart_esp_take_mission_request(&request) != 0u,
           "MISSION_REQUEST event missing");
    expect("mission_receive",
           request.task_type == LX_TASK_DYNAMIC_LAND &&
               request.mission_id == 0x1234u && request.calibration_id == 0x22u,
           "MISSION_REQUEST fields decoded incorrectly");
    expect("mission_receive", MY_uart_esp_take_mission_request(&request) == 0u,
           "MISSION_REQUEST was not consumed atomically");

    for (i = 0u; i < 6u; i++)
    {
        sc = (uint8_t)(sc + legacy[i]);
        ac = (uint8_t)(ac + sc);
    }
    legacy[6] = sc;
    legacy[7] = ac;
    feed_esp(legacy, sizeof(legacy));
    expect("legacy_receive",
           (MY_uart_esp_take_mission_requests() & ESP_MISSION_REQUEST_START) != 0u,
           "legacy 8-byte START compatibility regressed");
}

static void test_maintenance_reset_receive(void)
{
    uint8_t payload[8] = {0x00u, 0x01u, MAINTENANCE_RESET_CLEAR_CALIBRATION,
                          0x00u, 0x00u, 0x00u, 0x04u, 0xD2u};
    uint8_t frame[32];
    uint8_t len;
    lx_maintenance_reset_t reset;
    lx_car_pose_t pose;

    len = make_lx_extended(LX_EXT_MAINTENANCE_RESET, payload, sizeof(payload), frame);
    fake_time_ms += 20u;
    feed_esp(frame, len);
    expect("maintenance_reset", MY_uart_esp_take_maintenance_reset(&reset) != 0u,
           "valid MAINTENANCE_RESET event missing");
    expect("maintenance_reset",
           reset.reset_id == 1u &&
               reset.reset_flags == MAINTENANCE_RESET_CLEAR_CALIBRATION &&
               reset.source_time_ms == 1234u,
           "MAINTENANCE_RESET fields decoded incorrectly");
    expect("maintenance_reset", MY_uart_esp_get_car_pose(&pose) != 0u &&
                                   pose.usable == 0u && pose.valid_streak == 0u &&
                                   pose.calibration_id == 0u,
           "MAINTENANCE_RESET did not immediately invalidate CAR_POSE");

    feed_esp(frame, len);
    expect("maintenance_reset", MY_uart_esp_take_maintenance_reset(&reset) == 0u,
           "duplicate ResetId published a second reset event");

    payload[0] = 0x00u;
    payload[1] = 0x02u;
    payload[2] = 0x03u;
    len = make_lx_extended(LX_EXT_MAINTENANCE_RESET, payload, sizeof(payload), frame);
    feed_esp(frame, len);
    expect("maintenance_reset", MY_uart_esp_take_maintenance_reset(&reset) == 0u,
           "invalid ResetFlags changed reset state");
}

static void test_mission_abort_receive(void)
{
    uint8_t payload = 0x01u;
    uint8_t frame[16];
    uint8_t len;

    len = make_lx_extended(LX_EXT_MISSION_ABORT, 0, 0u, frame);
    feed_esp(frame, len);
    expect("mission_abort", MY_uart_esp_take_mission_abort() != 0u,
           "zero-payload MISSION_ABORT event missing");
    expect("mission_abort", MY_uart_esp_take_mission_abort() == 0u,
           "MISSION_ABORT was not consumed atomically");

    len = make_lx_extended(LX_EXT_MISSION_ABORT, &payload, 1u, frame);
    feed_esp(frame, len);
    expect("mission_abort", MY_uart_esp_take_mission_abort() == 0u,
           "MISSION_ABORT with an undefined payload was accepted");
}

static void test_car_pose_timestamp_rollback(void)
{
    uint8_t payload[22] = {
        0x01u, CAR_POSE_POSITION_VALID | CAR_POSE_CALIBRATED | CAR_POSE_YAW_VALID,
        0x00u, 0x02u,
        0x00u, 0x00u, 0x00u, 0x78u,
        0xFFu, 0xFFu, 0xFFu, 0xCEu,
        0x03u, 0x84u, 0x7Fu, 0xFFu, 0x7Fu, 0xFFu,
        0x00u, 0x00u, 0x00u, 0x10u};
    uint8_t frame[40];
    uint8_t len;
    uint8_t sample;
    lx_car_pose_t pose;

    for (sample = 0u; sample < 3u; sample++)
    {
        payload[21] = (uint8_t)(0x10u + sample);
        len = make_lx_extended(LX_EXT_CAR_POSE, payload, sizeof(payload), frame);
        fake_time_ms += 100u;
        feed_esp(frame, len);
    }
    MY_uart_esp_get_car_pose(&pose);
    expect("car_pose_timestamp_rollback", pose.usable != 0u && pose.valid_streak == 3u,
           "fresh calibrated CAR_POSE did not become usable");

    payload[1] = CAR_POSE_POSITION_VALID | CAR_POSE_YAW_VALID;
    payload[2] = 0u;
    payload[3] = 0u;
    payload[18] = 0u;
    payload[19] = 0u;
    payload[20] = 0u;
    payload[21] = 0x01u;
    len = make_lx_extended(LX_EXT_CAR_POSE, payload, sizeof(payload), frame);
    fake_time_ms += 100u;
    feed_esp(frame, len);
    MY_uart_esp_get_car_pose(&pose);
    expect("car_pose_timestamp_rollback",
           pose.usable == 0u && pose.valid_streak == 0u && pose.calibration_id == 0u,
           "Pi uptime rollback with uncalibrated CAR_POSE was ignored");
}

static void test_camera_receive(void)
{
    uint8_t ack_payload[4] = {CAMERA_TYPE_MODE, 0x33u, V21_ACK_ACCEPTED, 0u};
    uint8_t payload[18] = {
        0x01, 80, 0x00, 0x0C, 0xFF, 0xF9,
        0x01, 0x2C, 0x00, 0x2A, 0x00, 0x00, 0x04, 0xD2,
        LX_TASK_DYNAMIC_LAND, 0x12, 0x34, 0x33};
    uint8_t frame[40];
    uint8_t len;
    camera_target_t target;
    uint32_t counter;
    uint32_t freshness;

    MY_uart_camera_begin_session(LX_TASK_DYNAMIC_LAND, 0x1234u, 0x33u);
    MY_uart_camera_expect_ack(CAMERA_TYPE_MODE, 0x33u);

    len = make_camera_v22(CAMERA_TYPE_TARGET, 0x19u, 0u,
                          payload, sizeof(payload), frame);
    fake_time_ms += 20u;
    feed_camera(frame, len);
    expect("camera_receive", MY_uart_camera_get_target(&target) == 0u,
           "pre-MODE-ACK target entered control state");

    len = make_camera_v22(0x11u, 0x20u, 0u,
                          ack_payload, sizeof(ack_payload), frame);
    feed_camera(frame, len);
    expect("camera_receive",
           MY_uart_camera_session_ready(LX_TASK_DYNAMIC_LAND, 0x1234u, 0x33u) != 0u,
           "matching MODE ACK did not open the camera session");

    len = make_camera_v22(CAMERA_TYPE_TARGET, 0x19u, 0u,
                          payload, sizeof(payload), frame);
    fake_time_ms += 20u;
    feed_camera(frame, len);
    expect("camera_receive", MY_uart_camera_get_target(&target) != 0u,
           "CAMERA_TARGET was not published");
    expect("camera_receive",
           target.err_x_cm == 12 && target.err_y_cm == -7 &&
               target.quality == 80u && target.outer_diameter_px == 300u,
           "CAMERA_TARGET fields decoded incorrectly");
    counter = target.rx_counter;
    freshness = MY_uart_camera_last_rx_ms();

    payload[13] = 0xD3u;
    len = make_camera_v22(CAMERA_TYPE_TARGET, 0x1Au, 0x80u,
                          payload, sizeof(payload), frame);
    fake_time_ms += 20u;
    feed_camera(frame, len);
    MY_uart_camera_get_target(&target);
    expect("camera_receive", target.rx_counter == counter &&
                                   MY_uart_camera_last_rx_ms() == freshness,
           "reserved header Flags updated camera control/freshness state");

    payload[17] = 0x34u;
    len = make_camera_v22(CAMERA_TYPE_TARGET, 0x1Au, 0u,
                          payload, sizeof(payload), frame);
    fake_time_ms += 20u;
    feed_camera(frame, len);
    MY_uart_camera_get_target(&target);
    expect("camera_receive", target.rx_counter == counter &&
                                  MY_uart_camera_last_rx_ms() == freshness,
           "wrong ModeSeq refreshed camera control/freshness state");
    payload[17] = 0x33u;

    len = make_camera_v22(CAMERA_TYPE_TARGET, 0x1Bu, 0u,
                          payload, sizeof(payload), frame);
    frame[len - 1u] ^= 0x01u;
    feed_camera(frame, len);
    MY_uart_camera_get_target(&target);
    expect("camera_receive", target.rx_counter == counter,
           "bad camera CRC updated target state");

    MY_uart_maixcam_receive(0xAAu);
    fake_time_ms += 101u;
    feed_camera(frame, len); /* Still CRC-corrupt; also verifies timeout resync path. */
    MY_uart_camera_get_target(&target);
    expect("camera_receive", target.rx_counter == counter,
           "timed-out partial frame updated target state");

    /* A corrupted Length consumes the next AA; the stream parser must retain it. */
    payload[13] = 0xD3u;
    len = make_camera_v22(CAMERA_TYPE_TARGET, 0x1Cu, 0u,
                          payload, sizeof(payload), frame);
    {
        uint8_t bad[40];
        memcpy(bad, frame, len);
        bad[8] = 19u;
        feed_camera(bad, len);
    }
    feed_camera(frame, len);
    MY_uart_camera_get_target(&target);
    expect("camera_receive", target.rx_counter == counter + 1u,
           "corrupted Length prevented immediate AA55 stream resynchronization");
}

static void test_camera_action_result_gate(void)
{
    uint8_t ack_payload[4] = {CAMERA_TYPE_ACTION, 0x44u, V21_ACK_ACCEPTED, 0u};
    uint8_t result_payload[12] = {
        CAMERA_ACTION_DROP, 0u, 0x12, 0x34,
        0x00, 0x00, 0x04, 0xD3,
        LX_TASK_DYNAMIC_LAND, 0x12, 0x34, 0x33};
    uint8_t frame[32];
    uint8_t len;
    camera_action_result_t result;
    uint32_t freshness = MY_uart_camera_last_rx_ms();

    MY_uart_camera_begin_action(0x1234u, 0x44u);
    MY_uart_camera_expect_ack(CAMERA_TYPE_ACTION, 0x44u);
    len = make_camera_v22(CAMERA_TYPE_ACTION_RESULT, 0x30u, 0u,
                          result_payload, sizeof(result_payload), frame);
    fake_time_ms += 20u;
    feed_camera(frame, len);
    expect("camera_action_gate",
           MY_uart_camera_get_action_result(&result) == 0u &&
               MY_uart_camera_last_rx_ms() == freshness,
           "RESULT before ACTION ACK refreshed control/freshness state");

    len = make_camera_v22(0x11u, 0x31u, 0u,
                          ack_payload, sizeof(ack_payload), frame);
    feed_camera(frame, len);
    len = make_camera_v22(CAMERA_TYPE_ACTION_RESULT, 0x32u, 0u,
                          result_payload, sizeof(result_payload), frame);
    fake_time_ms += 20u;
    feed_camera(frame, len);
    expect("camera_action_gate",
           MY_uart_camera_get_action_result(&result) != 0u &&
               result.action_id == 0x1234u && result.mission_id == 0x1234u,
           "matching post-ACK ACTION_RESULT was not published");

    freshness = MY_uart_camera_last_rx_ms();
    result_payload[3] = 0x35u; /* Wrong ActionId, but a newer source timestamp. */
    result_payload[7] = 0xD4u;
    len = make_camera_v22(CAMERA_TYPE_ACTION_RESULT, 0x33u, 0u,
                          result_payload, sizeof(result_payload), frame);
    fake_time_ms += 20u;
    feed_camera(frame, len);
    expect("camera_action_gate", MY_uart_camera_last_rx_ms() == freshness,
           "wrong ActionId refreshed camera freshness");
}

static void expect_lx_checksums(const char *test, const uint8_t *frame, uint8_t len)
{
    uint8_t sc = 0;
    uint8_t ac = 0;
    uint8_t i;
    for (i = 0u; i < (uint8_t)(len - 2u); i++)
    {
        sc = (uint8_t)(sc + frame[i]);
        ac = (uint8_t)(ac + sc);
    }
    expect(test, frame[len - 2u] == sc && frame[len - 1u] == ac,
           "LX double-accumulator checksum mismatch");
}

static void test_transmit_frames(void)
{
    uint16_t crc;
    uint8_t seq;

    my_send_esp_mission_response(0x81u, 0x1234u,
                                 LX_RESULT_STATE_DENIED, LX_DETAIL_CH6);
    expect("tx_response", uart1_tx_len == 15u, "MISSION_RESPONSE length mismatch");
    expect("tx_response",
           uart1_tx[3] == 0x30u && uart1_tx[4] == LX_EXT_MISSION_RESPONSE &&
               uart1_tx[5] == 6u && uart1_tx[6] == 0x81u &&
               uart1_tx[7] == 0x12u && uart1_tx[8] == 0x34u &&
               uart1_tx[9] == LX_RESULT_STATE_DENIED && uart1_tx[10] == LX_DETAIL_CH6 &&
               uart1_tx[12] == 0xEEu,
           "MISSION_RESPONSE layout mismatch");
    expect_lx_checksums("tx_response", uart1_tx, uart1_tx_len);

    my_send_esp_mission_status(LX_TASK_DROP, 5u, 0x1234u,
                               0x2468u, 7u, 0x01020304u);
    expect("tx_status", uart1_tx_len == 21u &&
                                uart1_tx[4] == LX_EXT_MISSION_STATUS && uart1_tx[5] == 12u,
           "MISSION_STATUS envelope mismatch");
    expect("tx_status", uart1_tx[8] == 0x12u && uart1_tx[9] == 0x34u &&
                              uart1_tx[10] == 0x24u && uart1_tx[11] == 0x68u &&
                              uart1_tx[14] == 0x01u && uart1_tx[17] == 0x04u,
           "MISSION_STATUS big-endian payload mismatch");
    expect_lx_checksums("tx_status", uart1_tx, uart1_tx_len);

    {
        mission_trace_t trace = {
            MISSION_TRACE_ABORT_REQUEST, 5u, 8u,
            MISSION_TRACE_FLAG_CAMERA_LINK | MISSION_TRACE_FLAG_CAR_FRESH,
            0x1234u, 0x01020304u, 30000u,
            -12, 34, 56, -78, 135, 150,
            CAMERA_TARGET_VALID, 77u, -8, 9, 200u,
            MISSION_ALIGN_BLOCK_HEIGHT | MISSION_ALIGN_BLOCK_YAW};

        my_send_esp_mission_trace(&trace);
        expect("tx_trace", uart1_tx_len == 44u &&
               uart1_tx[4] == LX_EXT_MISSION_TRACE && uart1_tx[5] == 35u,
               "MISSION_TRACE envelope mismatch");
        expect("tx_trace", uart1_tx[6] == MISSION_TRACE_ABORT_REQUEST &&
               uart1_tx[7] == 5u && uart1_tx[8] == 8u &&
               uart1_tx[10] == 0x12u && uart1_tx[11] == 0x34u &&
               uart1_tx[20] == 0xFFu && uart1_tx[21] == 0xF4u &&
               uart1_tx[32] == CAMERA_TARGET_VALID && uart1_tx[33] == 77u &&
               uart1_tx[34] == 0xFFu && uart1_tx[35] == 0xF8u &&
               uart1_tx[38] == 0x00u && uart1_tx[39] == 0xC8u &&
               uart1_tx[40] == (MISSION_ALIGN_BLOCK_HEIGHT | MISSION_ALIGN_BLOCK_YAW),
               "MISSION_TRACE payload layout mismatch");
        expect_lx_checksums("tx_trace", uart1_tx, uart1_tx_len);
    }

    my_send_esp_1_test(ESP_EVENT_STAGE_PRECHECK);
    expect("tx_legacy_diagnostic", uart1_tx_len == 9u &&
               uart1_tx[0] == 0xBBu && uart1_tx[1] == 0x10u &&
               uart1_tx[2] == 0xF1u && uart1_tx[3] == 0x02u &&
               uart1_tx[4] == 0x01u && uart1_tx[5] == 0x50u &&
               uart1_tx[6] == 0xEEu,
           "0x50xx legacy diagnostic layout mismatch");
    expect_lx_checksums("tx_legacy_diagnostic", uart1_tx, uart1_tx_len);

    seq = my_camera_allocate_seq();
    my_send_camera_mode_v22(seq, V21_FLAG_ACK_REQUIRED,
                            CAMERA_MODE_TRACK_LAND,
                            LX_TASK_DYNAMIC_LAND, 0x1234u, 0u);
    expect("tx_camera_mode", uart3_tx_len == 17u && uart3_tx[3] == CAMERA_TYPE_MODE &&
                                     uart3_tx[4] == 0x21u && uart3_tx[5] == 0x50u &&
                                     uart3_tx[6] == seq && uart3_tx[7] == V21_FLAG_ACK_REQUIRED &&
                                     uart3_tx[8] == 6u && uart3_tx[9] == CAMERA_MODE_TRACK_LAND,
           "CAMERA_MODE V2.2 layout/ACK flag mismatch");
    crc = my_v21_crc16_ccitt_false(&uart3_tx[2], (uint8_t)(uart3_tx_len - 4u));
    expect("tx_camera_mode",
           uart3_tx[uart3_tx_len - 2u] == (uint8_t)(crc >> 8) &&
               uart3_tx[uart3_tx_len - 1u] == (uint8_t)crc,
           "CAMERA_MODE CRC mismatch");

    my_send_camera_mode_v22(seq,
                            V21_FLAG_ACK_REQUIRED | V21_FLAG_RETRANSMISSION,
                            CAMERA_MODE_TRACK_LAND,
                            LX_TASK_DYNAMIC_LAND, 0x1234u, 0u);
    expect("tx_camera_mode_retransmission",
           uart3_tx[6] == seq &&
               uart3_tx[7] == (V21_FLAG_ACK_REQUIRED | V21_FLAG_RETRANSMISSION),
           "CAMERA_MODE retransmission changed Seq/flags");

    seq = my_camera_allocate_seq();
    my_send_camera_action_v22(seq, V21_FLAG_ACK_REQUIRED,
                              CAMERA_ACTION_DROP, 0u, 0x4321u,
                              LX_TASK_DROP, 0x4321u, 0x33u);
    expect("tx_camera_action", uart3_tx_len == 19u &&
                                       uart3_tx[3] == CAMERA_TYPE_ACTION &&
                                       uart3_tx[6] == seq &&
                                       uart3_tx[7] == V21_FLAG_ACK_REQUIRED &&
                                       uart3_tx[9] == CAMERA_ACTION_DROP &&
                                       uart3_tx[11] == 0x43u && uart3_tx[12] == 0x21u &&
                                       uart3_tx[13] == LX_TASK_DROP &&
                                       uart3_tx[14] == 0x43u && uart3_tx[15] == 0x21u &&
                                       uart3_tx[16] == 0x33u,
           "CAMERA_ACTION V2.2 session layout mismatch");
}

int main(void)
{
    test_crc_vectors();
    test_extended_lx_receive();
    test_maintenance_reset_receive();
    test_mission_abort_receive();
    test_car_pose_timestamp_rollback();
    test_mission_and_legacy_receive();
    test_camera_receive();
    test_camera_action_result_gate();
    test_transmit_frames();
    puts("[PASS] V2.3/LX protocol simulation: CRC, reset, session gate, RX rejection, legacy, TX layouts");
    return 0;
}
