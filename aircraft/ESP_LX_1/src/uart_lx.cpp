#include "uart_lx.h"

#include "protocol_v2.h"
#include "uart_lora.h"

#include <Arduino.h>
#include <string.h>

using namespace gpio_lora_v2;

namespace {

static const uint32_t kLxInterByteTimeoutMs = 100;
static const size_t kLxFrameCapacity = 80;
static const uint8_t kMissionTraceLength = 35;
/* Must stay in sync with FcSrc/my_send_test.h on the flight-controller MCU. */
static const uint16_t ESP_DIAG_START_WARNING = 0x0010u;
static const uint16_t ESP_DIAG_RC_OR_CH6 = 0x0020u;
static const uint16_t ESP_DIAG_CAR_POSE = 0x0030u;
static const uint16_t ESP_DIAG_MID360 = 0x0031u;
static const uint16_t ESP_DIAG_LASER_HEIGHT = 0x0032u;
static const uint16_t ESP_DIAG_CALIBRATION = 0x0033u;
static const uint16_t ESP_DIAG_CAMERA_MODE = 0x0034u;
static const uint16_t ESP_DIAG_FC_MODE = 0x0040u;
static const uint16_t ESP_DIAG_FC_UNLOCK = 0x0041u;

static const uint16_t ESP_EVENT_STAGE_IDLE = 0x5000u;
static const uint16_t ESP_EVENT_STAGE_PRECHECK = 0x5001u;
static const uint16_t ESP_EVENT_STAGE_TAKEOFF = 0x5002u;
static const uint16_t ESP_EVENT_STAGE_INTERCEPT = 0x5003u;
static const uint16_t ESP_EVENT_STAGE_FOLLOW = 0x5004u;
static const uint16_t ESP_EVENT_STAGE_DROP_ALIGN = 0x5005u;
static const uint16_t ESP_EVENT_STAGE_DROP_ACTION = 0x5006u;
static const uint16_t ESP_EVENT_STAGE_LAND_ALIGN = 0x5007u;
static const uint16_t ESP_EVENT_STAGE_DESCEND = 0x5008u;
static const uint16_t ESP_EVENT_STAGE_ON_PLATFORM_HOLD = 0x5009u;
static const uint16_t ESP_EVENT_STAGE_PLATFORM_TAKEOFF = 0x500Au;
static const uint16_t ESP_EVENT_STAGE_RETURN_HOME = 0x500Bu;
static const uint16_t ESP_EVENT_STAGE_HOME_LAND = 0x500Cu;
static const uint16_t ESP_EVENT_STAGE_ABORT = 0x500Du;

static const uint16_t ESP_EVENT_CAMERA_MODE_ACK_CONFIRMED = 0x5100u;
static const uint16_t ESP_EVENT_FC_MODE_2_CONFIRMED = 0x5101u;
static const uint16_t ESP_EVENT_FC_UNLOCK_CONFIRMED = 0x5102u;
static const uint16_t ESP_EVENT_CAMERA_ACTION_ACK_CONFIRMED = 0x5103u;
static const uint16_t ESP_EVENT_CAMERA_DROP_COMPLETED = 0x5104u;
static const uint16_t ESP_EVENT_FC_LAND_COMMAND_ACCEPTED = 0x5105u;

static const uint16_t ESP_EVENT_ABORT_PRECHECK = 0x5200u;
static const uint16_t ESP_EVENT_ABORT_CAR_POSE_LOST = 0x5201u;
static const uint16_t ESP_EVENT_ABORT_CAMERA_LOST = 0x5202u;
static const uint16_t ESP_EVENT_ABORT_CALIBRATION = 0x5203u;
static const uint16_t ESP_EVENT_ABORT_MID360_LOST = 0x5204u;
static const uint16_t ESP_EVENT_ABORT_LASER_HEIGHT_LOST = 0x5205u;
static const uint16_t ESP_EVENT_ABORT_CAMERA_ACTION = 0x5206u;
static const uint16_t ESP_EVENT_ABORT_TIMEOUT = 0x5207u;
static const uint16_t ESP_EVENT_ABORT_RC_STOP = 0x5208u;
static const uint16_t ESP_EVENT_ABORT_REMOTE_COMMAND = 0x5209u;
static const uint16_t ESP_EVENT_ABORT_INVALID_STAGE = 0x520Au;

ANO_info_st s_ano_info;
bool s_has_position = false;
bool s_task_running = false;
uint8_t s_mode_code = 0;
uint32_t s_last_position_ms = 0;
portMUX_TYPE s_snapshot_mux = portMUX_INITIALIZER_UNLOCKED;

const char *diagnostic_name(uint16_t command)
{
    switch (command)
    {
    case ESP_DIAG_RC_OR_CH6: return "RC or CH6 not ready";
    case ESP_DIAG_CAR_POSE: return "car pose not fresh/valid";
    case ESP_DIAG_MID360: return "MID360 localization not fresh";
    case ESP_DIAG_LASER_HEIGHT: return "laser height not fresh/valid";
    case ESP_DIAG_CALIBRATION: return "car calibration mismatch";
    case ESP_DIAG_CAMERA_MODE: return "camera MODE ACK failed";
    case ESP_DIAG_FC_MODE: return "flight-control mode feedback timeout";
    case ESP_DIAG_FC_UNLOCK: return "flight-control unlock feedback timeout";
    case ESP_EVENT_STAGE_IDLE: return "IDLE entered";
    case ESP_EVENT_STAGE_PRECHECK: return "PRECHECK entered";
    case ESP_EVENT_STAGE_TAKEOFF: return "TAKEOFF entered";
    case ESP_EVENT_STAGE_INTERCEPT: return "INTERCEPT entered";
    case ESP_EVENT_STAGE_FOLLOW: return "FOLLOW entered";
    case ESP_EVENT_STAGE_DROP_ALIGN: return "DROP_ALIGN entered";
    case ESP_EVENT_STAGE_DROP_ACTION: return "DROP_ACTION entered";
    case ESP_EVENT_STAGE_LAND_ALIGN: return "LAND_ALIGN entered";
    case ESP_EVENT_STAGE_DESCEND: return "DESCEND entered";
    case ESP_EVENT_STAGE_ON_PLATFORM_HOLD: return "ON_PLATFORM_HOLD entered";
    case ESP_EVENT_STAGE_PLATFORM_TAKEOFF: return "PLATFORM_TAKEOFF entered";
    case ESP_EVENT_STAGE_RETURN_HOME: return "RETURN_HOME entered";
    case ESP_EVENT_STAGE_HOME_LAND: return "HOME_LAND entered";
    case ESP_EVENT_STAGE_ABORT: return "ABORT entered";
    case ESP_EVENT_CAMERA_MODE_ACK_CONFIRMED: return "CAMERA_MODE ACK confirmed";
    case ESP_EVENT_FC_MODE_2_CONFIRMED: return "FC mode 2 confirmed";
    case ESP_EVENT_FC_UNLOCK_CONFIRMED: return "FC unlock confirmed";
    case ESP_EVENT_CAMERA_ACTION_ACK_CONFIRMED: return "camera ACTION ACK confirmed";
    case ESP_EVENT_CAMERA_DROP_COMPLETED: return "camera DROP completed";
    case ESP_EVENT_FC_LAND_COMMAND_ACCEPTED: return "FC land command accepted";
    case ESP_EVENT_ABORT_PRECHECK: return "precheck failed";
    case ESP_EVENT_ABORT_CAR_POSE_LOST: return "car pose lost/stale";
    case ESP_EVENT_ABORT_CAMERA_LOST: return "camera target/link lost";
    case ESP_EVENT_ABORT_CALIBRATION: return "calibration changed/reset";
    case ESP_EVENT_ABORT_MID360_LOST: return "MID360 localization lost";
    case ESP_EVENT_ABORT_LASER_HEIGHT_LOST: return "laser height lost/invalid";
    case ESP_EVENT_ABORT_CAMERA_ACTION: return "camera action failed";
    case ESP_EVENT_ABORT_TIMEOUT: return "mission timeout";
    case ESP_EVENT_ABORT_RC_STOP: return "RC STOP/failsafe";
    case ESP_EVENT_ABORT_REMOTE_COMMAND: return "remote MISSION_ABORT";
    case ESP_EVENT_ABORT_INVALID_STAGE: return "invalid mission stage";
    default: return nullptr;
    }
}

const char *diagnostic_prefix(uint16_t command)
{
    if (command >= ESP_EVENT_STAGE_IDLE && command <= ESP_EVENT_STAGE_ABORT)
    {
        return "[LX Stage]";
    }
    if (command >= ESP_EVENT_CAMERA_MODE_ACK_CONFIRMED &&
        command <= ESP_EVENT_FC_LAND_COMMAND_ACCEPTED)
    {
        return "[LX Event]";
    }
    if (command >= ESP_EVENT_ABORT_PRECHECK &&
        command <= ESP_EVENT_ABORT_INVALID_STAGE)
    {
        return "[LX Abort]";
    }
    return "[LX Precheck]";
}

int16_t normalize_yaw_deci_degrees(int16_t yaw)
{
    int32_t normalized = static_cast<int32_t>(yaw) % 3600;
    if (normalized > 1800)
    {
        normalized -= 3600;
    }
    else if (normalized < -1800)
    {
        normalized += 3600;
    }
    return static_cast<int16_t>(normalized);
}

bool write_lx_locked(const uint8_t *packet, size_t length)
{
    if (packet == nullptr || length == 0)
    {
        return false;
    }

    if (lxTxMutex != nullptr &&
        xSemaphoreTake(lxTxMutex, pdMS_TO_TICKS(20)) != pdTRUE)
    {
        Serial.println("[LX Tx] mutex timeout");
        return false;
    }

    const size_t written = LX.write(packet, length);
    LX.flush();
    if (lxTxMutex != nullptr)
    {
        xSemaphoreGive(lxTxMutex);
    }
    return written == length;
}

bool lx_checksum_valid(const uint8_t *frame, size_t total_length)
{
    if (frame == nullptr || total_length < 8)
    {
        return false;
    }

    const size_t end_index = total_length - 3;
    if (frame[end_index] != 0xEE)
    {
        return false;
    }

    uint8_t sc;
    uint8_t ac;
    lxDoubleChecksum(frame, end_index + 1, sc, ac);
    return sc == frame[end_index + 1] && ac == frame[end_index + 2];
}

uint16_t read_be16(const uint8_t *data)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) |
                                 static_cast<uint16_t>(data[1]));
}

uint32_t read_be32(const uint8_t *data)
{
    return (static_cast<uint32_t>(data[0]) << 24) |
           (static_cast<uint32_t>(data[1]) << 16) |
           (static_cast<uint32_t>(data[2]) << 8) |
           static_cast<uint32_t>(data[3]);
}

const char *mission_trace_name(uint8_t event)
{
    switch (event)
    {
    case 0x01: return "stage-enter";
    case 0x02: return "abort-request";
    case 0x03: return "abort-return-start";
    case 0x04: return "abort-home-reached";
    case 0x05: return "abort-return-timeout";
    case 0x06: return "abort-land-accepted";
    default: return "unknown";
    }
}

void handle_extended_frame(uint8_t subtype, const uint8_t *payload,
                           uint8_t payload_length)
{
    if (subtype == LX_MISSION_RESPONSE && payload_length == 6)
    {
        MissionResponsePayload response;
        if (!decodeMissionResponsePayload(payload, payload_length, response) ||
            response.reserved != 0)
        {
            Serial.println("[LX Rx] invalid MISSION_RESPONSE");
            return;
        }
        lora_on_mission_response(response);
        return;
    }

    if (subtype == LX_MISSION_STATUS && payload_length == 12)
    {
        MissionStatusPayload status;
        if (!decodeMissionStatusPayload(payload, payload_length, status) ||
            status.reserved != 0 || status.stage > 13 ||
            (status.taskType != 1 && status.taskType != 2))
        {
            Serial.println("[LX Rx] invalid MISSION_STATUS");
            return;
        }

        portENTER_CRITICAL(&s_snapshot_mux);
        s_mode_code = status.stage;
        s_task_running = status.stage != 0;
        portEXIT_CRITICAL(&s_snapshot_mux);
        Serial.printf("[LX Status] missionId=%u task=%u stage=%u error=%u flags=0x%04X fcMs=%lu\n",
                      status.missionId, status.taskType, status.stage,
                      status.errorCode, status.statusFlags,
                      static_cast<unsigned long>(status.sourceTimeMs));
        lora_queue_mission_status(payload, payload_length);
        return;
    }

    if (subtype == LX_MISSION_TRACE && payload_length == kMissionTraceLength)
    {
        const uint8_t event = payload[0];
        const uint8_t stage = payload[1];
        const uint8_t error = payload[2];
        const uint8_t flags = payload[3];
        const uint16_t mission_id = read_be16(payload + 4);
        const uint32_t fc_ms = read_be32(payload + 6);
        const uint32_t stage_ms = read_be32(payload + 10);
        const int16_t air_x = static_cast<int16_t>(read_be16(payload + 14));
        const int16_t air_y = static_cast<int16_t>(read_be16(payload + 16));
        const int16_t home_x = static_cast<int16_t>(read_be16(payload + 18));
        const int16_t home_y = static_cast<int16_t>(read_be16(payload + 20));
        const int16_t height = static_cast<int16_t>(read_be16(payload + 22));
        const int16_t range_target = static_cast<int16_t>(read_be16(payload + 24));
        const uint8_t camera_flags = payload[26];
        const uint8_t camera_quality = payload[27];
        const int16_t camera_err_x = static_cast<int16_t>(read_be16(payload + 28));
        const int16_t camera_err_y = static_cast<int16_t>(read_be16(payload + 30));
        const uint16_t camera_age = read_be16(payload + 32);
        const uint8_t align_blockers = payload[34];

        Serial.printf("[LX Trace] %s missionId=%u stage=%u error=%u fcMs=%lu stageMs=%lu flags=0x%02X alignBlock=0x%02X air=(%d,%d) home=(%d,%d) h=%d range=%d cam=(flags=0x%02X q=%u err=%d,%d age=%u)\n",
                      mission_trace_name(event), mission_id, stage, error,
                      static_cast<unsigned long>(fc_ms),
                      static_cast<unsigned long>(stage_ms), flags, align_blockers,
                      air_x, air_y,
                      home_x, home_y, height, range_target, camera_flags,
                      camera_quality, camera_err_x, camera_err_y, camera_age);
        return;
    }

    Serial.printf("[LX Rx] unsupported extended subtype 0x%02X len %u\n",
                  subtype, payload_length);
}

} // namespace

void data_receive_LX()
{
    enum RxState : uint8_t {
        WAIT_HEAD = 0,
        LX_FRAME
    };

    static RxState state = WAIT_HEAD;
    static uint8_t lx_frame[kLxFrameCapacity] = {0};
    static size_t lx_count = 0;
    static size_t lx_expected = 0;
    static uint32_t last_byte_ms = 0;
    static bool partial_frame = false;

    const uint32_t call_time_ms = millis();
    if (partial_frame &&
        static_cast<uint32_t>(call_time_ms - last_byte_ms) >
            kLxInterByteTimeoutMs)
    {
        state = WAIT_HEAD;
        lx_count = 0;
        lx_expected = 0;
        partial_frame = false;
    }

    int processed = 0;
    while (LX.available() > 0 && processed++ < 96)
    {
        const uint8_t byte_in = static_cast<uint8_t>(LX.read());
        last_byte_ms = millis();
        partial_frame = true;

        switch (state)
        {
        case WAIT_HEAD:
            if (byte_in == 0xBB)
            {
                lx_frame[0] = byte_in;
                lx_count = 1;
                lx_expected = 0;
                state = LX_FRAME;
            }
            else
            {
                partial_frame = false;
            }
            break;

        case LX_FRAME:
            if (lx_count >= sizeof(lx_frame))
            {
                state = WAIT_HEAD;
                lx_count = 0;
                lx_expected = 0;
                partial_frame = false;
                break;
            }
            lx_frame[lx_count++] = byte_in;

            if ((lx_count == 2 && lx_frame[1] != 0x10) ||
                (lx_count == 3 && lx_frame[2] != 0xF1))
            {
                state = WAIT_HEAD;
                lx_count = 0;
                lx_expected = 0;
                partial_frame = false;
                break;
            }

            if (lx_count == 4 && lx_frame[3] != 0x30)
            {
                const uint8_t legacy_length = lx_frame[3];
                if (legacy_length != 0x02 && legacy_length != 0x08)
                {
                    state = WAIT_HEAD;
                    lx_count = 0;
                    partial_frame = false;
                    break;
                }
                lx_expected = static_cast<size_t>(legacy_length) + 7;
            }
            else if (lx_count == 6 && lx_frame[3] == 0x30)
            {
                const uint8_t subtype = lx_frame[4];
                const uint8_t payload_length = lx_frame[5];
                const bool valid_shape =
                    (subtype == LX_MISSION_RESPONSE && payload_length == 6) ||
                    (subtype == LX_MISSION_STATUS && payload_length == 12) ||
                    (subtype == LX_MISSION_TRACE &&
                     payload_length == kMissionTraceLength);
                if (!valid_shape)
                {
                    state = WAIT_HEAD;
                    lx_count = 0;
                    partial_frame = false;
                    break;
                }
                lx_expected = static_cast<size_t>(payload_length) + 9;
            }

            if (lx_expected != 0 && lx_count == lx_expected)
            {
                if (!lx_checksum_valid(lx_frame, lx_count))
                {
                    Serial.println("[LX Rx] checksum/tail error");
                }
                else if (lx_frame[3] == 0x30)
                {
                    handle_extended_frame(lx_frame[4], lx_frame + 6,
                                          lx_frame[5]);
                }
                else
                {
                    data_anl_LX(lx_frame + 4, lx_frame[3]);
                }
                state = WAIT_HEAD;
                lx_count = 0;
                lx_expected = 0;
                partial_frame = false;
            }
            break;
        }
    }

    RECEIVE_FLAG = flight_position_is_fresh(millis()) ? 1 : 0;
}

void data_anl_LX(uint8_t *data, uint8_t data_len)
{
    if (data == nullptr)
    {
        return;
    }

    // Legacy LX command: [BB 10 F1 02 cmd_lo cmd_hi EE SC AC].
    // Queue the beep so the UART receive task never blocks for its duration.
    if (data_len == 0x02)
    {
        const uint16_t command = static_cast<uint16_t>(data[0]) |
                                 (static_cast<uint16_t>(data[1]) << 8);
        if (command == ESP_DIAG_START_WARNING)
        {
            Beep_Async(1, 1000);
        }
        else if (const char *diagnostic = diagnostic_name(command))
        {
            Serial.printf("%s %s (0x%04X)\n", diagnostic_prefix(command),
                          diagnostic, command);
        }
        else
        {
            Serial.printf("[LX Rx] unsupported legacy command 0x%04X\n",
                          command);
        }
        return;
    }

    if (data_len == 0x08)
    {
        const int16_t x_cm = static_cast<int16_t>(
            static_cast<uint16_t>(data[0]) |
            (static_cast<uint16_t>(data[1]) << 8));
        const int16_t y_cm = static_cast<int16_t>(
            static_cast<uint16_t>(data[2]) |
            (static_cast<uint16_t>(data[3]) << 8));
        const int16_t height_cm = static_cast<int16_t>(
            static_cast<uint16_t>(data[4]) |
            (static_cast<uint16_t>(data[5]) << 8));
        const int16_t yaw_deci_degrees = static_cast<int16_t>(
            static_cast<uint16_t>(data[6]) |
            (static_cast<uint16_t>(data[7]) << 8));
        portENTER_CRITICAL(&s_snapshot_mux);
        s_ano_info.dis_x = x_cm;
        s_ano_info.dis_y = y_cm;
        s_ano_info.hight = height_cm;
        s_ano_info.yaw = yaw_deci_degrees;
        s_has_position = true;
        s_last_position_ms = millis();
        portEXIT_CRITICAL(&s_snapshot_mux);
        return;
    }

    Serial.println("[LX Rx] unknown legacy payload length");
}

bool data_send_LX_extended(uint8_t subtype, const uint8_t *payload,
                           uint8_t payload_length)
{
    if ((payload_length != 0 && payload == nullptr) || payload_length > 64)
    {
        return false;
    }

    uint8_t packet[73] = {0};
    packet[0] = 0xBB;
    packet[1] = 0x10;
    packet[2] = 0xF1;
    packet[3] = 0x30;
    packet[4] = subtype;
    packet[5] = payload_length;
    if (payload_length != 0)
    {
        memcpy(packet + 6, payload, payload_length);
    }
    const size_t end_index = static_cast<size_t>(payload_length) + 6;
    packet[end_index] = 0xEE;

    uint8_t sc;
    uint8_t ac;
    lxDoubleChecksum(packet, end_index + 1, sc, ac);
    packet[end_index + 1] = sc;
    packet[end_index + 2] = ac;
    return write_lx_locked(packet, end_index + 3);
}

bool data_send_LX_car_pose(const uint8_t *payload, size_t payload_length)
{
    return payload_length == 22 &&
           data_send_LX_extended(LX_CAR_POSE, payload, 22);
}

bool data_send_LX_mission_request(const uint8_t *payload,
                                  size_t payload_length)
{
    return payload_length == 12 &&
           data_send_LX_extended(LX_MISSION_REQUEST, payload, 12);
}

bool data_send_LX_mission_abort(void)
{
    return data_send_LX_extended(LX_MISSION_ABORT, nullptr, 0);
}

bool data_send_LX_maintenance_reset(const uint8_t *payload,
                                    size_t payload_length)
{
    return payload_length == 8 &&
           data_send_LX_extended(LX_MAINTENANCE_RESET, payload, 8);
}

bool data_send_LX(uint8_t data_func, uint8_t data_message)
{
    uint8_t packet[8] = {0xBB, 0x10, 0xF1, data_func, data_message, 0xEE, 0, 0};
    uint8_t sc = 0;
    uint8_t ac = 0;
    for (size_t i = 0; i < 6; ++i)
    {
        sc = static_cast<uint8_t>(sc + packet[i]);
        ac = static_cast<uint8_t>(ac + sc);
    }
    packet[6] = sc;
    packet[7] = ac;
    return write_lx_locked(packet, sizeof(packet));
}

void get_flight_status_snapshot(FlightStatusSnapshot *snapshot)
{
    if (snapshot == nullptr)
    {
        return;
    }

    portENTER_CRITICAL(&s_snapshot_mux);
    snapshot->hasPosition = s_has_position;
    snapshot->taskRunning = s_task_running;
    snapshot->modeCode = s_mode_code;
    snapshot->xCm = s_ano_info.dis_x;
    snapshot->yCm = s_ano_info.dis_y;
    snapshot->zCm = s_ano_info.hight;
    // The legacy FC telemetry already carries yaw in 0.1 degree units.
    snapshot->yawDeciDegrees = normalize_yaw_deci_degrees(s_ano_info.yaw);
    snapshot->sourceTimeMs = s_last_position_ms;
    portEXIT_CRITICAL(&s_snapshot_mux);
}

bool flight_position_is_fresh(uint32_t now_ms, uint32_t max_age_ms)
{
    bool has_position;
    uint32_t last_position_ms;
    portENTER_CRITICAL(&s_snapshot_mux);
    has_position = s_has_position;
    last_position_ms = s_last_position_ms;
    portEXIT_CRITICAL(&s_snapshot_mux);
    return has_position &&
           static_cast<uint32_t>(now_ms - last_position_ms) <= max_age_ms;
}
