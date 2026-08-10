#include "uart_lora.h"

#include "main.h"
#include "uart_lx.h"

#include <Arduino.h>
#include <string.h>

using namespace gpio_lora_v2;

namespace {

static const uint32_t kFlightDataFreshMs = 500;
static const uint16_t kTaskSelectBeepMs = 1000;
// Set false to silence the task-related USB serial diagnostics.
static const bool kTaskSerialDebug = true;
static const size_t kPendingCount = 8;
static const size_t kAckQueueCount = 8;
static const size_t kStatusQueueCount = 8;

struct PendingRequest {
    bool active;
    Frame request;
    uint16_t missionId;
    uint32_t receivedMs;
};

struct QueuedAck {
    bool active;
    uint32_t order;
    uint8_t destination;
    uint8_t requestType;
    uint8_t requestSeq;
    AckResult result;
    uint8_t detail;
};

struct QueuedStatus {
    bool active;
    uint32_t order;
    uint8_t payload[12];
};

enum SelectedKind : uint8_t {
    SELECT_NONE = 0,
    SELECT_ACK,
    SELECT_STATUS,
    SELECT_TELEMETRY
};

struct SelectedTransmission {
    SelectedKind kind;
    size_t index;
    QueuedAck ack;
    QueuedStatus status;
};

StreamParser s_lora_parser(kFlightAddress);
RequestDeduplicator s_request_dedup;
SequenceGenerator s_tx_sequence(0);
CarPoseTimebase s_car_pose_timebase;
PendingRequest s_pending[kPendingCount] = {};
QueuedAck s_ack_queue[kAckQueueCount] = {};
QueuedStatus s_status_queue[kStatusQueueCount] = {};
portMUX_TYPE s_protocol_mux = portMUX_INITIALIZER_UNLOCKED;

uint32_t s_queue_order = 0;
uint32_t s_last_valid_lora_ms = 0;
bool s_seen_valid_lora_frame = false;
uint16_t s_last_maintenance_reset_id = 0;
bool s_seen_maintenance_reset = false;

bool send_v2(uint8_t type, uint8_t destination, uint8_t flags,
             const uint8_t *payload, uint8_t payload_length)
{
    if (payload_length > kMaxPayloadLength ||
        (payload_length != 0 && payload == nullptr))
    {
        return false;
    }

    if (loraTxMutex != nullptr &&
        xSemaphoreTake(loraTxMutex, pdMS_TO_TICKS(50)) != pdTRUE)
    {
        Serial.println("[LORA V2 Tx] mutex timeout");
        return false;
    }

    Frame frame;
    frame.type = type;
    frame.src = kFlightAddress;
    frame.dst = destination;
    frame.seq = s_tx_sequence.next();
    frame.flags = flags;
    frame.length = payload_length;
    if (payload_length != 0)
    {
        memcpy(frame.payload, payload, payload_length);
    }

    uint8_t encoded[kMaxFrameLength];
    size_t encoded_length = 0;
    const EncodeResult result =
        encodeFrame(frame, encoded, sizeof(encoded), encoded_length);
    const bool sent = result == ENCODE_OK &&
                      Lora.write(encoded, encoded_length) == encoded_length;
    if (result == ENCODE_OK)
    {
        Lora.flush();
    }
    else
    {
        Serial.printf("[LORA V2 Tx] encode error %u\n",
                      static_cast<unsigned>(result));
    }

    if (loraTxMutex != nullptr)
    {
        xSemaphoreGive(loraTxMutex);
    }
    return sent;
}

bool flags_are_legal(const Frame &frame)
{
    if ((frame.flags & 0xF8u) != 0 || frame.src == kBroadcastAddress)
    {
        return false;
    }
    if (frame.dst == kBroadcastAddress &&
        (frame.flags & ACK_REQUIRED) != 0)
    {
        return false;
    }
    return frame.type != ACK || (frame.flags & ACK_REQUIRED) == 0;
}

void queue_ack_locked(const Frame &request, AckResult result, uint8_t detail)
{
    for (size_t i = 0; i < kAckQueueCount; ++i)
    {
        QueuedAck &entry = s_ack_queue[i];
        if (entry.active && entry.destination == request.src &&
            entry.requestType == request.type &&
            entry.requestSeq == request.seq)
        {
            entry.result = result;
            entry.detail = detail;
            return;
        }
    }

    size_t selected = kAckQueueCount;
    uint32_t oldest_order = 0;
    for (size_t i = 0; i < kAckQueueCount; ++i)
    {
        if (!s_ack_queue[i].active)
        {
            selected = i;
            break;
        }
        if (selected == kAckQueueCount ||
            static_cast<int32_t>(s_ack_queue[i].order - oldest_order) < 0)
        {
            selected = i;
            oldest_order = s_ack_queue[i].order;
        }
    }

    QueuedAck &entry = s_ack_queue[selected];
    entry.active = true;
    entry.order = s_queue_order++;
    entry.destination = request.src;
    entry.requestType = request.type;
    entry.requestSeq = request.seq;
    entry.result = result;
    entry.detail = detail;
}

size_t find_pending_identity_locked(const Frame &request)
{
    for (size_t i = 0; i < kPendingCount; ++i)
    {
        if (s_pending[i].active &&
            s_pending[i].request.src == request.src &&
            s_pending[i].request.type == request.type &&
            s_pending[i].request.seq == request.seq)
        {
            return i;
        }
    }
    return kPendingCount;
}

size_t allocate_pending_locked(const Frame &request, uint16_t mission_id,
                               uint32_t now_ms)
{
    for (size_t i = 0; i < kPendingCount; ++i)
    {
        if (!s_pending[i].active ||
            static_cast<uint32_t>(now_ms - s_pending[i].receivedMs) >=
                kDedupWindowMs)
        {
            s_pending[i].active = true;
            s_pending[i].request = request;
            s_pending[i].missionId = mission_id;
            s_pending[i].receivedMs = now_ms;
            return i;
        }
    }
    return kPendingCount;
}

void reject_and_remember(const Frame &request, uint32_t now_ms,
                         AckResult result, uint8_t detail)
{
    portENTER_CRITICAL(&s_protocol_mux);
    s_request_dedup.remember(request, now_ms, result, detail);
    queue_ack_locked(request, result, detail);
    portEXIT_CRITICAL(&s_protocol_mux);
}

void handle_car_pose(const Frame &frame, uint32_t completed_ms)
{
    CarPosePayload pose;
    if (frame.src != kCarAddress || frame.dst != kBroadcastAddress ||
        frame.flags != 0 ||
        !decodeCarPosePayload(frame.payload, frame.length, pose) ||
        pose.coordinateFrame != 0x01 || (pose.poseFlags & 0xE0u) != 0 ||
        (((pose.poseFlags & (1u << 1)) != 0) && pose.calibrationId == 0) ||
        (((pose.poseFlags & (1u << 2)) == 0) &&
          (pose.vxCmPerSec != 0x7FFF || pose.vyCmPerSec != 0x7FFF)))
    {
        return;
    }

    // The raw 22 bytes are forwarded exactly as section 6.2 requires. The FC
    // remains the only component that decides whether this pose is usable.
    if (!data_send_LX_car_pose(frame.payload, frame.length))
    {
        Serial.println("[LX Tx] CAR_POSE forwarding failed");
    }

    portENTER_CRITICAL(&s_protocol_mux);
    s_car_pose_timebase.observePose(frame.seq, completed_ms);
    portEXIT_CRITICAL(&s_protocol_mux);
}

void handle_task_request(const Frame &request, uint32_t now_ms)
{
    if (request.src != kCarAddress || request.dst != kFlightAddress ||
        (request.flags & ACK_REQUIRED) == 0 ||
        (request.flags & URGENT) != 0 ||
        (request.flags & 0xF8u) != 0)
    {
        return;
    }

    AckResult old_result;
    uint8_t old_detail;
    portENTER_CRITICAL(&s_protocol_mux);
    if (s_request_dedup.find(request, now_ms, old_result, old_detail))
    {
        queue_ack_locked(request, old_result, old_detail);
        portEXIT_CRITICAL(&s_protocol_mux);
        return;
    }
    if (find_pending_identity_locked(request) != kPendingCount)
    {
        portEXIT_CRITICAL(&s_protocol_mux);
        return;
    }
    portEXIT_CRITICAL(&s_protocol_mux);

    CarTaskRequestPayload task;
    if (!decodeCarTaskRequestPayload(request.payload, request.length, task) ||
        (task.taskType != 1 && task.taskType != 2) ||
        task.requestFlags != 0x01 || task.reserved != 0)
    {
        reject_and_remember(request, now_ms, ACK_INVALID_PARAMETER, 0x06);
        return;
    }

    // A valid, newly received car task selection gets an audible indication.
    // Beep_Async keeps the LoRa receive task responsive while the buzzer runs.
    Beep_Async(task.taskType, kTaskSelectBeepMs);
    if (kTaskSerialDebug)
    {
        Serial.printf("[TASK] Task %u received and validated "
                      "(missionId=%u, seq=%u); beep x%u\n",
                      task.taskType, task.missionId, request.seq,
                      task.taskType);
    }

    if (!flight_position_is_fresh(now_ms, kFlightDataFreshMs))
    {
        if (kTaskSerialDebug)
        {
            Serial.printf("[TASK] Task %u not forwarded: flight-controller "
                          "position data is stale\n",
                          task.taskType);
        }
        reject_and_remember(request, now_ms, ACK_STATE_DISALLOWED, 0x05);
        return;
    }

    portENTER_CRITICAL(&s_protocol_mux);
    const size_t pending_index =
        allocate_pending_locked(request, task.missionId, now_ms);
    portEXIT_CRITICAL(&s_protocol_mux);
    if (pending_index == kPendingCount)
    {
        if (kTaskSerialDebug)
        {
            Serial.printf("[TASK] Task %u not forwarded: pending queue is full\n",
                          task.taskType);
        }
        reject_and_remember(request, now_ms, ACK_BUSY, 0);
        return;
    }

    if (!data_send_LX_mission_request(request.payload, request.length))
    {
        portENTER_CRITICAL(&s_protocol_mux);
        s_pending[pending_index].active = false;
        portEXIT_CRITICAL(&s_protocol_mux);
        if (kTaskSerialDebug)
        {
            Serial.printf("[TASK] Task %u forwarding to flight controller failed\n",
                          task.taskType);
        }
        reject_and_remember(request, now_ms, ACK_INTERNAL_ERROR, 0x05);
        return;
    }

    if (kTaskSerialDebug)
    {
        Serial.printf("[TASK] Task %u forwarded to flight controller; "
                      "awaiting response\n", task.taskType);
    }

    // No ACK is queued here. Only lora_on_mission_response() may complete it.
}

void handle_mission_abort(const Frame &request, uint32_t now_ms)
{
    if (request.src != kCarAddress || request.dst != kFlightAddress ||
        (request.flags & (ACK_REQUIRED | URGENT)) !=
            (ACK_REQUIRED | URGENT) ||
        (request.flags & 0xF8u) != 0 || request.length != 0)
    {
        return;
    }

    AckResult old_result;
    uint8_t old_detail;
    portENTER_CRITICAL(&s_protocol_mux);
    if (s_request_dedup.find(request, now_ms, old_result, old_detail))
    {
        queue_ack_locked(request, old_result, old_detail);
        portEXIT_CRITICAL(&s_protocol_mux);
        return;
    }
    if (find_pending_identity_locked(request) != kPendingCount)
    {
        portEXIT_CRITICAL(&s_protocol_mux);
        return;
    }
    portEXIT_CRITICAL(&s_protocol_mux);

    // 0x84 has no V2.3 payload definition. Preserve the wireless request in
    // the pending table and forward only a zero-payload safety event to the
    // flight controller. The ACK is queued only after the FC confirms that it
    // has consumed the event via MISSION_RESPONSE(RequestType=0x84).
    portENTER_CRITICAL(&s_protocol_mux);
    const size_t pending_index = allocate_pending_locked(request, 0, now_ms);
    portEXIT_CRITICAL(&s_protocol_mux);
    if (pending_index == kPendingCount)
    {
        reject_and_remember(request, now_ms, ACK_BUSY, 0);
        return;
    }

    if (!data_send_LX_mission_abort())
    {
        portENTER_CRITICAL(&s_protocol_mux);
        s_pending[pending_index].active = false;
        portEXIT_CRITICAL(&s_protocol_mux);
        reject_and_remember(request, now_ms, ACK_INTERNAL_ERROR, 0x05);
    }
}

void handle_maintenance_reset(const Frame &frame)
{
    MaintenanceResetPayload reset;
    if (frame.src != kCarAddress || frame.dst != kBroadcastAddress ||
        (frame.flags & ~RETRANSMISSION) != 0 ||
        !decodeMaintenanceResetPayload(frame.payload, frame.length, reset))
    {
        return;
    }

    if (s_seen_maintenance_reset &&
        reset.resetId == s_last_maintenance_reset_id)
    {
        // C.4: a retransmission refreshes link statistics only.
        return;
    }

    // V2.3 requires no ACK path here. A new reset is bridged once so the FC can
    // invalidate its calibration input and mission session.
    if (!data_send_LX_maintenance_reset(frame.payload, frame.length))
    {
        Serial.println("[LX Tx] MAINTENANCE_RESET forwarding failed");
        return;
    }
    s_last_maintenance_reset_id = reset.resetId;
    s_seen_maintenance_reset = true;
}

void send_telemetry(const FlightStatusSnapshot &snapshot)
{
    uint16_t status_flags = 0;
    status_flags |= 1u << 0;
    if (snapshot.taskRunning)
    {
        status_flags |= 1u << 1;
    }
    status_flags |= 1u << 2;
    if (snapshot.zCm > 20)
    {
        status_flags |= 1u << 4;
    }

    FlightTelemetryPayload telemetry = {
        status_flags,
        0x01,
        snapshot.modeCode,
        snapshot.xCm,
        snapshot.yCm,
        snapshot.zCm,
        snapshot.yawDeciDegrees,
        0xFFFF,
        snapshot.sourceTimeMs};
    uint8_t payload[24];
    if (encodeFlightTelemetryPayload(telemetry, payload, sizeof(payload)))
    {
        (void)send_v2(FLIGHT_TELEMETRY, kBroadcastAddress, 0, payload,
                      sizeof(payload));
    }
}

size_t oldest_ack_locked()
{
    size_t selected = kAckQueueCount;
    for (size_t i = 0; i < kAckQueueCount; ++i)
    {
        if (s_ack_queue[i].active &&
            (selected == kAckQueueCount ||
             static_cast<int32_t>(s_ack_queue[i].order -
                                  s_ack_queue[selected].order) < 0))
        {
            selected = i;
        }
    }
    return selected;
}

size_t oldest_status_locked()
{
    size_t selected = kStatusQueueCount;
    for (size_t i = 0; i < kStatusQueueCount; ++i)
    {
        if (s_status_queue[i].active &&
            (selected == kStatusQueueCount ||
             static_cast<int32_t>(s_status_queue[i].order -
                                  s_status_queue[selected].order) < 0))
        {
            selected = i;
        }
    }
    return selected;
}

SelectedTransmission select_for_current_slot(uint32_t now_ms)
{
    SelectedTransmission selected = {};
    selected.kind = SELECT_NONE;

    portENTER_CRITICAL(&s_protocol_mux);
    for (size_t i = 0; i < kPendingCount; ++i)
    {
        if (s_pending[i].active &&
            static_cast<uint32_t>(now_ms - s_pending[i].receivedMs) >=
                kDedupWindowMs)
        {
            s_pending[i].active = false;
        }
    }

    uint32_t cycle = 0;
    if (!s_car_pose_timebase.responseWindowOpen(now_ms, cycle))
    {
        portEXIT_CRITICAL(&s_protocol_mux);
        return selected;
    }

    const size_t ack_index = oldest_ack_locked();
    if (ack_index != kAckQueueCount)
    {
        selected.kind = SELECT_ACK;
        selected.index = ack_index;
        selected.ack = s_ack_queue[ack_index];
    }
    else
    {
        const size_t status_index = oldest_status_locked();
        if (status_index != kStatusQueueCount)
        {
            selected.kind = SELECT_STATUS;
            selected.index = status_index;
            selected.status = s_status_queue[status_index];
        }
        else if (s_car_pose_timebase.telemetryEligible(now_ms))
        {
            selected.kind = SELECT_TELEMETRY;
        }
    }

    if (selected.kind != SELECT_NONE)
    {
        s_car_pose_timebase.consume(cycle);
    }
    portEXIT_CRITICAL(&s_protocol_mux);
    return selected;
}

void finish_selected(const SelectedTransmission &selected, bool sent)
{
    if (!sent)
    {
        return;
    }

    portENTER_CRITICAL(&s_protocol_mux);
    if (selected.kind == SELECT_ACK &&
        selected.index < kAckQueueCount &&
        s_ack_queue[selected.index].active &&
        s_ack_queue[selected.index].order == selected.ack.order)
    {
        s_ack_queue[selected.index].active = false;
    }
    else if (selected.kind == SELECT_STATUS &&
             selected.index < kStatusQueueCount &&
             s_status_queue[selected.index].active &&
             s_status_queue[selected.index].order == selected.status.order)
    {
        s_status_queue[selected.index].active = false;
    }
    portEXIT_CRITICAL(&s_protocol_mux);
}

} // namespace

void data_receive_Lora()
{
    int processed = 0;
    while (Lora.available() > 0 && processed++ < 128)
    {
        const uint8_t byte_in = static_cast<uint8_t>(Lora.read());
        const uint32_t now_ms = millis();
        ParseOutput output;
        if (!s_lora_parser.feed(byte_in, now_ms, output))
        {
            continue;
        }

        if (output.kind == PARSED_LEGACY)
        {
            // Old AA BB QR messages are not part of this competition protocol.
            continue;
        }
        if (output.kind != PARSED_V2 || !flags_are_legal(output.frame))
        {
            continue;
        }

        s_seen_valid_lora_frame = true;
        s_last_valid_lora_ms = now_ms;
        RECEIVE_FLAG_Lora = 1;

        switch (output.frame.type)
        {
        case CAR_POSE:
            handle_car_pose(output.frame, now_ms);
            break;
        case CAR_TASK_REQUEST:
            handle_task_request(output.frame, now_ms);
            break;
        case MISSION_ABORT:
            handle_mission_abort(output.frame, now_ms);
            break;
        case MAINTENANCE_RESET:
            handle_maintenance_reset(output.frame);
            break;
        default:
            break;
        }
    }

    if (s_seen_valid_lora_frame &&
        static_cast<uint32_t>(millis() - s_last_valid_lora_ms) > 1000)
    {
        RECEIVE_FLAG_Lora = 0;
    }
}

void lora_protocol_service()
{
    const uint32_t now_ms = millis();
    const SelectedTransmission selected = select_for_current_slot(now_ms);
    if (selected.kind == SELECT_NONE)
    {
        return;
    }

    bool sent = false;
    if (selected.kind == SELECT_ACK)
    {
        const AckPayload ack = {selected.ack.requestType,
                                selected.ack.requestSeq,
                                selected.ack.result,
                                selected.ack.detail};
        uint8_t payload[4];
        sent = encodeAckPayload(ack, payload, sizeof(payload)) &&
               send_v2(ACK, selected.ack.destination, 0, payload,
                       sizeof(payload));
    }
    else if (selected.kind == SELECT_STATUS)
    {
        sent = send_v2(MISSION_STATUS, kBroadcastAddress, 0,
                       selected.status.payload,
                       sizeof(selected.status.payload));
    }
    else if (selected.kind == SELECT_TELEMETRY)
    {
        FlightStatusSnapshot snapshot;
        get_flight_status_snapshot(&snapshot);
        if (snapshot.hasPosition &&
            static_cast<uint32_t>(now_ms - snapshot.sourceTimeMs) <=
                kFlightDataFreshMs)
        {
            send_telemetry(snapshot);
            sent = true;
        }
    }
    finish_selected(selected, sent);
}

void lora_on_mission_response(const MissionResponsePayload &response)
{
    if ((response.requestType != CAR_TASK_REQUEST &&
         response.requestType != MISSION_ABORT) ||
        response.result > ACK_INTERNAL_ERROR || response.detail > 0x06 ||
        response.reserved != 0 ||
        (response.requestType == MISSION_ABORT && response.missionId != 0))
    {
        return;
    }

    const uint32_t now_ms = millis();
    portENTER_CRITICAL(&s_protocol_mux);
    for (size_t i = 0; i < kPendingCount; ++i)
    {
        PendingRequest &pending = s_pending[i];
        if (pending.active && pending.request.type == response.requestType &&
            pending.missionId == response.missionId &&
            static_cast<uint32_t>(now_ms - pending.receivedMs) <
                kDedupWindowMs)
        {
            s_request_dedup.remember(pending.request, pending.receivedMs,
                                     response.result, response.detail);
            queue_ack_locked(pending.request, response.result,
                             response.detail);
            pending.active = false;
            portEXIT_CRITICAL(&s_protocol_mux);
            if (kTaskSerialDebug)
            {
                Serial.printf("[TASK] Flight-controller task response "
                              "(missionId=%u, result=%u, detail=%u)\n",
                              response.missionId, response.result,
                              response.detail);
            }
            return;
        }
    }
    portEXIT_CRITICAL(&s_protocol_mux);
    Serial.printf("[LX Rx] unmatched MISSION_RESPONSE Type:0x%02X Id:%u\n",
                  response.requestType, response.missionId);
}

bool lora_queue_mission_status(const uint8_t *payload, size_t payload_length)
{
    MissionStatusPayload status;
    if (payload == nullptr || payload_length != 12 ||
        !decodeMissionStatusPayload(payload, payload_length, status) ||
        status.reserved != 0)
    {
        return false;
    }

    bool queued = false;
    portENTER_CRITICAL(&s_protocol_mux);
    for (size_t i = 0; i < kStatusQueueCount; ++i)
    {
        if (!s_status_queue[i].active)
        {
            s_status_queue[i].active = true;
            s_status_queue[i].order = s_queue_order++;
            memcpy(s_status_queue[i].payload, payload, 12);
            queued = true;
            break;
        }
    }
    portEXIT_CRITICAL(&s_protocol_mux);
    if (!queued)
    {
        Serial.println("[LORA] MISSION_STATUS queue full");
    }
    return queued;
}
