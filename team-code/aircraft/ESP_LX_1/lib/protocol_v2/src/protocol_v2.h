#ifndef GPIO_LORA_PROTOCOL_V2_H
#define GPIO_LORA_PROTOCOL_V2_H

#include <stddef.h>
#include <stdint.h>

namespace gpio_lora_v2 {

static const uint8_t kHeader0 = 0xAA;
static const uint8_t kHeaderV2 = 0x55;
static const uint8_t kHeaderLegacy = 0xBB;
static const uint8_t kVersion = 0x02;
static const uint8_t kBroadcastAddress = 0x10;
static const uint8_t kFlightAddress = 0x20;
static const uint8_t kFlightControllerAddress = 0x21;
static const uint8_t kCarAddress = 0x30;
static const uint8_t kGroundAddress = 0x40;
static const size_t kMaxPayloadLength = 64;
static const size_t kFrameOverhead = 11;
static const size_t kMaxFrameLength = kMaxPayloadLength + kFrameOverhead;
static const uint32_t kInterByteTimeoutMs = 100;
static const uint32_t kDedupWindowMs = 5000;

enum MessageType : uint8_t {
    OBJECT_REPORT = 0x01,
    FLIGHT_TELEMETRY = 0x02,
    HEARTBEAT = 0x03,
    SYSTEM_COMMAND = 0x10,
    ACK = 0x11,
    EVENT_REPORT = 0x12,
    TARGET_SELECT = 0x13,
    CAR_POSE = 0x80,
    CAR_TASK_REQUEST = 0x81,
    MISSION_STATUS = 0x82,
    CALIBRATION_SET = 0x83,
    MISSION_ABORT = 0x84,
    MAINTENANCE_RESET = 0x85
};

enum FrameFlag : uint8_t {
    ACK_REQUIRED = 0x01,
    RETRANSMISSION = 0x02,
    URGENT = 0x04
};

enum Command : uint8_t {
    START_MISSION = 0x01,
    STOP_MISSION = 0x02,
    START_RECORDING = 0x20,
    STOP_RECORDING = 0x21
};

enum AckResult : uint8_t {
    ACK_ACCEPTED = 0x00,
    ACK_DUPLICATE = 0x01,
    ACK_BUSY = 0x02,
    ACK_STATE_DISALLOWED = 0x03,
    ACK_UNSUPPORTED = 0x04,
    ACK_INVALID_PARAMETER = 0x05,
    ACK_INTERNAL_ERROR = 0x06
};

struct Frame {
    uint8_t version;
    uint8_t type;
    uint8_t src;
    uint8_t dst;
    uint8_t seq;
    uint8_t flags;
    uint8_t length;
    uint8_t payload[kMaxPayloadLength];

    Frame();
};

struct LegacyFrame {
    uint8_t destination;
    uint8_t objectCode;
    int16_t xCm;
    int16_t yCm;
};

enum EncodeResult : uint8_t {
    ENCODE_OK = 0,
    ENCODE_BUFFER_TOO_SMALL,
    ENCODE_INVALID_VERSION,
    ENCODE_INVALID_LENGTH,
    ENCODE_INVALID_SOURCE,
    ENCODE_INVALID_FLAGS
};

uint16_t crc16CcittFalse(const uint8_t *data, size_t length);
void lxDoubleChecksum(const uint8_t *data, size_t length, uint8_t &sc,
                      uint8_t &ac);
EncodeResult encodeFrame(const Frame &frame, uint8_t *output, size_t capacity,
                         size_t &written);

struct ObjectReportPayload {
    uint8_t objectKind;
    uint16_t objectCode;
    uint16_t trackId;
    uint8_t coordinateFrame;
    uint8_t objectFlags;
    int32_t xCm;
    int32_t yCm;
    int32_t zCm;
    uint8_t confidence;
    uint32_t sourceTimeMs;
};

struct FlightTelemetryPayload {
    uint16_t statusFlags;
    uint8_t coordinateFrame;
    uint8_t modeCode;
    int32_t xCm;
    int32_t yCm;
    int32_t zCm;
    int16_t yawDeciDegrees;
    uint16_t groundSpeedCmPerSec;
    uint32_t sourceTimeMs;
};

struct HeartbeatPayload {
    uint8_t deviceStatus;
    uint8_t errorCode;
    uint32_t uptimeMs;
};

struct SystemCommandPayload {
    uint8_t command;
    uint8_t commandFlags;
    uint16_t argument;
};

struct AckPayload {
    uint8_t requestType;
    uint8_t requestSeq;
    AckResult result;
    uint8_t detail;
};

struct CarPosePayload {
    uint8_t coordinateFrame;
    uint8_t poseFlags;
    uint16_t calibrationId;
    int32_t xCm;
    int32_t yCm;
    int16_t yawDeciDegrees;
    int16_t vxCmPerSec;
    int16_t vyCmPerSec;
    uint32_t sourceTimeMs;
};

struct CarTaskRequestPayload {
    uint8_t taskType;
    uint8_t requestFlags;
    uint16_t missionId;
    uint16_t calibrationId;
    uint16_t reserved;
    uint32_t sourceTimeMs;
};

struct MissionStatusPayload {
    uint8_t taskType;
    uint8_t stage;
    uint16_t statusFlags;
    uint16_t missionId;
    uint8_t errorCode;
    uint8_t reserved;
    uint32_t sourceTimeMs;
};

struct MaintenanceResetPayload {
    uint16_t resetId;
    uint8_t resetFlags;
    uint8_t reserved;
    uint32_t sourceTimeMs;
};

// Section 6.2 wired-LX response payload (not a LoRa message payload).
struct MissionResponsePayload {
    uint8_t requestType;
    uint16_t missionId;
    AckResult result;
    uint8_t detail;
    uint8_t reserved;
};

struct TargetSelectPayload {
    uint8_t objectKind;
    uint16_t objectCode;
};

bool encodeObjectReportPayload(const ObjectReportPayload &value, uint8_t *payload,
                               size_t capacity);
bool decodeObjectReportPayload(const uint8_t *payload, size_t length,
                               ObjectReportPayload &value);
bool encodeFlightTelemetryPayload(const FlightTelemetryPayload &value,
                                  uint8_t *payload, size_t capacity);
bool decodeFlightTelemetryPayload(const uint8_t *payload, size_t length,
                                  FlightTelemetryPayload &value);
bool encodeHeartbeatPayload(const HeartbeatPayload &value, uint8_t *payload,
                            size_t capacity);
bool decodeHeartbeatPayload(const uint8_t *payload, size_t length,
                            HeartbeatPayload &value);
bool encodeSystemCommandPayload(const SystemCommandPayload &value,
                                uint8_t *payload, size_t capacity);
bool decodeSystemCommandPayload(const uint8_t *payload, size_t length,
                                SystemCommandPayload &value);
bool encodeAckPayload(const AckPayload &value, uint8_t *payload, size_t capacity);
bool decodeAckPayload(const uint8_t *payload, size_t length, AckPayload &value);
bool encodeCarPosePayload(const CarPosePayload &value, uint8_t *payload,
                          size_t capacity);
bool decodeCarPosePayload(const uint8_t *payload, size_t length,
                          CarPosePayload &value);
bool encodeCarTaskRequestPayload(const CarTaskRequestPayload &value,
                                 uint8_t *payload, size_t capacity);
bool decodeCarTaskRequestPayload(const uint8_t *payload, size_t length,
                                 CarTaskRequestPayload &value);
bool encodeMissionStatusPayload(const MissionStatusPayload &value,
                                uint8_t *payload, size_t capacity);
bool decodeMissionStatusPayload(const uint8_t *payload, size_t length,
                                MissionStatusPayload &value);
bool encodeMaintenanceResetPayload(const MaintenanceResetPayload &value,
                                   uint8_t *payload, size_t capacity);
bool decodeMaintenanceResetPayload(const uint8_t *payload, size_t length,
                                   MaintenanceResetPayload &value);
bool encodeMissionResponsePayload(const MissionResponsePayload &value,
                                  uint8_t *payload, size_t capacity);
bool decodeMissionResponsePayload(const uint8_t *payload, size_t length,
                                  MissionResponsePayload &value);
bool encodeTargetSelectPayload(const TargetSelectPayload &value, uint8_t *payload,
                               size_t capacity);
bool decodeTargetSelectPayload(const uint8_t *payload, size_t length,
                               TargetSelectPayload &value);

enum ParsedKind : uint8_t {
    PARSED_NONE = 0,
    PARSED_V2,
    PARSED_LEGACY
};

struct ParseOutput {
    ParsedKind kind;
    Frame frame;
    LegacyFrame legacy;

    ParseOutput();
};

struct ParserStats {
    uint32_t validV2Frames;
    uint32_t validLegacyFrames;
    uint32_t crcErrors;
    uint32_t lengthErrors;
    uint32_t versionErrors;
    uint32_t destinationErrors;
    uint32_t framingErrors;
    uint32_t timeoutErrors;

    ParserStats();
};

class StreamParser {
public:
    // localAddress == 0 disables destination filtering.
    explicit StreamParser(uint8_t localAddress = 0);

    bool feed(uint8_t byte, uint32_t nowMs, ParseOutput &output);
    void reset();
    const ParserStats &stats() const;

private:
    bool processBuffered(ParseOutput &output);
    void recoverFromNextHeader();
    void consume(size_t length);
    bool destinationAccepted(uint8_t destination) const;

    uint8_t localAddress_;
    uint8_t buffer_[kMaxFrameLength];
    size_t count_;
    uint32_t lastByteMs_;
    bool hasLastByteTime_;
    ParserStats stats_;
};

class SequenceGenerator {
public:
    explicit SequenceGenerator(uint8_t initial = 0);
    uint8_t next();
    uint8_t peek() const;

private:
    uint8_t next_;
};

// Section 7 response-slot tracker. A recent CAR_POSE anchors 100 ms cycles;
// cycles may be extrapolated while a task request temporarily replaces pose.
class CarPoseTimebase {
public:
    CarPoseTimebase();
    void observePose(uint8_t seq, uint32_t completedMs);
    bool responseWindowOpen(uint32_t nowMs, uint32_t &cycle) const;
    bool telemetryEligible(uint32_t nowMs) const;
    void consume(uint32_t cycle);

private:
    bool active_;
    uint8_t seq_;
    uint32_t anchorMs_;
    uint32_t consumedCycle_;
};

class RequestDeduplicator {
public:
    RequestDeduplicator();

    // Returns true when Src + Type + Seq is still inside the five-second window.
    // A previously accepted request is reported as ACK_DUPLICATE; rejected requests
    // repeat their original result and detail.
    bool find(const Frame &request, uint32_t nowMs, AckResult &result,
              uint8_t &detail) const;
    void remember(const Frame &request, uint32_t nowMs, AckResult result,
                  uint8_t detail);
    void clear();

private:
    struct Entry {
        bool active;
        uint8_t src;
        uint8_t type;
        uint8_t seq;
        uint32_t firstSeenMs;
        AckResult result;
        uint8_t detail;
    };

    static const size_t kEntryCount = 8;
    Entry entries_[kEntryCount];
};

enum FlightActionKind : uint8_t {
    FLIGHT_ACTION_NONE = 0,
    FLIGHT_ACTION_START,
    FLIGHT_ACTION_STOP,
    FLIGHT_ACTION_SELECT_TARGET
};

struct FlightAction {
    FlightActionKind kind;
    uint8_t objectKind;
    uint16_t objectCode;

    FlightAction();
};

struct RequestValidation {
    AckResult result;
    uint8_t detail;
    FlightAction action;
};

RequestValidation validateFlightRequest(const Frame &request);

enum RetryPollResult : uint8_t {
    RETRY_NOT_DUE = 0,
    RETRY_FRAME_READY,
    RETRY_EXHAUSTED
};

enum AckMatchResult : uint8_t {
    ACK_NOT_MATCHED = 0,
    ACK_MATCHED_SUCCESS,
    ACK_MATCHED_REJECTED
};

class ReliableRequest {
public:
    ReliableRequest(uint32_t ackTimeoutMs = 500, uint8_t maxAttempts = 3);

    bool start(const Frame &request, uint32_t nowMs);
    RetryPollResult poll(uint32_t nowMs, Frame &retryFrame);
    AckMatchResult acceptAck(const Frame &ackFrame, AckPayload *ack = 0);
    bool active() const;
    uint8_t attempts() const;

private:
    Frame request_;
    uint32_t ackTimeoutMs_;
    uint32_t lastSendMs_;
    uint8_t maxAttempts_;
    uint8_t attempts_;
    bool active_;
};

} // namespace gpio_lora_v2

#endif
