#include "protocol_v2.h"

#include <string.h>

namespace gpio_lora_v2 {

namespace {

void putU16(uint8_t *out, uint16_t value)
{
    out[0] = static_cast<uint8_t>(value >> 8);
    out[1] = static_cast<uint8_t>(value);
}

void putU32(uint8_t *out, uint32_t value)
{
    out[0] = static_cast<uint8_t>(value >> 24);
    out[1] = static_cast<uint8_t>(value >> 16);
    out[2] = static_cast<uint8_t>(value >> 8);
    out[3] = static_cast<uint8_t>(value);
}

uint16_t getU16(const uint8_t *in)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(in[0]) << 8) | in[1]);
}

uint32_t getU32(const uint8_t *in)
{
    return (static_cast<uint32_t>(in[0]) << 24) |
           (static_cast<uint32_t>(in[1]) << 16) |
           (static_cast<uint32_t>(in[2]) << 8) |
           static_cast<uint32_t>(in[3]);
}

bool isQrCode(uint16_t code)
{
    return code == 0x00CA || code == 0x00CB || code == 0x00CC;
}

} // namespace

Frame::Frame()
    : version(kVersion), type(0), src(0), dst(0), seq(0), flags(0), length(0)
{
    memset(payload, 0, sizeof(payload));
}

ParseOutput::ParseOutput() : kind(PARSED_NONE), legacy()
{
    memset(&legacy, 0, sizeof(legacy));
}

ParserStats::ParserStats()
    : validV2Frames(0), validLegacyFrames(0), crcErrors(0), lengthErrors(0),
      versionErrors(0), destinationErrors(0), framingErrors(0), timeoutErrors(0)
{
}

FlightAction::FlightAction()
    : kind(FLIGHT_ACTION_NONE), objectKind(0), objectCode(0)
{
}

uint16_t crc16CcittFalse(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) != 0
                      ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                      : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

void lxDoubleChecksum(const uint8_t *data, size_t length, uint8_t &sc,
                      uint8_t &ac)
{
    sc = 0;
    ac = 0;
    if (data == 0) return;
    for (size_t i = 0; i < length; ++i) {
        sc = static_cast<uint8_t>(sc + data[i]);
        ac = static_cast<uint8_t>(ac + sc);
    }
}

EncodeResult encodeFrame(const Frame &frame, uint8_t *output, size_t capacity,
                         size_t &written)
{
    written = 0;
    if (frame.version != kVersion) {
        return ENCODE_INVALID_VERSION;
    }
    if (frame.length > kMaxPayloadLength) {
        return ENCODE_INVALID_LENGTH;
    }
    if (frame.src == kBroadcastAddress) {
        return ENCODE_INVALID_SOURCE;
    }
    if ((frame.flags & 0xF8) != 0 ||
        (frame.dst == kBroadcastAddress && (frame.flags & ACK_REQUIRED) != 0) ||
        (frame.type == ACK && (frame.flags & ACK_REQUIRED) != 0)) {
        return ENCODE_INVALID_FLAGS;
    }

    const size_t total = static_cast<size_t>(frame.length) + kFrameOverhead;
    if (output == 0 || capacity < total) {
        return ENCODE_BUFFER_TOO_SMALL;
    }

    output[0] = kHeader0;
    output[1] = kHeaderV2;
    output[2] = frame.version;
    output[3] = frame.type;
    output[4] = frame.src;
    output[5] = frame.dst;
    output[6] = frame.seq;
    output[7] = frame.flags;
    output[8] = frame.length;
    if (frame.length != 0) {
        memcpy(output + 9, frame.payload, frame.length);
    }

    const uint16_t crc = crc16CcittFalse(output + 2, 7 + frame.length);
    output[9 + frame.length] = static_cast<uint8_t>(crc >> 8);
    output[10 + frame.length] = static_cast<uint8_t>(crc);
    written = total;
    return ENCODE_OK;
}

bool encodeObjectReportPayload(const ObjectReportPayload &v, uint8_t *p, size_t n)
{
    if (p == 0 || n < 24) return false;
    p[0] = v.objectKind;
    putU16(p + 1, v.objectCode);
    putU16(p + 3, v.trackId);
    p[5] = v.coordinateFrame;
    p[6] = v.objectFlags;
    putU32(p + 7, static_cast<uint32_t>(v.xCm));
    putU32(p + 11, static_cast<uint32_t>(v.yCm));
    putU32(p + 15, static_cast<uint32_t>(v.zCm));
    p[19] = v.confidence;
    putU32(p + 20, v.sourceTimeMs);
    return true;
}

bool decodeObjectReportPayload(const uint8_t *p, size_t n, ObjectReportPayload &v)
{
    if (p == 0 || n != 24) return false;
    v.objectKind = p[0];
    v.objectCode = getU16(p + 1);
    v.trackId = getU16(p + 3);
    v.coordinateFrame = p[5];
    v.objectFlags = p[6];
    v.xCm = static_cast<int32_t>(getU32(p + 7));
    v.yCm = static_cast<int32_t>(getU32(p + 11));
    v.zCm = static_cast<int32_t>(getU32(p + 15));
    v.confidence = p[19];
    v.sourceTimeMs = getU32(p + 20);
    return true;
}

bool encodeFlightTelemetryPayload(const FlightTelemetryPayload &v, uint8_t *p,
                                  size_t n)
{
    if (p == 0 || n < 24) return false;
    putU16(p, v.statusFlags);
    p[2] = v.coordinateFrame;
    p[3] = v.modeCode;
    putU32(p + 4, static_cast<uint32_t>(v.xCm));
    putU32(p + 8, static_cast<uint32_t>(v.yCm));
    putU32(p + 12, static_cast<uint32_t>(v.zCm));
    putU16(p + 16, static_cast<uint16_t>(v.yawDeciDegrees));
    putU16(p + 18, v.groundSpeedCmPerSec);
    putU32(p + 20, v.sourceTimeMs);
    return true;
}

bool decodeFlightTelemetryPayload(const uint8_t *p, size_t n,
                                  FlightTelemetryPayload &v)
{
    if (p == 0 || n != 24) return false;
    v.statusFlags = getU16(p);
    v.coordinateFrame = p[2];
    v.modeCode = p[3];
    v.xCm = static_cast<int32_t>(getU32(p + 4));
    v.yCm = static_cast<int32_t>(getU32(p + 8));
    v.zCm = static_cast<int32_t>(getU32(p + 12));
    v.yawDeciDegrees = static_cast<int16_t>(getU16(p + 16));
    v.groundSpeedCmPerSec = getU16(p + 18);
    v.sourceTimeMs = getU32(p + 20);
    return true;
}

bool encodeHeartbeatPayload(const HeartbeatPayload &v, uint8_t *p, size_t n)
{
    if (p == 0 || n < 8) return false;
    p[0] = v.deviceStatus;
    p[1] = v.errorCode;
    p[2] = 0;
    p[3] = 0;
    putU32(p + 4, v.uptimeMs);
    return true;
}

bool decodeHeartbeatPayload(const uint8_t *p, size_t n, HeartbeatPayload &v)
{
    if (p == 0 || n != 8 || p[2] != 0 || p[3] != 0) return false;
    v.deviceStatus = p[0];
    v.errorCode = p[1];
    v.uptimeMs = getU32(p + 4);
    return true;
}

bool encodeSystemCommandPayload(const SystemCommandPayload &v, uint8_t *p,
                                size_t n)
{
    if (p == 0 || n < 4) return false;
    p[0] = v.command;
    p[1] = v.commandFlags;
    putU16(p + 2, v.argument);
    return true;
}

bool decodeSystemCommandPayload(const uint8_t *p, size_t n,
                                SystemCommandPayload &v)
{
    if (p == 0 || n != 4) return false;
    v.command = p[0];
    v.commandFlags = p[1];
    v.argument = getU16(p + 2);
    return true;
}

bool encodeAckPayload(const AckPayload &v, uint8_t *p, size_t n)
{
    if (p == 0 || n < 4) return false;
    p[0] = v.requestType;
    p[1] = v.requestSeq;
    p[2] = static_cast<uint8_t>(v.result);
    p[3] = v.detail;
    return true;
}

bool decodeAckPayload(const uint8_t *p, size_t n, AckPayload &v)
{
    if (p == 0 || n != 4 || p[2] > ACK_INTERNAL_ERROR) return false;
    v.requestType = p[0];
    v.requestSeq = p[1];
    v.result = static_cast<AckResult>(p[2]);
    v.detail = p[3];
    return true;
}

bool encodeCarPosePayload(const CarPosePayload &v, uint8_t *p, size_t n)
{
    if (p == 0 || n < 22) return false;
    p[0] = v.coordinateFrame;
    p[1] = v.poseFlags;
    putU16(p + 2, v.calibrationId);
    putU32(p + 4, static_cast<uint32_t>(v.xCm));
    putU32(p + 8, static_cast<uint32_t>(v.yCm));
    putU16(p + 12, static_cast<uint16_t>(v.yawDeciDegrees));
    putU16(p + 14, static_cast<uint16_t>(v.vxCmPerSec));
    putU16(p + 16, static_cast<uint16_t>(v.vyCmPerSec));
    putU32(p + 18, v.sourceTimeMs);
    return true;
}

bool decodeCarPosePayload(const uint8_t *p, size_t n, CarPosePayload &v)
{
    if (p == 0 || n != 22) return false;
    v.coordinateFrame = p[0];
    v.poseFlags = p[1];
    v.calibrationId = getU16(p + 2);
    v.xCm = static_cast<int32_t>(getU32(p + 4));
    v.yCm = static_cast<int32_t>(getU32(p + 8));
    v.yawDeciDegrees = static_cast<int16_t>(getU16(p + 12));
    v.vxCmPerSec = static_cast<int16_t>(getU16(p + 14));
    v.vyCmPerSec = static_cast<int16_t>(getU16(p + 16));
    v.sourceTimeMs = getU32(p + 18);
    return true;
}

bool encodeCarTaskRequestPayload(const CarTaskRequestPayload &v, uint8_t *p,
                                 size_t n)
{
    if (p == 0 || n < 12) return false;
    p[0] = v.taskType;
    p[1] = v.requestFlags;
    putU16(p + 2, v.missionId);
    putU16(p + 4, v.calibrationId);
    putU16(p + 6, v.reserved);
    putU32(p + 8, v.sourceTimeMs);
    return true;
}

bool decodeCarTaskRequestPayload(const uint8_t *p, size_t n,
                                 CarTaskRequestPayload &v)
{
    if (p == 0 || n != 12) return false;
    v.taskType = p[0];
    v.requestFlags = p[1];
    v.missionId = getU16(p + 2);
    v.calibrationId = getU16(p + 4);
    v.reserved = getU16(p + 6);
    v.sourceTimeMs = getU32(p + 8);
    return true;
}

bool encodeMissionStatusPayload(const MissionStatusPayload &v, uint8_t *p,
                                size_t n)
{
    if (p == 0 || n < 12) return false;
    p[0] = v.taskType;
    p[1] = v.stage;
    putU16(p + 2, v.statusFlags);
    putU16(p + 4, v.missionId);
    p[6] = v.errorCode;
    p[7] = v.reserved;
    putU32(p + 8, v.sourceTimeMs);
    return true;
}

bool decodeMissionStatusPayload(const uint8_t *p, size_t n,
                                MissionStatusPayload &v)
{
    if (p == 0 || n != 12) return false;
    v.taskType = p[0];
    v.stage = p[1];
    v.statusFlags = getU16(p + 2);
    v.missionId = getU16(p + 4);
    v.errorCode = p[6];
    v.reserved = p[7];
    v.sourceTimeMs = getU32(p + 8);
    return true;
}

bool encodeMaintenanceResetPayload(const MaintenanceResetPayload &v, uint8_t *p,
                                   size_t n)
{
    if (p == 0 || n < 8 || v.resetId == 0 || v.resetFlags != 0x01 ||
        v.reserved != 0) {
        return false;
    }
    putU16(p, v.resetId);
    p[2] = v.resetFlags;
    p[3] = v.reserved;
    putU32(p + 4, v.sourceTimeMs);
    return true;
}

bool decodeMaintenanceResetPayload(const uint8_t *p, size_t n,
                                   MaintenanceResetPayload &v)
{
    if (p == 0 || n != 8 || getU16(p) == 0 || p[2] != 0x01 || p[3] != 0) {
        return false;
    }
    v.resetId = getU16(p);
    v.resetFlags = p[2];
    v.reserved = p[3];
    v.sourceTimeMs = getU32(p + 4);
    return true;
}

bool encodeMissionResponsePayload(const MissionResponsePayload &v, uint8_t *p,
                                  size_t n)
{
    if (p == 0 || n < 6) return false;
    p[0] = v.requestType;
    putU16(p + 1, v.missionId);
    p[3] = static_cast<uint8_t>(v.result);
    p[4] = v.detail;
    p[5] = v.reserved;
    return true;
}

bool decodeMissionResponsePayload(const uint8_t *p, size_t n,
                                  MissionResponsePayload &v)
{
    if (p == 0 || n != 6 || p[3] > ACK_INTERNAL_ERROR) return false;
    v.requestType = p[0];
    v.missionId = getU16(p + 1);
    v.result = static_cast<AckResult>(p[3]);
    v.detail = p[4];
    v.reserved = p[5];
    return true;
}

bool encodeTargetSelectPayload(const TargetSelectPayload &v, uint8_t *p, size_t n)
{
    if (p == 0 || n < 4) return false;
    p[0] = v.objectKind;
    p[1] = 0;
    putU16(p + 2, v.objectCode);
    return true;
}

bool decodeTargetSelectPayload(const uint8_t *p, size_t n, TargetSelectPayload &v)
{
    if (p == 0 || n != 4 || p[1] != 0) return false;
    v.objectKind = p[0];
    v.objectCode = getU16(p + 2);
    return true;
}

StreamParser::StreamParser(uint8_t localAddress)
    : localAddress_(localAddress), count_(0), lastByteMs_(0),
      hasLastByteTime_(false)
{
    memset(buffer_, 0, sizeof(buffer_));
}

void StreamParser::reset()
{
    count_ = 0;
    hasLastByteTime_ = false;
}

const ParserStats &StreamParser::stats() const
{
    return stats_;
}

bool StreamParser::destinationAccepted(uint8_t destination) const
{
    return localAddress_ == 0 || destination == localAddress_ ||
           destination == kBroadcastAddress;
}

void StreamParser::consume(size_t length)
{
    if (length >= count_) {
        count_ = 0;
        return;
    }
    memmove(buffer_, buffer_ + length, count_ - length);
    count_ -= length;
}

void StreamParser::recoverFromNextHeader()
{
    size_t next = 1;
    while (next < count_ && buffer_[next] != kHeader0) {
        ++next;
    }
    if (next >= count_) {
        count_ = 0;
    } else {
        consume(next);
    }
}

bool StreamParser::processBuffered(ParseOutput &output)
{
    while (count_ != 0) {
        if (buffer_[0] != kHeader0) {
            recoverFromNextHeader();
            continue;
        }
        if (count_ < 2) return false;

        if (buffer_[1] != kHeaderLegacy && buffer_[1] != kHeaderV2) {
            recoverFromNextHeader();
            continue;
        }

        if (buffer_[1] == kHeaderLegacy) {
            if (count_ < 9) return false;
            if (buffer_[8] != 0xFF) {
                ++stats_.framingErrors;
                recoverFromNextHeader();
                continue;
            }
            if (!destinationAccepted(buffer_[2])) {
                ++stats_.destinationErrors;
                consume(9);
                continue;
            }

            output.kind = PARSED_LEGACY;
            output.legacy.destination = buffer_[2];
            output.legacy.objectCode = buffer_[3];
            output.legacy.xCm = static_cast<int16_t>(getU16(buffer_ + 4));
            output.legacy.yCm = static_cast<int16_t>(getU16(buffer_ + 6));
            ++stats_.validLegacyFrames;
            consume(9);
            return true;
        }

        if (count_ >= 3 && buffer_[2] != kVersion) {
            ++stats_.versionErrors;
            recoverFromNextHeader();
            continue;
        }
        if (count_ < 9) return false;
        const uint8_t payloadLength = buffer_[8];
        if (payloadLength > kMaxPayloadLength) {
            ++stats_.lengthErrors;
            recoverFromNextHeader();
            continue;
        }

        const size_t total = static_cast<size_t>(payloadLength) + kFrameOverhead;
        if (count_ < total) return false;
        const uint16_t expectedCrc = getU16(buffer_ + 9 + payloadLength);
        const uint16_t actualCrc = crc16CcittFalse(buffer_ + 2, 7 + payloadLength);
        if (actualCrc != expectedCrc) {
            ++stats_.crcErrors;
            recoverFromNextHeader();
            continue;
        }
        if (!destinationAccepted(buffer_[5])) {
            ++stats_.destinationErrors;
            consume(total);
            continue;
        }

        output.kind = PARSED_V2;
        output.frame.version = buffer_[2];
        output.frame.type = buffer_[3];
        output.frame.src = buffer_[4];
        output.frame.dst = buffer_[5];
        output.frame.seq = buffer_[6];
        output.frame.flags = buffer_[7];
        output.frame.length = payloadLength;
        memset(output.frame.payload, 0, sizeof(output.frame.payload));
        if (payloadLength != 0) {
            memcpy(output.frame.payload, buffer_ + 9, payloadLength);
        }
        ++stats_.validV2Frames;
        consume(total);
        return true;
    }
    return false;
}

bool StreamParser::feed(uint8_t byte, uint32_t nowMs, ParseOutput &output)
{
    output.kind = PARSED_NONE;
    if (count_ != 0 && hasLastByteTime_ &&
        static_cast<uint32_t>(nowMs - lastByteMs_) > kInterByteTimeoutMs) {
        ++stats_.timeoutErrors;
        reset();
    }

    if (count_ == 0 && byte != kHeader0) {
        return false;
    }
    if (count_ >= sizeof(buffer_)) {
        ++stats_.framingErrors;
        reset();
        if (byte != kHeader0) return false;
    }

    buffer_[count_++] = byte;
    lastByteMs_ = nowMs;
    hasLastByteTime_ = true;
    const bool parsed = processBuffered(output);
    if (count_ == 0) hasLastByteTime_ = false;
    return parsed;
}

SequenceGenerator::SequenceGenerator(uint8_t initial) : next_(initial)
{
}

uint8_t SequenceGenerator::next()
{
    const uint8_t result = next_;
    next_ = static_cast<uint8_t>(next_ + 1);
    return result;
}

uint8_t SequenceGenerator::peek() const
{
    return next_;
}

CarPoseTimebase::CarPoseTimebase()
    : active_(false), seq_(0), anchorMs_(0), consumedCycle_(0xFFFFFFFFu)
{
}

void CarPoseTimebase::observePose(uint8_t seq, uint32_t completedMs)
{
    active_ = true;
    seq_ = seq;
    anchorMs_ = completedMs;
    consumedCycle_ = 0xFFFFFFFFu;
}

bool CarPoseTimebase::responseWindowOpen(uint32_t nowMs, uint32_t &cycle) const
{
    static const uint32_t kCycleMs = 100;
    static const uint32_t kWindowStartMs = 30;
    // Starting no later than 45 ms leaves at least 5 ms before the 55 ms edge.
    static const uint32_t kLatestStartMs = 45;
    static const uint32_t kTimebaseMaxAgeMs = 500;

    if (!active_) return false;
    const uint32_t elapsed = static_cast<uint32_t>(nowMs - anchorMs_);
    if (elapsed > kTimebaseMaxAgeMs) return false;
    cycle = elapsed / kCycleMs;
    const uint32_t phase = elapsed % kCycleMs;
    return cycle != consumedCycle_ && phase >= kWindowStartMs &&
           phase <= kLatestStartMs;
}

bool CarPoseTimebase::telemetryEligible(uint32_t nowMs) const
{
    if (!active_ || (seq_ % 5u) != 0u) return false;
    const uint32_t elapsed = static_cast<uint32_t>(nowMs - anchorMs_);
    return elapsed <= 500u && elapsed / 100u == 0u;
}

void CarPoseTimebase::consume(uint32_t cycle)
{
    consumedCycle_ = cycle;
}

RequestDeduplicator::RequestDeduplicator()
{
    clear();
}

void RequestDeduplicator::clear()
{
    memset(entries_, 0, sizeof(entries_));
}

bool RequestDeduplicator::find(const Frame &request, uint32_t nowMs,
                               AckResult &result, uint8_t &detail) const
{
    for (size_t i = 0; i < kEntryCount; ++i) {
        const Entry &entry = entries_[i];
        if (entry.active && entry.src == request.src && entry.type == request.type &&
            entry.seq == request.seq &&
            static_cast<uint32_t>(nowMs - entry.firstSeenMs) < kDedupWindowMs) {
            result = entry.result == ACK_ACCEPTED || entry.result == ACK_DUPLICATE
                         ? ACK_DUPLICATE
                         : entry.result;
            detail = entry.detail;
            return true;
        }
    }
    return false;
}

void RequestDeduplicator::remember(const Frame &request, uint32_t nowMs,
                                   AckResult result, uint8_t detail)
{
    size_t selected = kEntryCount;
    uint32_t oldestAge = 0;
    for (size_t i = 0; i < kEntryCount; ++i) {
        Entry &entry = entries_[i];
        const uint32_t age = static_cast<uint32_t>(nowMs - entry.firstSeenMs);
        if (!entry.active || age >= kDedupWindowMs) {
            selected = i;
            break;
        }
        if (selected == kEntryCount || age > oldestAge) {
            selected = i;
            oldestAge = age;
        }
    }

    Entry &entry = entries_[selected];
    entry.active = true;
    entry.src = request.src;
    entry.type = request.type;
    entry.seq = request.seq;
    entry.firstSeenMs = nowMs;
    entry.result = result;
    entry.detail = detail;
}

RequestValidation validateFlightRequest(const Frame &request)
{
    RequestValidation validation;
    validation.result = ACK_INVALID_PARAMETER;
    validation.detail = 0;

    if (request.src != kGroundAddress || request.dst != kFlightAddress ||
        (request.flags & ACK_REQUIRED) == 0) {
        return validation;
    }

    if (request.type == SYSTEM_COMMAND) {
        SystemCommandPayload command;
        if (!decodeSystemCommandPayload(request.payload, request.length, command) ||
            command.commandFlags != 0 || command.argument != 0) {
            return validation;
        }

        if (command.command == START_MISSION) {
            if ((request.flags & URGENT) != 0) return validation;
            validation.result = ACK_ACCEPTED;
            validation.action.kind = FLIGHT_ACTION_START;
            return validation;
        }
        if (command.command == STOP_MISSION) {
            if ((request.flags & URGENT) == 0) return validation;
            validation.result = ACK_ACCEPTED;
            validation.action.kind = FLIGHT_ACTION_STOP;
            return validation;
        }

        validation.result = ACK_UNSUPPORTED;
        return validation;
    }

    if (request.type == TARGET_SELECT) {
        TargetSelectPayload target;
        if ((request.flags & URGENT) != 0 ||
            !decodeTargetSelectPayload(request.payload, request.length, target)) {
            return validation;
        }
        if (target.objectKind != 0x01 || !isQrCode(target.objectCode)) {
            return validation;
        }

        validation.result = ACK_ACCEPTED;
        validation.action.kind = FLIGHT_ACTION_SELECT_TARGET;
        validation.action.objectKind = target.objectKind;
        validation.action.objectCode = target.objectCode;
        return validation;
    }

    validation.result = ACK_UNSUPPORTED;
    return validation;
}

ReliableRequest::ReliableRequest(uint32_t ackTimeoutMs, uint8_t maxAttempts)
    : ackTimeoutMs_(ackTimeoutMs), lastSendMs_(0), maxAttempts_(maxAttempts),
      attempts_(0), active_(false)
{
}

bool ReliableRequest::start(const Frame &request, uint32_t nowMs)
{
    if (active_ || request.dst == kBroadcastAddress ||
        (request.flags & ACK_REQUIRED) == 0 || request.type == ACK ||
        maxAttempts_ == 0) {
        return false;
    }
    request_ = request;
    request_.flags = static_cast<uint8_t>(request_.flags & ~RETRANSMISSION);
    lastSendMs_ = nowMs;
    attempts_ = 1;
    active_ = true;
    return true;
}

RetryPollResult ReliableRequest::poll(uint32_t nowMs, Frame &retryFrame)
{
    if (!active_ || static_cast<uint32_t>(nowMs - lastSendMs_) < ackTimeoutMs_) {
        return RETRY_NOT_DUE;
    }
    if (attempts_ >= maxAttempts_) {
        active_ = false;
        return RETRY_EXHAUSTED;
    }

    retryFrame = request_;
    retryFrame.flags = static_cast<uint8_t>(retryFrame.flags | RETRANSMISSION);
    lastSendMs_ = nowMs;
    ++attempts_;
    return RETRY_FRAME_READY;
}

AckMatchResult ReliableRequest::acceptAck(const Frame &ackFrame, AckPayload *ack)
{
    AckPayload decoded;
    if (!active_ || ackFrame.type != ACK || (ackFrame.flags & ACK_REQUIRED) != 0 ||
        ackFrame.src != request_.dst || ackFrame.dst != request_.src ||
        !decodeAckPayload(ackFrame.payload, ackFrame.length, decoded) ||
        decoded.requestType != request_.type || decoded.requestSeq != request_.seq) {
        return ACK_NOT_MATCHED;
    }

    active_ = false;
    if (ack != 0) *ack = decoded;
    return decoded.result == ACK_ACCEPTED || decoded.result == ACK_DUPLICATE
               ? ACK_MATCHED_SUCCESS
               : ACK_MATCHED_REJECTED;
}

bool ReliableRequest::active() const
{
    return active_;
}

uint8_t ReliableRequest::attempts() const
{
    return attempts_;
}

} // namespace gpio_lora_v2
