#include <unity.h>

#include "protocol_v2.h"

#include <string.h>

using namespace gpio_lora_v2;

void setUp(void) {}
void tearDown(void) {}

namespace {

const uint8_t kStartVector[] = {
    0xAA, 0x55, 0x02, 0x10, 0x40, 0x20, 0x01, 0x01,
    0x04, 0x01, 0x00, 0x00, 0x00, 0xC6, 0xE8};
const uint8_t kStopVector[] = {
    0xAA, 0x55, 0x02, 0x10, 0x40, 0x20, 0x02, 0x05,
    0x04, 0x02, 0x00, 0x00, 0x00, 0x83, 0x17};
const uint8_t kTargetVector[] = {
    0xAA, 0x55, 0x02, 0x13, 0x40, 0x20, 0x03, 0x01,
    0x04, 0x01, 0x00, 0x00, 0xCA, 0x6F, 0xC2};
const uint8_t kCarPoseV23Vector[] = {
    0xAA, 0x55, 0x02, 0x80, 0x30, 0x10, 0x42, 0x00, 0x16,
    0x01, 0x0F, 0x00, 0x01, 0x00, 0x00, 0x00, 0x78,
    0xFF, 0xFF, 0xFF, 0xCE, 0x03, 0x84, 0x00, 0x0A,
    0x00, 0x00, 0x00, 0x00, 0x04, 0xD2, 0xB2, 0x2D};
const uint8_t kCarTaskV23Vector[] = {
    0xAA, 0x55, 0x02, 0x81, 0x30, 0x20, 0x43, 0x01, 0x0C,
    0x01, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x04, 0xD2, 0x7F, 0x78};
const uint8_t kCalibrationV23Vector[] = {
    0xAA, 0x55, 0x02, 0x83, 0x40, 0x30, 0x16, 0x01, 0x0C,
    0x00, 0x00, 0x00, 0x64, 0xFF, 0xFF, 0xFF, 0xCE,
    0x00, 0x01, 0x01, 0x00, 0xF1, 0x24};
const uint8_t kMaintenanceResetV23Vector[] = {
    0xAA, 0x55, 0x02, 0x85, 0x30, 0x10, 0x52, 0x00, 0x08,
    0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x04, 0xD2, 0x9C, 0xB2};

size_t feedBytes(StreamParser &parser, const uint8_t *data, size_t length,
                 uint32_t &nowMs, ParseOutput *last = 0)
{
    size_t parsed = 0;
    for (size_t i = 0; i < length; ++i) {
        ParseOutput output;
        if (parser.feed(data[i], nowMs++, output)) {
            ++parsed;
            if (last != 0) *last = output;
        }
    }
    return parsed;
}

Frame parseOne(const uint8_t *data, size_t length)
{
    StreamParser parser(kFlightAddress);
    ParseOutput output;
    uint32_t now = 0;
    TEST_ASSERT_EQUAL_UINT32(1, feedBytes(parser, data, length, now, &output));
    TEST_ASSERT_EQUAL(PARSED_V2, output.kind);
    return output.frame;
}

Frame parseOneAnyDestination(const uint8_t *data, size_t length)
{
    StreamParser parser(0);
    ParseOutput output;
    uint32_t now = 0;
    TEST_ASSERT_EQUAL_UINT32(1, feedBytes(parser, data, length, now, &output));
    TEST_ASSERT_EQUAL(PARSED_V2, output.kind);
    return output.frame;
}

Frame makeStart(uint8_t seq)
{
    Frame frame;
    frame.type = SYSTEM_COMMAND;
    frame.src = kGroundAddress;
    frame.dst = kFlightAddress;
    frame.seq = seq;
    frame.flags = ACK_REQUIRED;
    frame.length = 4;
    SystemCommandPayload command = {START_MISSION, 0, 0};
    TEST_ASSERT_TRUE(encodeSystemCommandPayload(command, frame.payload,
                                                sizeof(frame.payload)));
    return frame;
}

Frame makeAckFor(const Frame &request, AckResult result)
{
    Frame ack;
    ack.type = ACK;
    ack.src = request.dst;
    ack.dst = request.src;
    ack.seq = 0x77;
    ack.flags = 0;
    ack.length = 4;
    AckPayload payload = {request.type, request.seq, result, 0};
    TEST_ASSERT_TRUE(encodeAckPayload(payload, ack.payload, sizeof(ack.payload)));
    return ack;
}

void test_crc_standard_vector()
{
    const uint8_t input[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    TEST_ASSERT_EQUAL_HEX16(0x29B1, crc16CcittFalse(input, sizeof(input)));
}

void assertV23VectorRoundTrip(const uint8_t *vector, size_t length,
                              uint8_t type)
{
    Frame frame = parseOneAnyDestination(vector, length);
    TEST_ASSERT_EQUAL_HEX8(type, frame.type);
    uint8_t encoded[kMaxFrameLength];
    size_t written = 0;
    TEST_ASSERT_EQUAL(ENCODE_OK,
                      encodeFrame(frame, encoded, sizeof(encoded), written));
    TEST_ASSERT_EQUAL_UINT32(length, written);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(vector, encoded, length);
}

void test_v23_document_vectors_and_payloads()
{
    assertV23VectorRoundTrip(kCarPoseV23Vector, sizeof(kCarPoseV23Vector),
                             CAR_POSE);
    assertV23VectorRoundTrip(kCarTaskV23Vector, sizeof(kCarTaskV23Vector),
                             CAR_TASK_REQUEST);
    assertV23VectorRoundTrip(kCalibrationV23Vector,
                             sizeof(kCalibrationV23Vector), CALIBRATION_SET);

    Frame poseFrame = parseOneAnyDestination(kCarPoseV23Vector,
                                              sizeof(kCarPoseV23Vector));
    TEST_ASSERT_EQUAL_HEX8(kCarAddress, poseFrame.src);
    TEST_ASSERT_EQUAL_HEX8(kBroadcastAddress, poseFrame.dst);
    TEST_ASSERT_EQUAL_HEX8(0x42, poseFrame.seq);
    CarPosePayload pose;
    TEST_ASSERT_TRUE(decodeCarPosePayload(poseFrame.payload, poseFrame.length,
                                          pose));
    TEST_ASSERT_EQUAL_HEX8(0x01, pose.coordinateFrame);
    TEST_ASSERT_EQUAL_HEX8(0x0F, pose.poseFlags);
    TEST_ASSERT_EQUAL_HEX16(1, pose.calibrationId);
    TEST_ASSERT_EQUAL_INT32(120, pose.xCm);
    TEST_ASSERT_EQUAL_INT32(-50, pose.yCm);
    TEST_ASSERT_EQUAL_INT16(900, pose.yawDeciDegrees);
    TEST_ASSERT_EQUAL_INT16(10, pose.vxCmPerSec);
    TEST_ASSERT_EQUAL_INT16(0, pose.vyCmPerSec);
    TEST_ASSERT_EQUAL_UINT32(1234, pose.sourceTimeMs);

    Frame taskFrame = parseOneAnyDestination(kCarTaskV23Vector,
                                              sizeof(kCarTaskV23Vector));
    CarTaskRequestPayload task;
    TEST_ASSERT_TRUE(decodeCarTaskRequestPayload(taskFrame.payload,
                                                 taskFrame.length, task));
    TEST_ASSERT_EQUAL_HEX8(1, task.taskType);
    TEST_ASSERT_EQUAL_HEX8(1, task.requestFlags);
    TEST_ASSERT_EQUAL_HEX16(1, task.missionId);
    TEST_ASSERT_EQUAL_HEX16(1, task.calibrationId);
    TEST_ASSERT_EQUAL_HEX16(0, task.reserved);
    TEST_ASSERT_EQUAL_UINT32(1234, task.sourceTimeMs);
}

void test_v23_maintenance_reset_vector_and_payload()
{
    assertV23VectorRoundTrip(kMaintenanceResetV23Vector,
                             sizeof(kMaintenanceResetV23Vector),
                             MAINTENANCE_RESET);

    Frame frame = parseOneAnyDestination(kMaintenanceResetV23Vector,
                                         sizeof(kMaintenanceResetV23Vector));
    TEST_ASSERT_EQUAL_HEX8(kCarAddress, frame.src);
    TEST_ASSERT_EQUAL_HEX8(kBroadcastAddress, frame.dst);
    TEST_ASSERT_EQUAL_HEX8(0x52, frame.seq);
    TEST_ASSERT_EQUAL_HEX8(0, frame.flags);
    TEST_ASSERT_EQUAL_UINT8(8, frame.length);

    MaintenanceResetPayload reset;
    TEST_ASSERT_TRUE(decodeMaintenanceResetPayload(frame.payload, frame.length,
                                                   reset));
    TEST_ASSERT_EQUAL_HEX16(1, reset.resetId);
    TEST_ASSERT_EQUAL_HEX8(0x01, reset.resetFlags);
    TEST_ASSERT_EQUAL_HEX8(0, reset.reserved);
    TEST_ASSERT_EQUAL_UINT32(1234, reset.sourceTimeMs);

    uint8_t encoded[8];
    TEST_ASSERT_TRUE(encodeMaintenanceResetPayload(reset, encoded,
                                                   sizeof(encoded)));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(frame.payload, encoded, sizeof(encoded));

    encoded[2] = 0x02;
    TEST_ASSERT_FALSE(decodeMaintenanceResetPayload(encoded, sizeof(encoded),
                                                    reset));
    encoded[2] = 0x01;
    encoded[3] = 0x01;
    TEST_ASSERT_FALSE(decodeMaintenanceResetPayload(encoded, sizeof(encoded),
                                                    reset));
    encoded[3] = 0x00;
    encoded[0] = 0x00;
    encoded[1] = 0x00;
    TEST_ASSERT_FALSE(decodeMaintenanceResetPayload(encoded, sizeof(encoded),
                                                    reset));
    encoded[1] = 0x01;
    encoded[2] = 0x00;
    TEST_ASSERT_FALSE(decodeMaintenanceResetPayload(encoded, sizeof(encoded),
                                                    reset));

    MaintenanceResetPayload invalid_reset = reset;
    invalid_reset.resetId = 0;
    TEST_ASSERT_FALSE(encodeMaintenanceResetPayload(invalid_reset, encoded,
                                                    sizeof(encoded)));
    invalid_reset = reset;
    invalid_reset.resetFlags = 0;
    TEST_ASSERT_FALSE(encodeMaintenanceResetPayload(invalid_reset, encoded,
                                                    sizeof(encoded)));

    const uint8_t lxResetThroughEe[] = {
        0xBB, 0x10, 0xF1, 0x30, 0x05, 0x08, 0x00, 0x01,
        0x01, 0x00, 0x00, 0x00, 0x04, 0xD2, 0xEE};
    uint8_t sc;
    uint8_t ac;
    lxDoubleChecksum(lxResetThroughEe, sizeof(lxResetThroughEe), sc, ac);
    TEST_ASSERT_EQUAL_HEX8(0xBF, sc);
    TEST_ASSERT_EQUAL_HEX8(0x86, ac);
}

void test_v23_status_response_and_lx_checksum()
{
    MissionStatusPayload status = {
        2, 8, 0x1234, 0xABCD, 6, 0, 0x01020304};
    uint8_t statusBytes[12];
    TEST_ASSERT_TRUE(encodeMissionStatusPayload(status, statusBytes,
                                                sizeof(statusBytes)));
    const uint8_t expectedStatus[] = {
        0x02, 0x08, 0x12, 0x34, 0xAB, 0xCD,
        0x06, 0x00, 0x01, 0x02, 0x03, 0x04};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expectedStatus, statusBytes,
                                 sizeof(expectedStatus));
    MissionStatusPayload decodedStatus;
    TEST_ASSERT_TRUE(decodeMissionStatusPayload(statusBytes,
                                                sizeof(statusBytes),
                                                decodedStatus));
    TEST_ASSERT_EQUAL_HEX16(status.missionId, decodedStatus.missionId);

    MissionResponsePayload response = {
        CAR_TASK_REQUEST, 0x1234, ACK_STATE_DISALLOWED, 0x02, 0};
    uint8_t responseBytes[6];
    TEST_ASSERT_TRUE(encodeMissionResponsePayload(response, responseBytes,
                                                  sizeof(responseBytes)));
    const uint8_t expectedResponse[] = {0x81, 0x12, 0x34, 0x03, 0x02, 0x00};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expectedResponse, responseBytes,
                                 sizeof(expectedResponse));
    MissionResponsePayload decodedResponse;
    TEST_ASSERT_TRUE(decodeMissionResponsePayload(responseBytes,
                                                  sizeof(responseBytes),
                                                  decodedResponse));
    TEST_ASSERT_EQUAL(ACK_STATE_DISALLOWED, decodedResponse.result);

    MissionResponsePayload abortResponse = {
        MISSION_ABORT, 0, ACK_ACCEPTED, 0, 0};
    TEST_ASSERT_TRUE(encodeMissionResponsePayload(abortResponse, responseBytes,
                                                  sizeof(responseBytes)));
    const uint8_t expectedAbortResponse[] = {0x84, 0x00, 0x00, 0x00, 0x00, 0x00};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expectedAbortResponse, responseBytes,
                                 sizeof(expectedAbortResponse));

    const uint8_t lxAbortThroughEe[] = {
        0xBB, 0x10, 0xF1, 0x30, 0x06, 0x00, 0xEE};
    uint8_t abortSc;
    uint8_t abortAc;
    lxDoubleChecksum(lxAbortThroughEe, sizeof(lxAbortThroughEe), abortSc,
                     abortAc);
    TEST_ASSERT_EQUAL_HEX8(0xE0, abortSc);
    TEST_ASSERT_EQUAL_HEX8(0xF2, abortAc);

    const uint8_t lxCarPoseThroughEe[] = {
        0xBB, 0x10, 0xF1, 0x30, 0x01, 0x16,
        0x01, 0x0F, 0x00, 0x01, 0x00, 0x00, 0x00, 0x78,
        0xFF, 0xFF, 0xFF, 0xCE, 0x03, 0x84, 0x00, 0x0A,
        0x00, 0x00, 0x00, 0x00, 0x04, 0xD2, 0xEE};
    uint8_t sc;
    uint8_t ac;
    lxDoubleChecksum(lxCarPoseThroughEe, sizeof(lxCarPoseThroughEe), sc, ac);
    TEST_ASSERT_EQUAL_HEX8(0xAC, sc);
    TEST_ASSERT_EQUAL_HEX8(0x0D, ac);
}

void test_v23_aa55_never_enters_legacy_channel()
{
    StreamParser parser(kFlightAddress);
    ParseOutput output;
    uint32_t now = 0;
    TEST_ASSERT_EQUAL_UINT32(
        1, feedBytes(parser, kCarPoseV23Vector, sizeof(kCarPoseV23Vector),
                     now, &output));
    TEST_ASSERT_EQUAL(PARSED_V2, output.kind);
    TEST_ASSERT_EQUAL_UINT32(0, parser.stats().validLegacyFrames);
}

void test_v23_car_pose_timebase_and_priority_slot_gate()
{
    CarPoseTimebase timebase;
    uint32_t cycle = 99;
    timebase.observePose(0x0A, 1000);
    TEST_ASSERT_FALSE(timebase.responseWindowOpen(1029, cycle));
    TEST_ASSERT_TRUE(timebase.responseWindowOpen(1030, cycle));
    TEST_ASSERT_EQUAL_UINT32(0, cycle);
    TEST_ASSERT_TRUE(timebase.telemetryEligible(1030));
    TEST_ASSERT_TRUE(timebase.responseWindowOpen(1045, cycle));
    TEST_ASSERT_FALSE(timebase.responseWindowOpen(1046, cycle));

    timebase.consume(0);
    TEST_ASSERT_FALSE(timebase.responseWindowOpen(1040, cycle));

    // A task request may replace the next CAR_POSE; ACK/status can still use
    // the extrapolated response slot, while telemetry cannot.
    TEST_ASSERT_TRUE(timebase.responseWindowOpen(1130, cycle));
    TEST_ASSERT_EQUAL_UINT32(1, cycle);
    TEST_ASSERT_FALSE(timebase.telemetryEligible(1130));
    timebase.consume(cycle);
    TEST_ASSERT_FALSE(timebase.responseWindowOpen(1140, cycle));
    TEST_ASSERT_FALSE(timebase.responseWindowOpen(1530, cycle));

    // A newly received pose resets the one-frame gate. Non-mod-5 Seq does not
    // make telemetry eligible, but ACK/status still have a valid slot.
    timebase.observePose(0x0B, 2000);
    TEST_ASSERT_TRUE(timebase.responseWindowOpen(2030, cycle));
    TEST_ASSERT_FALSE(timebase.telemetryEligible(2030));

    // uint32_t millis rollover must preserve the phase calculation.
    timebase.observePose(0x0F, 0xFFFFFFF0u);
    TEST_ASSERT_TRUE(timebase.responseWindowOpen(0x0000000Eu, cycle));
    TEST_ASSERT_EQUAL_UINT32(0, cycle);
    TEST_ASSERT_TRUE(timebase.telemetryEligible(0x0000000Eu));
}

void assertVectorRoundTrip(const uint8_t *vector, size_t length, uint8_t type,
                           uint8_t seq, uint8_t flags)
{
    Frame frame = parseOne(vector, length);
    TEST_ASSERT_EQUAL_HEX8(type, frame.type);
    TEST_ASSERT_EQUAL_HEX8(kGroundAddress, frame.src);
    TEST_ASSERT_EQUAL_HEX8(kFlightAddress, frame.dst);
    TEST_ASSERT_EQUAL_HEX8(seq, frame.seq);
    TEST_ASSERT_EQUAL_HEX8(flags, frame.flags);

    uint8_t encoded[kMaxFrameLength];
    size_t written = 0;
    TEST_ASSERT_EQUAL(ENCODE_OK,
                      encodeFrame(frame, encoded, sizeof(encoded), written));
    TEST_ASSERT_EQUAL_UINT32(length, written);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(vector, encoded, length);
}

void test_all_document_full_frame_vectors()
{
    assertVectorRoundTrip(kStartVector, sizeof(kStartVector), SYSTEM_COMMAND, 0x01,
                          ACK_REQUIRED);
    assertVectorRoundTrip(kStopVector, sizeof(kStopVector), SYSTEM_COMMAND, 0x02,
                          ACK_REQUIRED | URGENT);
    assertVectorRoundTrip(kTargetVector, sizeof(kTargetVector), TARGET_SELECT, 0x03,
                          ACK_REQUIRED);
}

void test_payload_byte_order_and_round_trip()
{
    FlightTelemetryPayload telemetry = {
        0x0017, 0x01, 0xFF, -123456, 654321, -50, -1799, 0xFFFF, 0x89ABCDEF};
    uint8_t bytes[24];
    TEST_ASSERT_TRUE(encodeFlightTelemetryPayload(telemetry, bytes, sizeof(bytes)));
    const uint8_t expected[] = {
        0x00, 0x17, 0x01, 0xFF, 0xFF, 0xFE, 0x1D, 0xC0,
        0x00, 0x09, 0xFB, 0xF1, 0xFF, 0xFF, 0xFF, 0xCE,
        0xF8, 0xF9, 0xFF, 0xFF, 0x89, 0xAB, 0xCD, 0xEF};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, bytes, sizeof(expected));

    FlightTelemetryPayload decoded;
    TEST_ASSERT_TRUE(decodeFlightTelemetryPayload(bytes, sizeof(bytes), decoded));
    TEST_ASSERT_EQUAL_INT32(telemetry.xCm, decoded.xCm);
    TEST_ASSERT_EQUAL_INT32(telemetry.yCm, decoded.yCm);
    TEST_ASSERT_EQUAL_INT16(telemetry.yawDeciDegrees, decoded.yawDeciDegrees);

    ObjectReportPayload object = {
        0x06, 0x1234, 0xFEDC, 0x01, 0x0D, -1, -2, -3, 100, 123456};
    TEST_ASSERT_TRUE(encodeObjectReportPayload(object, bytes, sizeof(bytes)));
    ObjectReportPayload objectDecoded;
    TEST_ASSERT_TRUE(decodeObjectReportPayload(bytes, sizeof(bytes), objectDecoded));
    TEST_ASSERT_EQUAL_HEX16(object.objectCode, objectDecoded.objectCode);
    TEST_ASSERT_EQUAL_INT32(-3, objectDecoded.zCm);

    HeartbeatPayload heartbeat = {2, 7, 0x01020304};
    uint8_t heartbeatBytes[8];
    TEST_ASSERT_TRUE(
        encodeHeartbeatPayload(heartbeat, heartbeatBytes, sizeof(heartbeatBytes)));
    const uint8_t heartbeatExpected[] = {2, 7, 0, 0, 1, 2, 3, 4};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(heartbeatExpected, heartbeatBytes, 8);
}

void test_split_frame_at_every_boundary()
{
    for (size_t split = 0; split <= sizeof(kStartVector); ++split) {
        StreamParser parser(kFlightAddress);
        ParseOutput output;
        uint32_t now = 10;
        size_t count = feedBytes(parser, kStartVector, split, now, &output);
        count += feedBytes(parser, kStartVector + split,
                           sizeof(kStartVector) - split, now, &output);
        TEST_ASSERT_EQUAL_UINT32(1, count);
        TEST_ASSERT_EQUAL(PARSED_V2, output.kind);
        TEST_ASSERT_EQUAL_HEX8(0x01, output.frame.seq);
    }
}

void test_sticky_frames_and_legacy_compatibility()
{
    uint8_t stream[sizeof(kStartVector) + sizeof(kStopVector) + 9];
    memcpy(stream, kStartVector, sizeof(kStartVector));
    memcpy(stream + sizeof(kStartVector), kStopVector, sizeof(kStopVector));
    const uint8_t legacy[] = {0xAA, 0xBB, 0x20, 0xCA, 0xFF,
                              0x9C, 0x00, 0xC8, 0xFF};
    memcpy(stream + sizeof(kStartVector) + sizeof(kStopVector), legacy,
           sizeof(legacy));

    StreamParser parser(kFlightAddress);
    ParseOutput output;
    uint32_t now = 0;
    size_t count = 0;
    size_t legacyCount = 0;
    for (size_t i = 0; i < sizeof(stream); ++i) {
        if (parser.feed(stream[i], now++, output)) {
            ++count;
            if (output.kind == PARSED_LEGACY) {
                ++legacyCount;
                TEST_ASSERT_EQUAL_INT16(-100, output.legacy.xCm);
                TEST_ASSERT_EQUAL_INT16(200, output.legacy.yCm);
            }
        }
    }
    TEST_ASSERT_EQUAL_UINT32(3, count);
    TEST_ASSERT_EQUAL_UINT32(1, legacyCount);
}

void test_noise_crc_error_and_resynchronization()
{
    uint8_t bad[sizeof(kStartVector)];
    memcpy(bad, kStartVector, sizeof(bad));
    bad[10] ^= 0x80;
    const uint8_t noise[] = {0x00, 0x7F, 0xAA, 0x12, 0xAA, 0xAA, 0x99};

    StreamParser parser(kFlightAddress);
    ParseOutput output;
    uint32_t now = 0;
    TEST_ASSERT_EQUAL_UINT32(0, feedBytes(parser, noise, sizeof(noise), now, &output));
    TEST_ASSERT_EQUAL_UINT32(0, feedBytes(parser, bad, sizeof(bad), now, &output));
    TEST_ASSERT_EQUAL_UINT32(
        1, feedBytes(parser, kTargetVector, sizeof(kTargetVector), now, &output));
    TEST_ASSERT_EQUAL(PARSED_V2, output.kind);
    TEST_ASSERT_EQUAL_HEX8(TARGET_SELECT, output.frame.type);
    TEST_ASSERT_EQUAL_UINT32(1, parser.stats().crcErrors);
}

void test_invalid_length_and_inter_byte_timeout()
{
    const uint8_t invalidLength[] = {
        0xAA, 0x55, 0x02, 0x10, 0x40, 0x20, 0x10, 0x01, 0x41};
    StreamParser parser(kFlightAddress);
    ParseOutput output;
    uint32_t now = 0;
    TEST_ASSERT_EQUAL_UINT32(
        0, feedBytes(parser, invalidLength, sizeof(invalidLength), now, &output));
    TEST_ASSERT_EQUAL_UINT32(1, parser.stats().lengthErrors);

    for (size_t i = 0; i < 5; ++i) {
        TEST_ASSERT_FALSE(parser.feed(kStartVector[i], now++, output));
    }
    now += 101;
    TEST_ASSERT_FALSE(parser.feed(kStartVector[5], now++, output));
    TEST_ASSERT_EQUAL_UINT32(1, parser.stats().timeoutErrors);

    TEST_ASSERT_EQUAL_UINT32(
        1, feedBytes(parser, kStartVector, sizeof(kStartVector), now, &output));
}

void test_invalid_version_and_destination_are_dropped()
{
    uint8_t invalidVersion[sizeof(kStartVector)];
    memcpy(invalidVersion, kStartVector, sizeof(invalidVersion));
    invalidVersion[2] = 0x03;

    StreamParser parser(kFlightAddress);
    ParseOutput output;
    uint32_t now = 0;
    TEST_ASSERT_EQUAL_UINT32(
        0, feedBytes(parser, invalidVersion, sizeof(invalidVersion), now, &output));
    TEST_ASSERT_EQUAL_UINT32(1, parser.stats().versionErrors);

    Frame wrongDestination = makeStart(0x42);
    wrongDestination.dst = 0x30;
    uint8_t encoded[kMaxFrameLength];
    size_t written = 0;
    TEST_ASSERT_EQUAL(ENCODE_OK,
                      encodeFrame(wrongDestination, encoded, sizeof(encoded), written));
    TEST_ASSERT_EQUAL_UINT32(0, feedBytes(parser, encoded, written, now, &output));
    TEST_ASSERT_EQUAL_UINT32(1, parser.stats().destinationErrors);
}

void test_crc_failure_recovers_from_nested_next_header()
{
    // A valid frame is deliberately embedded in a longer frame whose CRC is bad.
    // Recovery must scan the already-buffered bytes for the next 0xAA.
    uint8_t outer[kMaxFrameLength] = {0};
    outer[0] = 0xAA;
    outer[1] = 0x55;
    outer[2] = 0x02;
    outer[3] = 0x7F;
    outer[4] = 0x40;
    outer[5] = 0x20;
    outer[6] = 0x99;
    outer[7] = 0;
    outer[8] = sizeof(kStartVector);
    memcpy(outer + 9, kStartVector, sizeof(kStartVector));
    outer[9 + sizeof(kStartVector)] = 0;
    outer[10 + sizeof(kStartVector)] = 0;

    StreamParser parser(kFlightAddress);
    ParseOutput output;
    uint32_t now = 0;
    TEST_ASSERT_EQUAL_UINT32(
        1, feedBytes(parser, outer, sizeof(kStartVector) + 11, now, &output));
    TEST_ASSERT_EQUAL_HEX8(SYSTEM_COMMAND, output.frame.type);
    TEST_ASSERT_EQUAL_UINT32(1, parser.stats().crcErrors);
}

void test_sequence_wraps_after_ff()
{
    SequenceGenerator sequence(0xFE);
    TEST_ASSERT_EQUAL_HEX8(0xFE, sequence.next());
    TEST_ASSERT_EQUAL_HEX8(0xFF, sequence.next());
    TEST_ASSERT_EQUAL_HEX8(0x00, sequence.next());
    TEST_ASSERT_EQUAL_HEX8(0x01, sequence.peek());
}

void test_request_validation_excludes_recording()
{
    Frame start = parseOne(kStartVector, sizeof(kStartVector));
    RequestValidation validation = validateFlightRequest(start);
    TEST_ASSERT_EQUAL(ACK_ACCEPTED, validation.result);
    TEST_ASSERT_EQUAL(FLIGHT_ACTION_START, validation.action.kind);

    Frame stop = parseOne(kStopVector, sizeof(kStopVector));
    validation = validateFlightRequest(stop);
    TEST_ASSERT_EQUAL(ACK_ACCEPTED, validation.result);
    TEST_ASSERT_EQUAL(FLIGHT_ACTION_STOP, validation.action.kind);

    Frame target = parseOne(kTargetVector, sizeof(kTargetVector));
    validation = validateFlightRequest(target);
    TEST_ASSERT_EQUAL(ACK_ACCEPTED, validation.result);
    TEST_ASSERT_EQUAL(FLIGHT_ACTION_SELECT_TARGET, validation.action.kind);
    TEST_ASSERT_EQUAL_HEX16(0x00CA, validation.action.objectCode);

    Frame record = makeStart(4);
    record.payload[0] = START_RECORDING;
    validation = validateFlightRequest(record);
    TEST_ASSERT_EQUAL(ACK_UNSUPPORTED, validation.result);
    TEST_ASSERT_EQUAL(FLIGHT_ACTION_NONE, validation.action.kind);

    Frame badStop = stop;
    badStop.flags = ACK_REQUIRED;
    validation = validateFlightRequest(badStop);
    TEST_ASSERT_EQUAL(ACK_INVALID_PARAMETER, validation.result);
}

void test_ack_loss_same_seq_retransmission_is_idempotent()
{
    Frame request = makeStart(0x55);
    RequestDeduplicator dedup;
    unsigned actionCount = 0;

    AckResult result;
    uint8_t detail;
    TEST_ASSERT_FALSE(dedup.find(request, 1000, result, detail));
    RequestValidation validation = validateFlightRequest(request);
    if (validation.result == ACK_ACCEPTED) ++actionCount;
    dedup.remember(request, 1000, validation.result, validation.detail);

    // The first ACK is lost. Ground retransmits the same Seq with RETRANSMISSION.
    Frame retry = request;
    retry.flags |= RETRANSMISSION;
    TEST_ASSERT_TRUE(dedup.find(retry, 1500, result, detail));
    TEST_ASSERT_EQUAL(ACK_DUPLICATE, result);
    TEST_ASSERT_EQUAL_UINT32(1, actionCount);

    // A retry with changed payload is still the same Src + Type + Seq request.
    retry.payload[0] = STOP_MISSION;
    retry.flags |= URGENT;
    TEST_ASSERT_TRUE(dedup.find(retry, 2000, result, detail));
    TEST_ASSERT_EQUAL(ACK_DUPLICATE, result);
    TEST_ASSERT_EQUAL_UINT32(1, actionCount);
}

void test_five_second_dedup_and_seq_reuse()
{
    Frame request = makeStart(0xFF);
    RequestDeduplicator dedup;
    dedup.remember(request, 100, ACK_ACCEPTED, 0);

    AckResult result;
    uint8_t detail;
    TEST_ASSERT_TRUE(dedup.find(request, 5099, result, detail));
    TEST_ASSERT_EQUAL(ACK_DUPLICATE, result);
    TEST_ASSERT_FALSE(dedup.find(request, 5100, result, detail));

    Frame sameSeqDifferentType = request;
    sameSeqDifferentType.type = TARGET_SELECT;
    TEST_ASSERT_FALSE(dedup.find(sameSeqDifferentType, 200, result, detail));

    // uint32 millis rollover must not break the age calculation.
    dedup.clear();
    dedup.remember(request, 0xFFFFFF00u, ACK_ACCEPTED, 0);
    TEST_ASSERT_TRUE(dedup.find(request, 0x00000020u, result, detail));
}

void test_retry_timing_and_ack_matching()
{
    Frame request = makeStart(0xFE);
    ReliableRequest pending(500, 3);
    TEST_ASSERT_TRUE(pending.start(request, 100));

    Frame retry;
    TEST_ASSERT_EQUAL(RETRY_NOT_DUE, pending.poll(599, retry));
    TEST_ASSERT_EQUAL(RETRY_FRAME_READY, pending.poll(600, retry));
    TEST_ASSERT_EQUAL_HEX8(request.seq, retry.seq);
    TEST_ASSERT_BITS_HIGH(RETRANSMISSION, retry.flags);
    TEST_ASSERT_EQUAL_UINT8(2, pending.attempts());
    TEST_ASSERT_EQUAL(RETRY_FRAME_READY, pending.poll(1100, retry));
    TEST_ASSERT_EQUAL_UINT8(3, pending.attempts());

    Frame wrongAck = makeAckFor(request, ACK_ACCEPTED);
    wrongAck.payload[1] ^= 1;
    TEST_ASSERT_EQUAL(ACK_NOT_MATCHED, pending.acceptAck(wrongAck));
    Frame ack = makeAckFor(request, ACK_DUPLICATE);
    TEST_ASSERT_EQUAL(ACK_MATCHED_SUCCESS, pending.acceptAck(ack));
    TEST_ASSERT_FALSE(pending.active());

    TEST_ASSERT_TRUE(pending.start(request, 2000));
    TEST_ASSERT_EQUAL(RETRY_FRAME_READY, pending.poll(2500, retry));
    TEST_ASSERT_EQUAL(RETRY_FRAME_READY, pending.poll(3000, retry));
    TEST_ASSERT_EQUAL(RETRY_EXHAUSTED, pending.poll(3500, retry));
}

void test_encoder_rejects_illegal_ack_and_broadcast_flags()
{
    Frame frame = makeStart(1);
    uint8_t bytes[kMaxFrameLength];
    size_t written;

    frame.dst = kBroadcastAddress;
    TEST_ASSERT_EQUAL(ENCODE_INVALID_FLAGS,
                      encodeFrame(frame, bytes, sizeof(bytes), written));
    frame.dst = kFlightAddress;
    frame.src = kBroadcastAddress;
    TEST_ASSERT_EQUAL(ENCODE_INVALID_SOURCE,
                      encodeFrame(frame, bytes, sizeof(bytes), written));
    frame.src = kFlightAddress;
    frame.type = ACK;
    TEST_ASSERT_EQUAL(ENCODE_INVALID_FLAGS,
                      encodeFrame(frame, bytes, sizeof(bytes), written));
}

} // namespace

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_crc_standard_vector);
    RUN_TEST(test_v23_document_vectors_and_payloads);
    RUN_TEST(test_v23_maintenance_reset_vector_and_payload);
    RUN_TEST(test_v23_status_response_and_lx_checksum);
    RUN_TEST(test_v23_aa55_never_enters_legacy_channel);
    RUN_TEST(test_v23_car_pose_timebase_and_priority_slot_gate);
    RUN_TEST(test_all_document_full_frame_vectors);
    RUN_TEST(test_payload_byte_order_and_round_trip);
    RUN_TEST(test_split_frame_at_every_boundary);
    RUN_TEST(test_sticky_frames_and_legacy_compatibility);
    RUN_TEST(test_noise_crc_error_and_resynchronization);
    RUN_TEST(test_invalid_length_and_inter_byte_timeout);
    RUN_TEST(test_invalid_version_and_destination_are_dropped);
    RUN_TEST(test_crc_failure_recovers_from_nested_next_header);
    RUN_TEST(test_sequence_wraps_after_ff);
    RUN_TEST(test_request_validation_excludes_recording);
    RUN_TEST(test_ack_loss_same_seq_retransmission_is_idempotent);
    RUN_TEST(test_five_second_dedup_and_seq_reuse);
    RUN_TEST(test_retry_timing_and_ack_matching);
    RUN_TEST(test_encoder_rejects_illegal_ack_and_broadcast_flags);
    return UNITY_END();
}
