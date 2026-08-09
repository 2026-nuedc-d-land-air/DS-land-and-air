/** V2.2 Pi-to-MCU CAR_POSE ingress; see bsp_car_pose_link.h. */

#include "bsp_car_pose_link.h"

#include "bsp_robot_uart.h"
#include "bsp_v22_protocol.h"

#define CAR_POSE_LINK_POLL_LIMIT          128U
#define CAR_POSE_LINK_PAYLOAD_LENGTH        22U
#define CAR_POSE_LINK_MAX_YAW_TENTHS      1800
#define CAR_POSE_LINK_INVALID_VELOCITY  0x7FFF
#define CAR_POSE_LINK_REQUIRED_TASK_FLAGS \
    (V22_POSE_FLAG_POSITION_VALID | V22_POSE_FLAG_CALIBRATED | \
     V22_POSE_FLAG_YAW_VALID)
#define CAR_POSE_LINK_ALLOWED_FLAGS \
    (V22_POSE_FLAG_POSITION_VALID | V22_POSE_FLAG_CALIBRATED | \
     V22_POSE_FLAG_VELOCITY_VALID | V22_POSE_FLAG_YAW_VALID | \
     V22_POSE_FLAG_CAR_RUNNING)
#define CAR_POSE_LINK_TASK_READY_FRAME_COUNT 3U
#define CAR_POSE_LINK_LEGACY_HEAD         0xFAU
#define CAR_POSE_LINK_LEGACY_TAIL         0xABU
#define CAR_POSE_LINK_LEGACY_SIZE            14U
#define CAR_POSE_LINK_LEGACY_FIRST_TIME_MS     0U
#define CAR_POSE_LINK_ACK_QUEUE_COUNT           4U

#define CAR_POSE_LINK_SOURCE_NONE             0U
#define CAR_POSE_LINK_SOURCE_LEGACY           1U
#define CAR_POSE_LINK_SOURCE_V22              2U

static CarPoseLinkState_t s_state;
static V22StreamParser_t s_parser;
static uint8_t s_legacyFrame[CAR_POSE_LINK_LEGACY_SIZE];
static uint8_t s_legacyFrameIndex;
static uint8_t s_legacyRelayEpochStarted;
static uint32_t s_legacyRelaySourceTimeMs;
static uint32_t s_legacyRelayLastReceiveMs;
static CarPoseLinkAck_t s_ackQueue[CAR_POSE_LINK_ACK_QUEUE_COUNT];
static uint8_t s_ackQueueHead;
static uint8_t s_ackQueueTail;
static uint8_t s_ackQueueCount;

static uint8_t CarPoseLink_NextAckIndex(uint8_t index)
{
    ++index;
    return (index >= CAR_POSE_LINK_ACK_QUEUE_COUNT) ? 0U : index;
}

static void CarPoseLink_QueueAck(const CarPoseLinkAck_t *ack)
{
    if (ack == 0)
    {
        return;
    }
    if (s_ackQueueCount >= CAR_POSE_LINK_ACK_QUEUE_COUNT)
    {
        ++s_state.ackInvalidFrameCount;
        return;
    }

    s_ackQueue[s_ackQueueHead] = *ack;
    s_ackQueueHead = CarPoseLink_NextAckIndex(s_ackQueueHead);
    ++s_ackQueueCount;
}

static uint16_t CarPoseLink_ReadU16Be(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static uint32_t CarPoseLink_ReadU32Be(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24) |
           ((uint32_t)data[1] << 16) |
           ((uint32_t)data[2] << 8) |
           (uint32_t)data[3];
}

static int16_t CarPoseLink_ReadI16Be(const uint8_t *data)
{
    return (int16_t)CarPoseLink_ReadU16Be(data);
}

static int32_t CarPoseLink_ReadI32Be(const uint8_t *data)
{
    return (int32_t)CarPoseLink_ReadU32Be(data);
}

static int32_t CarPoseLink_ReadLegacySigned3(uint8_t sign, const uint8_t *magnitude)
{
    int32_t value;

    value = ((int32_t)magnitude[0] << 16) |
            ((int32_t)magnitude[1] << 8) |
            (int32_t)magnitude[2];
    return (sign != 0U) ? -value : value;
}

static uint8_t CarPoseLink_IsTaskPose(uint8_t flags, uint16_t calibrationId)
{
    return (((flags & CAR_POSE_LINK_REQUIRED_TASK_FLAGS) ==
             CAR_POSE_LINK_REQUIRED_TASK_FLAGS) &&
            (calibrationId != 0U)) ? 1U : 0U;
}

static void CarPoseLink_RecordInvalidPose(void)
{
    ++s_state.invalidFrameCount;
    /* A task start requires three uninterrupted accepted calibrated samples.
     * A malformed or reordered pose must not contribute to that evidence. */
    s_state.calibratedConsecutiveFrameCount = 0U;
}

static void CarPoseLink_ResetLegacy(uint8_t byte)
{
    s_legacyFrameIndex = 0U;
    if (byte == CAR_POSE_LINK_LEGACY_HEAD)
    {
        s_legacyFrame[s_legacyFrameIndex++] = byte;
    }
}

static void CarPoseLink_ConsumeLegacyByte(uint8_t byte, uint32_t nowMs)
{
    int32_t yawTenthsDeg;
    uint8_t index;

    if (s_legacyFrameIndex == 0U)
    {
        CarPoseLink_ResetLegacy(byte);
        return;
    }

    s_legacyFrame[s_legacyFrameIndex++] = byte;
    if (s_legacyFrameIndex < CAR_POSE_LINK_LEGACY_SIZE)
    {
        return;
    }

    /* Preserve the exact candidate that reached the 14-byte boundary before
     * validation.  This lets the diagnostic UART prove whether a large pose
     * originated in the Pi bridge or in local byte alignment/decoding. */
    for (index = 0U; index < CAR_POSE_LINK_LEGACY_SIZE; ++index)
    {
        s_state.lastLegacyFrame[index] = s_legacyFrame[index];
    }
    s_state.lastLegacyFrameAvailable = 1U;

    if ((s_legacyFrame[0] != CAR_POSE_LINK_LEGACY_HEAD) ||
        (s_legacyFrame[CAR_POSE_LINK_LEGACY_SIZE - 1U] != CAR_POSE_LINK_LEGACY_TAIL) ||
        (s_legacyFrame[1] > 1U) || (s_legacyFrame[5] > 1U) ||
        (s_legacyFrame[9] > 1U))
    {
        ++s_state.invalidFrameCount;
        ++s_state.legacyInvalidFrameCount;
        CarPoseLink_ResetLegacy(byte);
        return;
    }

    yawTenthsDeg = CarPoseLink_ReadLegacySigned3(s_legacyFrame[9],
                                                  &s_legacyFrame[10]);
    if ((yawTenthsDeg > CAR_POSE_LINK_MAX_YAW_TENTHS) ||
        (yawTenthsDeg < -CAR_POSE_LINK_MAX_YAW_TENTHS))
    {
        ++s_state.invalidFrameCount;
        ++s_state.legacyInvalidFrameCount;
        CarPoseLink_ResetLegacy(byte);
        return;
    }

    /* The legacy bridge carries no CRC, CalibrationId, velocity or Pi time.
     * Relay it only as explicitly uncalibrated preflight display data. */
    s_state.sourceFormat = CAR_POSE_LINK_SOURCE_LEGACY;
    s_state.sequence = 0U;
    s_state.coordinateFrame = V22_COORDINATE_FIELD_GLOBAL;
    s_state.poseFlags = V22_POSE_FLAG_POSITION_VALID | V22_POSE_FLAG_YAW_VALID;
    s_state.calibrationId = 0U;
    s_state.xCm = CarPoseLink_ReadLegacySigned3(s_legacyFrame[1],
                                                &s_legacyFrame[2]);
    s_state.yCm = CarPoseLink_ReadLegacySigned3(s_legacyFrame[5],
                                                &s_legacyFrame[6]);
    s_state.yawTenthsDeg = (int16_t)yawTenthsDeg;
    s_state.vxCmPerSec = CAR_POSE_LINK_INVALID_VELOCITY;
    s_state.vyCmPerSec = CAR_POSE_LINK_INVALID_VELOCITY;
    s_state.calibratedConsecutiveFrameCount = 0U;
    /* The legacy packet has no Pi timestamp.  Start a local relay epoch at
     * the first accepted legacy sample, rather than using MCU boot uptime.
     * A radar/Pi start delay must not hide an MCU reboot from the ground
     * station's source-time reset gate.  This remains uncalibrated display
     * telemetry only; native V2.2 Pi input supplies the real Pi timestamp. */
    if (s_legacyRelayEpochStarted == 0U)
    {
        s_legacyRelayEpochStarted = 1U;
        s_legacyRelaySourceTimeMs = CAR_POSE_LINK_LEGACY_FIRST_TIME_MS;
    }
    else
    {
        s_legacyRelaySourceTimeMs +=
            (uint32_t)(nowMs - s_legacyRelayLastReceiveMs);
    }
    s_legacyRelayLastReceiveMs = nowMs;
    s_state.sourceTimeMs = s_legacyRelaySourceTimeMs;
    s_state.lastFrameMs = nowMs;
    s_state.valid = 1U;
    ++s_state.validFrameCount;
    ++s_state.legacyFrameCount;
    s_legacyFrameIndex = 0U;
}

static void CarPoseLink_RecordParseDrop(V22ParseResult_t result)
{
    if (result == V22_PARSE_VERSION_ERROR)
    {
        ++s_state.versionErrorCount;
    }
    else if (result == V22_PARSE_LENGTH_ERROR)
    {
        ++s_state.lengthErrorCount;
    }
    else if (result == V22_PARSE_CRC_ERROR)
    {
        ++s_state.crcErrorCount;
    }
    else if (result == V22_PARSE_TIMEOUT)
    {
        ++s_state.timeoutCount;
    }
}

static void CarPoseLink_ConsumeAck(const V22Frame_t *frame)
{
    CarPoseLinkAck_t ack;

    if ((frame->source != V22_ADDR_CAR_PI) ||
        (frame->destination != V22_ADDR_CAR_MCU) ||
        (frame->flags != 0U) ||
        (frame->length != 4U) ||
        (frame->payload[2] > V22_ACK_RESULT_INTERNAL))
    {
        ++s_state.invalidFrameCount;
        ++s_state.ackInvalidFrameCount;
        return;
    }

    ack.requestType = frame->payload[0];
    ack.requestSeq = frame->payload[1];
    ack.result = frame->payload[2];
    ack.detail = frame->payload[3];
    s_state.lastAckRequestType = ack.requestType;
    s_state.lastAckRequestSeq = ack.requestSeq;
    s_state.lastAckResult = ack.result;
    s_state.lastAckDetail = ack.detail;
    CarPoseLink_QueueAck(&ack);
    ++s_state.ackFrameCount;
}

static void CarPoseLink_ConsumeFrame(const V22Frame_t *frame, uint32_t nowMs)
{
    uint8_t flags;
    uint8_t taskPose;
    uint8_t previousTaskPose;
    uint16_t calibrationId;
    int16_t yawTenthsDeg;
    int16_t vxCmPerSec;
    int16_t vyCmPerSec;
    uint32_t sourceTimeMs;
    int32_t sourceTimeDelta;

    if (frame->type == V22_TYPE_ACK)
    {
        CarPoseLink_ConsumeAck(frame);
        return;
    }

    if ((frame->type != V22_TYPE_CAR_POSE) ||
        (frame->source != V22_ADDR_CAR_PI) ||
        (frame->destination != V22_ADDR_CAR_MCU) ||
        (frame->flags != 0U) ||
        (frame->length != CAR_POSE_LINK_PAYLOAD_LENGTH) ||
        (frame->payload[0] != V22_COORDINATE_FIELD_GLOBAL))
    {
        CarPoseLink_RecordInvalidPose();
        return;
    }

    flags = frame->payload[1];
    calibrationId = CarPoseLink_ReadU16Be(&frame->payload[2]);
    yawTenthsDeg = CarPoseLink_ReadI16Be(&frame->payload[12]);
    vxCmPerSec = CarPoseLink_ReadI16Be(&frame->payload[14]);
    vyCmPerSec = CarPoseLink_ReadI16Be(&frame->payload[16]);
    sourceTimeMs = CarPoseLink_ReadU32Be(&frame->payload[18]);

    if (((flags & (uint8_t)~CAR_POSE_LINK_ALLOWED_FLAGS) != 0U) ||
        ((flags & (V22_POSE_FLAG_POSITION_VALID | V22_POSE_FLAG_YAW_VALID)) !=
         (V22_POSE_FLAG_POSITION_VALID | V22_POSE_FLAG_YAW_VALID)) ||
        (((flags & V22_POSE_FLAG_CALIBRATED) != 0U) && (calibrationId == 0U)) ||
        (((flags & V22_POSE_FLAG_CALIBRATED) == 0U) && (calibrationId != 0U)) ||
        (((flags & V22_POSE_FLAG_VELOCITY_VALID) == 0U) &&
         ((vxCmPerSec != CAR_POSE_LINK_INVALID_VELOCITY) ||
          (vyCmPerSec != CAR_POSE_LINK_INVALID_VELOCITY))) ||
        (yawTenthsDeg > CAR_POSE_LINK_MAX_YAW_TENTHS) ||
        (yawTenthsDeg < -CAR_POSE_LINK_MAX_YAW_TENTHS))
    {
        CarPoseLink_RecordInvalidPose();
        return;
    }

    if (((flags & V22_POSE_FLAG_VELOCITY_VALID) != 0U) &&
        ((vxCmPerSec == CAR_POSE_LINK_INVALID_VELOCITY) ||
         (vyCmPerSec == CAR_POSE_LINK_INVALID_VELOCITY)))
    {
        ++s_state.invalidVelocityCount;
        CarPoseLink_RecordInvalidPose();
        return;
    }

    taskPose = CarPoseLink_IsTaskPose(flags, calibrationId);
    previousTaskPose = ((s_state.valid != 0U) &&
                        (s_state.sourceFormat == CAR_POSE_LINK_SOURCE_V22) &&
                        CarPoseLink_IsTaskPose(s_state.poseFlags,
                                               s_state.calibrationId)) ? 1U : 0U;

    if ((s_state.valid != 0U) &&
        (s_state.sourceFormat == CAR_POSE_LINK_SOURCE_V22))
    {
        sourceTimeDelta = (int32_t)(sourceTimeMs - s_state.sourceTimeMs);
        if (sourceTimeDelta <= 0)
        {
            /* A Pi reboot invalidates calibration and begins with an
             * uncalibrated source-time epoch. Accept that explicit reset
             * indication, but never let a delayed/duplicate calibrated frame
             * refresh task readiness. */
            if ((sourceTimeDelta < 0) && (previousTaskPose != 0U) &&
                (taskPose == 0U))
            {
                ++s_state.sourceTimeRollbackCount;
            }
            else
            {
                ++s_state.outOfOrderFrameCount;
                CarPoseLink_RecordInvalidPose();
                return;
            }
        }
    }

    if (taskPose != 0U)
    {
        if ((previousTaskPose != 0U) &&
            (s_state.calibrationId == calibrationId))
        {
            if (s_state.calibratedConsecutiveFrameCount < 0xFFU)
            {
                ++s_state.calibratedConsecutiveFrameCount;
            }
        }
        else
        {
            s_state.calibratedConsecutiveFrameCount = 1U;
        }
    }
    else
    {
        s_state.calibratedConsecutiveFrameCount = 0U;
    }

    s_state.sequence = frame->sequence;
    s_state.sourceFormat = CAR_POSE_LINK_SOURCE_V22;
    s_state.coordinateFrame = frame->payload[0];
    s_state.poseFlags = flags;
    s_state.calibrationId = calibrationId;
    s_state.xCm = CarPoseLink_ReadI32Be(&frame->payload[4]);
    s_state.yCm = CarPoseLink_ReadI32Be(&frame->payload[8]);
    s_state.yawTenthsDeg = yawTenthsDeg;
    s_state.vxCmPerSec = vxCmPerSec;
    s_state.vyCmPerSec = vyCmPerSec;
    s_state.sourceTimeMs = sourceTimeMs;
    s_state.lastFrameMs = nowMs;
    s_state.valid = 1U;
    ++s_state.validFrameCount;
    ++s_state.v22FrameCount;
}

void CarPoseLink_Init(uint32_t baudrate)
{
    uint8_t *raw = (uint8_t *)&s_state;
    uint16_t index;

    RobotUart_RadarInit(baudrate);
    V22Protocol_ParserInit(&s_parser);
    s_legacyFrameIndex = 0U;
    s_legacyRelayEpochStarted = 0U;
    s_legacyRelaySourceTimeMs = 0U;
    s_legacyRelayLastReceiveMs = 0U;
    s_ackQueueHead = 0U;
    s_ackQueueTail = 0U;
    s_ackQueueCount = 0U;
    for (index = 0U; index < sizeof(s_state); ++index)
    {
        raw[index] = 0U;
    }
}

void CarPoseLink_Poll(uint32_t nowMs)
{
    uint8_t byte;
    uint16_t count = 0U;
    V22Frame_t frame;
    V22ParseResult_t result;

    result = V22Protocol_ParserCheckTimeout(&s_parser, nowMs);
    CarPoseLink_RecordParseDrop(result);

    while ((count < CAR_POSE_LINK_POLL_LIMIT) &&
           (RobotUart_RadarTryReadByte(&byte) != 0U))
    {
        ++s_state.rawByteCount;
        result = V22Protocol_ParserPush(&s_parser, byte, nowMs, &frame);
        if (result == V22_PARSE_FRAME)
        {
            CarPoseLink_ConsumeFrame(&frame, nowMs);
        }
        else
        {
            CarPoseLink_RecordParseDrop(result);
        }
        CarPoseLink_ConsumeLegacyByte(byte, nowMs);
        ++count;
    }
    s_state.uartErrorFlags |= RobotUart_RadarConsumeErrorFlags();
    s_state.uartRingOverflowCount += RobotUart_RadarConsumeOverflowCount();
}

const CarPoseLinkState_t *CarPoseLink_GetState(void)
{
    return &s_state;
}

uint8_t CarPoseLink_IsFresh(uint32_t nowMs, uint32_t maxAgeMs)
{
    if (s_state.valid == 0U)
    {
        return 0U;
    }
    return ((uint32_t)(nowMs - s_state.lastFrameMs) <= maxAgeMs) ? 1U : 0U;
}

uint8_t CarPoseLink_TakeAck(CarPoseLinkAck_t *ack)
{
    if ((ack == 0) || (s_ackQueueCount == 0U))
    {
        return 0U;
    }

    *ack = s_ackQueue[s_ackQueueTail];
    s_ackQueueTail = CarPoseLink_NextAckIndex(s_ackQueueTail);
    --s_ackQueueCount;
    return 1U;
}

void CarPoseLink_InvalidateCalibration(void)
{
    s_state.poseFlags &= (uint8_t)~V22_POSE_FLAG_CALIBRATED;
    s_state.calibrationId = 0U;
    s_state.calibratedConsecutiveFrameCount = 0U;
}

uint8_t CarPoseLink_IsTaskReady(uint32_t nowMs, uint32_t maxAgeMs)
{
    if (CarPoseLink_IsFresh(nowMs, maxAgeMs) == 0U)
    {
        return 0U;
    }
    if ((s_state.sourceFormat != CAR_POSE_LINK_SOURCE_V22) ||
        (s_state.coordinateFrame != V22_COORDINATE_FIELD_GLOBAL) ||
        (CarPoseLink_IsTaskPose(s_state.poseFlags,
                                s_state.calibrationId) == 0U) ||
        (s_state.calibratedConsecutiveFrameCount <
         CAR_POSE_LINK_TASK_READY_FRAME_COUNT))
    {
        return 0U;
    }
    return 1U;
}
