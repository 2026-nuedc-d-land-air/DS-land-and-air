/** Pi radar/SLAM 14-byte pose-frame parser; see bsp_radar_pose.h. */

#include "bsp_radar_pose.h"

#include "bsp_robot_uart.h"

#define RADAR_POSE_HEAD                    0xFAU
#define RADAR_POSE_TAIL                    0xABU
#define RADAR_POSE_FRAME_SIZE                 14U
#define RADAR_POSE_POLL_LIMIT                128U
#define RADAR_POSE_MAX_YAW_TENTHS           1800L

static RadarPoseState_t s_state;
static uint8_t s_frame[RADAR_POSE_FRAME_SIZE];
static uint8_t s_frameIndex;

static int32_t RadarPose_ReadSigned3(uint8_t sign, const uint8_t *magnitude)
{
    int32_t value;

    value = ((int32_t)magnitude[0] << 16) |
            ((int32_t)magnitude[1] << 8) |
            (int32_t)magnitude[2];
    return (sign != 0U) ? -value : value;
}

static void RadarPose_ResetParser(uint8_t byte)
{
    s_frameIndex = 0U;
    if (byte == RADAR_POSE_HEAD)
    {
        s_frame[s_frameIndex++] = byte;
        ++s_state.frameHeadCount;
    }
}

static void RadarPose_ConsumeByte(uint8_t byte, uint32_t nowMs)
{
    int32_t xCm;
    int32_t yCm;
    int32_t yawTenths;

    ++s_state.rawByteCount;

    if (s_frameIndex == 0U)
    {
        RadarPose_ResetParser(byte);
        return;
    }

    s_frame[s_frameIndex++] = byte;
    if (s_frameIndex < RADAR_POSE_FRAME_SIZE)
    {
        return;
    }

    if ((s_frame[0] != RADAR_POSE_HEAD) ||
        (s_frame[RADAR_POSE_FRAME_SIZE - 1U] != RADAR_POSE_TAIL) ||
        (s_frame[1] > 1U) || (s_frame[5] > 1U) || (s_frame[9] > 1U))
    {
        ++s_state.invalidFrameCount;
        RadarPose_ResetParser(byte);
        return;
    }

    xCm = RadarPose_ReadSigned3(s_frame[1], &s_frame[2]);
    yCm = RadarPose_ReadSigned3(s_frame[5], &s_frame[6]);
    yawTenths = RadarPose_ReadSigned3(s_frame[9], &s_frame[10]);
    if ((yawTenths > RADAR_POSE_MAX_YAW_TENTHS) ||
        (yawTenths < -RADAR_POSE_MAX_YAW_TENTHS))
    {
        ++s_state.invalidFrameCount;
        RadarPose_ResetParser(byte);
        return;
    }

    s_state.xCm = xCm;
    s_state.yCm = yCm;
    s_state.yawTenthsDeg = (int16_t)yawTenths;
    s_state.lastFrameMs = nowMs;
    s_state.valid = 1U;
    ++s_state.validFrameCount;
    s_frameIndex = 0U;
}

void RadarPose_Init(uint32_t baudrate)
{
    uint8_t index;

    RobotUart_RadarInit(baudrate);
    s_state.valid = 0U;
    s_state.xCm = 0;
    s_state.yCm = 0;
    s_state.yawTenthsDeg = 0;
    s_state.lastFrameMs = 0U;
    s_state.rawByteCount = 0U;
    s_state.frameHeadCount = 0U;
    s_state.validFrameCount = 0U;
    s_state.invalidFrameCount = 0U;
    s_state.uartErrorFlags = 0U;
    s_frameIndex = 0U;
    for (index = 0U; index < RADAR_POSE_FRAME_SIZE; ++index)
    {
        s_frame[index] = 0U;
    }
}

void RadarPose_Poll(uint32_t nowMs)
{
    uint8_t byte;
    uint16_t count = 0U;

    while ((count < RADAR_POSE_POLL_LIMIT) &&
           (RobotUart_RadarTryReadByte(&byte) != 0U))
    {
        RadarPose_ConsumeByte(byte, nowMs);
        ++count;
    }
    s_state.uartErrorFlags |= RobotUart_RadarConsumeErrorFlags();
}

const RadarPoseState_t *RadarPose_GetState(void)
{
    return &s_state;
}

uint8_t RadarPose_IsFresh(uint32_t nowMs, uint32_t maxAgeMs)
{
    if (s_state.valid == 0U)
    {
        return 0U;
    }
    return (((uint32_t)(nowMs - s_state.lastFrameMs) <= maxAgeMs) ? 1U : 0U);
}
