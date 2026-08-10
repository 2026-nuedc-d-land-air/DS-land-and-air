#include "app_gray_tracking_test.h"

#include "app_line_follow_control.h"
#include "bsp_diag_uart.h"
#include "bsp_gray_tracking.h"

#define SAMPLE_PERIOD_MS          20U
#define REPORT_PERIOD_MS          200U
#define CALIBRATION_SAMPLES       64U
#define STABLE_SAMPLE_COUNT       3U
#define GRAY_FULL_MASK            0xFFU
#define GRAY_NORMAL_LINE_BITS     3U

/* X1 is vehicle-left and X8 is vehicle-right in the verified installation. */
static const int8_t s_channelWeight[8] = {-7, -5, -3, -1, 1, 3, 5, 7};

static GrayTrackingSample_t s_sample;
static uint8_t s_whiteMask;
static uint8_t s_blackMask;
static uint8_t s_whiteValid;
static uint8_t s_blackValid;
static uint8_t s_rawMask;
static uint8_t s_activeMask;
static uint8_t s_candidateMask;
static uint8_t s_stableMask;
static uint8_t s_candidateCount;
static uint8_t s_streamEnabled;
static uint8_t s_mark;
static uint8_t s_activeCount;
static int16_t s_errorX100;
static LineFollowController_t s_lineController;
static uint32_t s_lastSampleMs;
static uint32_t s_lastReportMs;

static uint8_t TimeReached(uint32_t nowMs, uint32_t deadlineMs)
{
    return ((int32_t)(nowMs - deadlineMs) >= 0) ? 1U : 0U;
}

static uint8_t GrayTracking_MajorityMask(void)
{
    uint8_t highCount[8] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    GrayTrackingSample_t sample;
    uint8_t sampleIndex;
    uint8_t channel;
    uint8_t mask = 0U;

    for (sampleIndex = 0U; sampleIndex < CALIBRATION_SAMPLES; ++sampleIndex)
    {
        GrayTracking_Read(&sample);
        for (channel = 0U; channel < 8U; ++channel)
        {
            if ((sample.rawMask & (uint8_t)(1U << channel)) != 0U)
            {
                ++highCount[channel];
            }
        }
    }

    for (channel = 0U; channel < 8U; ++channel)
    {
        if (highCount[channel] >= (CALIBRATION_SAMPLES / 2U))
        {
            mask |= (uint8_t)(1U << channel);
        }
    }
    return mask;
}

static uint8_t GrayTracking_ActiveMask(uint8_t rawMask)
{
    return (s_whiteValid != 0U) ? (uint8_t)(rawMask ^ s_whiteMask) : 0U;
}

static const char *GrayTracking_StateName(void)
{
    if (s_whiteValid == 0U)
    {
        return "UNCAL";
    }
    return (s_stableMask == 0U) ? "WHITE" : "TRIGGER";
}

static uint8_t GrayTracking_CountActiveBits(uint8_t mask)
{
    uint8_t count = 0U;

    while (mask != 0U)
    {
        count += (uint8_t)(mask & 1U);
        mask >>= 1U;
    }
    return count;
}

static const char *GrayTracking_LineClassName(void)
{
    if (s_whiteValid == 0U)
    {
        return "UNCAL";
    }
    if (s_stableMask == 0U)
    {
        return "LOST";
    }
    if (s_stableMask == GRAY_FULL_MASK)
    {
        return "A_MARK";
    }
    return (s_activeCount <= GRAY_NORMAL_LINE_BITS) ? "LINE" : "WIDE";
}

static const char *GrayTracking_LineSideName(void)
{
    if ((s_whiteValid == 0U) || (s_stableMask == 0U) ||
        (s_stableMask == GRAY_FULL_MASK))
    {
        return "N/A";
    }
    if (s_errorX100 < 0)
    {
        return "VEH_LEFT";
    }
    if (s_errorX100 > 0)
    {
        return "VEH_RIGHT";
    }
    return "CENTER";
}

static void GrayTracking_UpdateLineObservation(void)
{
    int16_t weightedSum = 0;
    uint8_t channel;

    s_activeCount = GrayTracking_CountActiveBits(s_stableMask);
    s_errorX100 = 0;

    if ((s_whiteValid != 0U) && (s_stableMask != 0U) &&
        (s_stableMask != GRAY_FULL_MASK))
    {
        for (channel = 0U; channel < 8U; ++channel)
        {
            if ((s_stableMask & (uint8_t)(1U << channel)) != 0U)
            {
                weightedSum += s_channelWeight[channel];
            }
        }
        s_errorX100 = (int16_t)((weightedSum * 100) / (int16_t)s_activeCount);
    }

    LineFollowControl_Update(&s_lineController,
                             s_whiteValid,
                             s_stableMask,
                             s_activeCount,
                             s_errorX100);
}

static void GrayTracking_WriteRecord(const char *event, uint32_t nowMs)
{
    uint8_t calFlags = (uint8_t)((s_whiteValid != 0U ? 1U : 0U) |
                                 (s_blackValid != 0U ? 2U : 0U));

    DiagUart_WriteString("GRAY,event,");
    DiagUart_WriteString(event);
    DiagUart_WriteString(",t_ms,");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",raw_mask,");
    DiagUart_WriteUInt32(s_rawMask);
    DiagUart_WriteString(",active_mask,");
    DiagUart_WriteUInt32(s_activeMask);
    DiagUart_WriteString(",stable_mask,");
    DiagUart_WriteUInt32(s_stableMask);
    DiagUart_WriteString(",state,");
    DiagUart_WriteString(GrayTracking_StateName());
    DiagUart_WriteString(",line_class,");
    DiagUart_WriteString(GrayTracking_LineClassName());
    DiagUart_WriteString(",line_side,");
    DiagUart_WriteString(GrayTracking_LineSideName());
    DiagUart_WriteString(",active_count,");
    DiagUart_WriteUInt32(s_activeCount);
    DiagUart_WriteString(",err_x100,");
    DiagUart_WriteInt32((int32_t)s_errorX100);
    DiagUart_WriteString(",ctrl_state,");
    DiagUart_WriteString(LineFollowControl_StateName(&s_lineController));
    DiagUart_WriteString(",target_vx,");
    DiagUart_WriteInt32((int32_t)s_lineController.baseForwardCommand);
    DiagUart_WriteString(",steer_demand,");
    DiagUart_WriteInt32((int32_t)s_lineController.steerDemand);
    DiagUart_WriteString(",steer_dir,");
    DiagUart_WriteString(LineFollowControl_SteerName(&s_lineController));
    DiagUart_WriteString(",motor_permit,");
    DiagUart_WriteUInt32(s_lineController.motorPermit);
    DiagUart_WriteString(",cal,");
    DiagUart_WriteUInt32(calFlags);
    DiagUart_WriteString(",mark,");
    DiagUart_WriteUInt32(s_mark);
    DiagUart_WriteString("\r\n");
}

static void GrayTracking_CaptureReference(uint8_t black, uint32_t nowMs)
{
    s_rawMask = GrayTracking_MajorityMask();
    s_sample.rawMask = s_rawMask;

    if (black != 0U)
    {
        s_blackMask = s_rawMask;
        s_blackValid = 1U;
        s_activeMask = GrayTracking_ActiveMask(s_rawMask);
        GrayTracking_UpdateLineObservation();
        GrayTracking_WriteRecord("cal_black", nowMs);
    }
    else
    {
        s_whiteMask = s_rawMask;
        s_whiteValid = 1U;
        s_activeMask = 0U;
        s_candidateMask = 0U;
        s_stableMask = 0U;
        s_candidateCount = 0U;
        GrayTracking_UpdateLineObservation();
        GrayTracking_WriteRecord("cal_white", nowMs);
    }
}

static void GrayTracking_Sample(uint32_t nowMs)
{
    uint8_t previousMask = s_stableMask;
    uint8_t stableChanged = 0U;

    GrayTracking_Read(&s_sample);
    s_rawMask = s_sample.rawMask;
    s_activeMask = GrayTracking_ActiveMask(s_rawMask);

    if (s_activeMask != s_candidateMask)
    {
        s_candidateMask = s_activeMask;
        s_candidateCount = 1U;
    }
    else if (s_candidateCount < STABLE_SAMPLE_COUNT)
    {
        ++s_candidateCount;
    }

    if ((s_candidateCount >= STABLE_SAMPLE_COUNT) &&
        (s_stableMask != s_candidateMask))
    {
        s_stableMask = s_candidateMask;
        stableChanged = 1U;
    }

    GrayTracking_UpdateLineObservation();
    if ((stableChanged != 0U) && (previousMask != s_stableMask))
    {
        GrayTracking_WriteRecord("trigger_change", nowMs);
    }
}

void GrayTrackingTest_Init(uint32_t nowMs)
{
    GrayTracking_Init();
    GrayTracking_Read(&s_sample);
    s_whiteMask = 0U;
    s_blackMask = 0U;
    s_whiteValid = 0U;
    s_blackValid = 0U;
    s_rawMask = s_sample.rawMask;
    s_activeMask = 0U;
    s_candidateMask = 0U;
    s_stableMask = 0U;
    s_candidateCount = 0U;
    s_streamEnabled = 1U;
    s_mark = 0U;
    s_activeCount = 0U;
    s_errorX100 = 0;
    LineFollowControl_Init(&s_lineController);
    GrayTracking_UpdateLineObservation();
    s_lastSampleMs = nowMs;
    s_lastReportMs = nowMs;

    DiagUart_WriteString("GRAY,boot,pins=pc0_ad0_pc1_ad1_pc2_ad2_pg0_out,mode=mux8_control_dryrun,motors=off\r\n");
    DiagUart_WriteString("GRAY,commands,W=all_white,K=black_snapshot,R=sample,F=freeze,1-8=mark,L=stream,H=help\r\n");
}

void GrayTrackingTest_Update(uint32_t nowMs)
{
    if (TimeReached(nowMs, s_lastSampleMs + SAMPLE_PERIOD_MS) != 0U)
    {
        s_lastSampleMs = nowMs;
        GrayTracking_Sample(nowMs);
    }

    if ((s_streamEnabled != 0U) &&
        (TimeReached(nowMs, s_lastReportMs + REPORT_PERIOD_MS) != 0U))
    {
        s_lastReportMs = nowMs;
        GrayTracking_WriteRecord("sample", nowMs);
    }
}

void GrayTrackingTest_HandleCommand(char command, uint32_t nowMs)
{
    if ((command >= '1') && (command <= '8'))
    {
        s_mark = (uint8_t)(command - '0');
        GrayTracking_WriteRecord("mark", nowMs);
        return;
    }

    switch (command)
    {
        case 'W':
        case 'w':
            GrayTracking_CaptureReference(0U, nowMs);
            break;
        case 'K':
        case 'k':
            GrayTracking_CaptureReference(1U, nowMs);
            break;
        case 'R':
        case 'r':
            GrayTracking_Sample(nowMs);
            GrayTracking_WriteRecord("sample", nowMs);
            break;
        case 'F':
        case 'f':
            GrayTracking_WriteRecord("freeze", nowMs);
            break;
        case 'L':
        case 'l':
            s_streamEnabled = (s_streamEnabled == 0U) ? 1U : 0U;
            GrayTracking_WriteRecord((s_streamEnabled != 0U) ?
                                      "stream_on" : "stream_off", nowMs);
            break;
        case 'H':
        case 'h':
        case '?':
            DiagUart_WriteString("GRAY,commands,W=all_white,K=black_snapshot,R=sample,F=freeze,1-8=mark,L=stream,H=help\r\n");
            break;
        default:
            break;
    }
}
