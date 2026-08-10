#include "app_line_observer.h"

#include "bsp_gray_tracking.h"
#include "bsp_gyro_wit.h"

#define LINE_OBSERVER_WHITE_RAW_MASK         0x00U
#define LINE_OBSERVER_FULL_MASK              0xFFU
#define LINE_OBSERVER_CENTER_MASK             0x18U
#define LINE_OBSERVER_STABLE_SAMPLES            2U
/* A zero mask is safety-significant, but it is also the most likely result
 * of a short optical/electrical disturbance.  Preserve the last verified
 * line position until three consecutive scans agree on all-white. */
#define LINE_OBSERVER_LOST_CONFIRM_SAMPLES      3U
#define LINE_OBSERVER_MAX_NORMAL_BITS           3U
#define LINE_OBSERVER_GYRO_STALE_MS           250U
#define LINE_OBSERVER_MAX_DIFF_CPS           5200
#define LINE_OBSERVER_MAX_GYRO_DIFF_CPS       300
#define LINE_OBSERVER_MAX_TOTAL_DIFF_CPS     5200
#define LINE_OBSERVER_MAX_YAW_REF_TENTHS     1200
#define LINE_OBSERVER_MAX_HEADING_ERROR_TENTHS 600
#define LINE_OBSERVER_MAX_YAW_RATE_TENTHS  30000

/* X1 is vehicle-left, X8 is vehicle-right. The map is physically verified. */
static const int8_t s_channelWeight[8] = {-7, -5, -3, -1, 1, 3, 5, 7};

static uint8_t s_candidateMask;
static uint8_t s_candidateCount;
static uint32_t s_lastGyroAngleFrameCount;
static uint32_t s_lastGyroAngleMs;
static int16_t s_lastGyroYawTenthsDeg;

static int16_t LineObserver_Clamp(int32_t value, int16_t lower, int16_t upper)
{
    if (value > upper) { return upper; }
    if (value < lower) { return lower; }
    return (int16_t)value;
}

static int16_t LineObserver_Abs(int16_t value)
{
    return (value < 0) ? (int16_t)-value : value;
}

static int16_t LineObserver_WrapTenthsDeg(int32_t value)
{
    while (value > 1800) { value -= 3600; }
    while (value < -1800) { value += 3600; }
    return (int16_t)value;
}

static uint8_t LineObserver_CountBits(uint8_t mask)
{
    uint8_t count = 0U;
    while (mask != 0U)
    {
        count += (uint8_t)(mask & 1U);
        mask >>= 1U;
    }
    return count;
}

static uint8_t LineObserver_IsCenterCapture(uint8_t mask)
{
    return (((mask & LINE_OBSERVER_CENTER_MASK) != 0U) &&
            ((mask & (uint8_t)~LINE_OBSERVER_CENTER_MASK) == 0U)) ? 1U : 0U;
}

static void LineObserver_UpdateLineClass(LineObserver_t *observer)
{
    int16_t weightedSum = 0;
    uint8_t channel;
    int16_t absError;
    int32_t grayMagnitude;
    int32_t yawMagnitude;

    observer->activeCount = LineObserver_CountBits(observer->stableMask);
    observer->centerCaptureActive = 0U;
    observer->grayErrorX100 = 0;
    observer->grayDifferentialCps = 0;
    observer->yawRateReferenceTenthsPerSec = 0;

    if (observer->stableMask == LINE_OBSERVER_FULL_MASK)
    {
        observer->lineClass = LINE_OBSERVER_A_MARK;
        return;
    }
    if (observer->stableMask == 0U)
    {
        observer->lineClass = LINE_OBSERVER_LOST;
        return;
    }

    observer->lineClass = (observer->activeCount > LINE_OBSERVER_MAX_NORMAL_BITS) ?
                          LINE_OBSERVER_WIDE : LINE_OBSERVER_TRACK;
    /* A line seen by either centre sensor, or by both, is centred closely
     * enough that chasing the one-bit mask change only adds steering noise. */
    observer->centerCaptureActive =
        LineObserver_IsCenterCapture(observer->stableMask);
    if (observer->centerCaptureActive != 0U)
    {
        return;
    }

    for (channel = 0U; channel < 8U; ++channel)
    {
        if ((observer->stableMask & (uint8_t)(1U << channel)) != 0U)
        {
            weightedSum += s_channelWeight[channel];
        }
    }
    observer->grayErrorX100 = (int16_t)((weightedSum * 100) /
                                         (int16_t)observer->activeCount);
    absError = LineObserver_Abs(observer->grayErrorX100);

    /* Keep the response continuous while making the outer two sensor bands
     * decisive enough to pull the inside wheel through zero on a tight bend.
     * Mission-side target slew still bounds each 20 ms wheel-target change. */
    if (absError <= 100)
    {
        grayMagnitude = (int32_t)absError * 2L;
        yawMagnitude = (int32_t)absError * 3L;
    }
    else if (absError <= 300)
    {
        grayMagnitude = 200L + ((int32_t)(absError - 100) * 1000L) / 200L;
        yawMagnitude = 300L + ((int32_t)(absError - 100) * 900L) / 200L;
    }
    else if (absError <= 500)
    {
        grayMagnitude = 1200L + ((int32_t)(absError - 300) * 2400L) / 200L;
        yawMagnitude = LINE_OBSERVER_MAX_YAW_REF_TENTHS;
    }
    else
    {
        grayMagnitude = 3600L + ((int32_t)(absError - 500) * 1600L) / 100L;
        yawMagnitude = LINE_OBSERVER_MAX_YAW_REF_TENTHS;
    }

    observer->grayDifferentialCps = LineObserver_Clamp(
        (observer->grayErrorX100 < 0) ? grayMagnitude : -grayMagnitude,
        -LINE_OBSERVER_MAX_DIFF_CPS, LINE_OBSERVER_MAX_DIFF_CPS);
    observer->yawRateReferenceTenthsPerSec = LineObserver_Clamp(
        (observer->grayErrorX100 < 0) ? yawMagnitude : -yawMagnitude,
        -LINE_OBSERVER_MAX_YAW_REF_TENTHS, LINE_OBSERVER_MAX_YAW_REF_TENTHS);
}

static void LineObserver_UpdateGyro(LineObserver_t *observer, uint32_t nowMs)
{
    const volatile GyroWitState_t *gyro;
    int32_t headingAdvance;
    int32_t correction;
    int16_t yawRateError;
    uint32_t periodMs;

    (void)GyroWit_Poll();
    gyro = GyroWit_GetState();
    observer->yawTenthsDeg = gyro->yawTenthsDeg;

    /* JY901 angle frames are verified, while 0x52 rate frames are not enabled
     * in the present configuration. Derive yaw rate from consecutive 0x53
     * angle samples instead of pretending the rate is always zero. */
    if (gyro->angleFrameCount != s_lastGyroAngleFrameCount)
    {
        if (s_lastGyroAngleFrameCount != 0U)
        {
            periodMs = nowMs - s_lastGyroAngleMs;
            if ((periodMs >= 5U) && (periodMs <= 1000U))
            {
                observer->yawRateTenthsPerSec = LineObserver_Clamp(
                    ((int32_t)LineObserver_WrapTenthsDeg(
                        (int32_t)gyro->yawTenthsDeg - s_lastGyroYawTenthsDeg) * 1000L) /
                    (int32_t)periodMs,
                    -LINE_OBSERVER_MAX_YAW_RATE_TENTHS,
                    LINE_OBSERVER_MAX_YAW_RATE_TENTHS);
            }
        }
        s_lastGyroAngleFrameCount = gyro->angleFrameCount;
        s_lastGyroAngleMs = nowMs;
        s_lastGyroYawTenthsDeg = gyro->yawTenthsDeg;
    }

    observer->gyroAgeMs = nowMs - s_lastGyroAngleMs;
    observer->gyroFresh = ((gyro->angleFrameCount != 0U) &&
                           (observer->gyroAgeMs <= LINE_OBSERVER_GYRO_STALE_MS)) ?
                          1U : 0U;
    observer->gyroDifferentialCps = 0;

    if (observer->gyroFresh == 0U)
    {
        observer->headingErrorTenthsDeg = 0;
        observer->totalDifferentialCps = observer->grayDifferentialCps;
        return;
    }

    if (observer->centerCaptureActive != 0U)
    {
        /* Re-anchor after every centre capture so a previous curve cannot
         * keep steering after the gray sensors have recovered the line. */
        observer->headingReferenceTenthsDeg = observer->yawTenthsDeg;
        observer->headingErrorTenthsDeg = 0;
        observer->gyroDifferentialCps = LineObserver_Clamp(
            -((int32_t)observer->yawRateTenthsPerSec / 4L),
            -LINE_OBSERVER_MAX_GYRO_DIFF_CPS,
            LINE_OBSERVER_MAX_GYRO_DIFF_CPS);
        observer->totalDifferentialCps = observer->gyroDifferentialCps;
        return;
    }

    headingAdvance = ((int32_t)observer->yawRateReferenceTenthsPerSec * 20L) / 1000L;
    observer->headingReferenceTenthsDeg = LineObserver_WrapTenthsDeg(
        (int32_t)observer->headingReferenceTenthsDeg + headingAdvance);

    /* Stop the virtual heading reference winding around 180 degrees on a
     * long bend, which would otherwise flip the gyro correction abruptly. */
    observer->headingErrorTenthsDeg = LineObserver_Clamp(
        LineObserver_WrapTenthsDeg(
            (int32_t)observer->headingReferenceTenthsDeg - observer->yawTenthsDeg),
        -LINE_OBSERVER_MAX_HEADING_ERROR_TENTHS,
        LINE_OBSERVER_MAX_HEADING_ERROR_TENTHS);
    observer->headingReferenceTenthsDeg = LineObserver_WrapTenthsDeg(
        (int32_t)observer->yawTenthsDeg + observer->headingErrorTenthsDeg);
    yawRateError = (int16_t)(observer->yawRateReferenceTenthsPerSec -
                             observer->yawRateTenthsPerSec);
    correction = ((int32_t)yawRateError / 4L) +
                 ((int32_t)observer->headingErrorTenthsDeg / 6L);
    observer->gyroDifferentialCps = LineObserver_Clamp(
        correction, -LINE_OBSERVER_MAX_GYRO_DIFF_CPS,
        LINE_OBSERVER_MAX_GYRO_DIFF_CPS);
    observer->totalDifferentialCps = LineObserver_Clamp(
        (int32_t)observer->grayDifferentialCps + observer->gyroDifferentialCps,
        -LINE_OBSERVER_MAX_TOTAL_DIFF_CPS,
        LINE_OBSERVER_MAX_TOTAL_DIFF_CPS);
}

void LineObserver_Init(LineObserver_t *observer, uint32_t nowMs)
{
    GrayTrackingSample_t sample;
    if (observer == 0) { return; }

    GrayTracking_Init();
    GrayTracking_Read(&sample);
    observer->rawMask = sample.rawMask;
    observer->activeMask = (uint8_t)(sample.rawMask ^ LINE_OBSERVER_WHITE_RAW_MASK);
    observer->stableMask = observer->activeMask;
    observer->lostConfirmCount =
        (observer->activeMask == 0U) ? LINE_OBSERVER_LOST_CONFIRM_SAMPLES : 0U;
    observer->activeCount = 0U;
    observer->centerCaptureActive = 0U;
    observer->gyroFresh = 0U;
    observer->radarPoseValid = 0U;
    observer->gyroAgeMs = 0U;
    observer->radarXcm = 0;
    observer->radarYcm = 0;
    observer->grayErrorX100 = 0;
    observer->grayDifferentialCps = 0;
    observer->yawRateReferenceTenthsPerSec = 0;
    observer->yawTenthsDeg = 0;
    observer->yawRateTenthsPerSec = 0;
    observer->headingReferenceTenthsDeg = 0;
    observer->headingErrorTenthsDeg = 0;
    observer->gyroDifferentialCps = 0;
    observer->totalDifferentialCps = 0;
    observer->lineClass = LINE_OBSERVER_LOST;
    s_candidateMask = observer->activeMask;
    s_candidateCount = (observer->activeMask == 0U) ?
                       LINE_OBSERVER_LOST_CONFIRM_SAMPLES :
                       LINE_OBSERVER_STABLE_SAMPLES;
    s_lastGyroAngleFrameCount = 0U;
    s_lastGyroAngleMs = nowMs;
    s_lastGyroYawTenthsDeg = 0;
    LineObserver_UpdateLineClass(observer);
}

void LineObserver_ResetHeadingReference(LineObserver_t *observer)
{
    if (observer == 0) { return; }
    observer->headingReferenceTenthsDeg = observer->yawTenthsDeg;
    observer->headingErrorTenthsDeg = 0;
    observer->gyroDifferentialCps = 0;
    observer->totalDifferentialCps = observer->grayDifferentialCps;
}

void LineObserver_ResetRadarOrigin(LineObserver_t *observer)
{
    if (observer == 0) { return; }
    observer->radarXcm = 0;
    observer->radarYcm = 0;
    observer->radarPoseValid = 0U;
}

void LineObserver_SetRadarPose(LineObserver_t *observer, int32_t xCm, int32_t yCm,
                               uint8_t valid)
{
    if (observer == 0) { return; }
    observer->radarXcm = xCm;
    observer->radarYcm = yCm;
    observer->radarPoseValid = (valid != 0U) ? 1U : 0U;
}

void LineObserver_Update(LineObserver_t *observer, uint32_t nowMs)
{
    GrayTrackingSample_t sample;
    uint8_t stableSamplesRequired;
    if (observer == 0) { return; }

    GrayTracking_Read(&sample);
    observer->rawMask = sample.rawMask;
    observer->activeMask = (uint8_t)(sample.rawMask ^ LINE_OBSERVER_WHITE_RAW_MASK);
    stableSamplesRequired = (observer->activeMask == 0U) ?
                            LINE_OBSERVER_LOST_CONFIRM_SAMPLES :
                            LINE_OBSERVER_STABLE_SAMPLES;
    if (observer->activeMask != s_candidateMask)
    {
        s_candidateMask = observer->activeMask;
        s_candidateCount = 1U;
    }
    else if (s_candidateCount < stableSamplesRequired)
    {
        ++s_candidateCount;
    }
    observer->lostConfirmCount = (observer->activeMask == 0U) ?
                                 s_candidateCount : 0U;
    if (s_candidateCount >= stableSamplesRequired)
    {
        observer->stableMask = s_candidateMask;
    }

    LineObserver_UpdateLineClass(observer);
    LineObserver_UpdateGyro(observer, nowMs);
}

const char *LineObserver_ClassName(LineObserverClass_t lineClass)
{
    switch (lineClass)
    {
        case LINE_OBSERVER_A_MARK: return "A_MARK";
        case LINE_OBSERVER_TRACK: return "TRACK";
        case LINE_OBSERVER_WIDE: return "WIDE";
        case LINE_OBSERVER_LOST:
        default: return "LOST";
    }
}
