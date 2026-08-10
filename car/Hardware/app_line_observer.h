/** Gray-line and direct JY901 joint observation state. */

#ifndef __APP_LINE_OBSERVER_H
#define __APP_LINE_OBSERVER_H

#include "stm32f10x.h"

typedef enum
{
    LINE_OBSERVER_A_MARK = 0,
    LINE_OBSERVER_TRACK,
    LINE_OBSERVER_LOST,
    LINE_OBSERVER_WIDE
} LineObserverClass_t;

typedef struct
{
    uint8_t rawMask;
    uint8_t activeMask;
    uint8_t stableMask;
    /* Consecutive raw all-white reads pending before stableMask may become
     * zero.  This makes rotor/light/connector glitches observable in F. */
    uint8_t lostConfirmCount;
    uint8_t activeCount;
    uint8_t centerCaptureActive;
    uint8_t gyroFresh;
    uint8_t radarPoseValid;
    uint32_t gyroAgeMs;
    int32_t radarXcm;
    int32_t radarYcm;
    int16_t grayErrorX100;
    int16_t grayDifferentialCps;
    int16_t yawRateReferenceTenthsPerSec;
    int16_t yawTenthsDeg;
    int16_t yawRateTenthsPerSec;
    int16_t headingReferenceTenthsDeg;
    int16_t headingErrorTenthsDeg;
    int16_t gyroDifferentialCps;
    int16_t totalDifferentialCps;
    LineObserverClass_t lineClass;
} LineObserver_t;

void LineObserver_Init(LineObserver_t *observer, uint32_t nowMs);
void LineObserver_ResetHeadingReference(LineObserver_t *observer);
void LineObserver_ResetRadarOrigin(LineObserver_t *observer);
void LineObserver_SetRadarPose(LineObserver_t *observer, int32_t xCm, int32_t yCm,
                               uint8_t valid);
void LineObserver_Update(LineObserver_t *observer, uint32_t nowMs);
const char *LineObserver_ClassName(LineObserverClass_t lineClass);

#endif /* __APP_LINE_OBSERVER_H */
