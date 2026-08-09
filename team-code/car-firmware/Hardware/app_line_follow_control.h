#ifndef __APP_LINE_FOLLOW_CONTROL_H
#define __APP_LINE_FOLLOW_CONTROL_H

#include "stm32f10x.h"

typedef enum
{
    LINE_FOLLOW_CTRL_UNCAL = 0,
    LINE_FOLLOW_CTRL_TRACK,
    LINE_FOLLOW_CTRL_LOST,
    LINE_FOLLOW_CTRL_A_MARK,
    LINE_FOLLOW_CTRL_WIDE
} LineFollowControlState_t;

typedef struct
{
    int16_t baseForwardCommand;
    int16_t errorX100;
    int16_t steerDemand;
    int16_t kpNumerator;
    int16_t kpDenominator;
    int16_t steerDemandLimit;
    uint8_t motorPermit;
    LineFollowControlState_t state;
} LineFollowController_t;

void LineFollowControl_Init(LineFollowController_t *controller);
void LineFollowControl_Update(LineFollowController_t *controller,
                              uint8_t whiteCalibrated,
                              uint8_t stableMask,
                              uint8_t activeCount,
                              int16_t errorX100);
const char *LineFollowControl_StateName(const LineFollowController_t *controller);
const char *LineFollowControl_SteerName(const LineFollowController_t *controller);

#endif /* __APP_LINE_FOLLOW_CONTROL_H */
