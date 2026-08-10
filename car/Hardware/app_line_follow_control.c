#include "app_line_follow_control.h"

/*
 * This module intentionally produces intent only.  It never initializes a
 * motor, changes STBY, or writes PWM.  The first physical-control image must
 * bind this intent to the already verified front speed loop in a later stage.
 */
#define LINE_FOLLOW_FULL_MASK              0xFFU
#define LINE_FOLLOW_MAX_NORMAL_ACTIVE_BITS 3U
#define LINE_FOLLOW_INITIAL_FORWARD_CMD    400
#define LINE_FOLLOW_KP_NUMERATOR           2
#define LINE_FOLLOW_KP_DENOMINATOR         1
#define LINE_FOLLOW_STEER_LIMIT            250

static int16_t LineFollowControl_Clamp(int32_t value,
                                       int16_t lower,
                                       int16_t upper)
{
    if (value > upper)
    {
        return upper;
    }
    if (value < lower)
    {
        return lower;
    }
    return (int16_t)value;
}

void LineFollowControl_Init(LineFollowController_t *controller)
{
    if (controller == 0)
    {
        return;
    }

    controller->baseForwardCommand = 0;
    controller->errorX100 = 0;
    controller->steerDemand = 0;
    controller->kpNumerator = LINE_FOLLOW_KP_NUMERATOR;
    controller->kpDenominator = LINE_FOLLOW_KP_DENOMINATOR;
    controller->steerDemandLimit = LINE_FOLLOW_STEER_LIMIT;
    controller->motorPermit = 0U;
    controller->state = LINE_FOLLOW_CTRL_UNCAL;
}

void LineFollowControl_Update(LineFollowController_t *controller,
                              uint8_t whiteCalibrated,
                              uint8_t stableMask,
                              uint8_t activeCount,
                              int16_t errorX100)
{
    int32_t demand;

    if (controller == 0)
    {
        return;
    }

    controller->baseForwardCommand = 0;
    controller->errorX100 = errorX100;
    controller->steerDemand = 0;
    controller->motorPermit = 0U;

    if (whiteCalibrated == 0U)
    {
        controller->state = LINE_FOLLOW_CTRL_UNCAL;
        return;
    }
    if (stableMask == 0U)
    {
        controller->state = LINE_FOLLOW_CTRL_LOST;
        return;
    }
    if (stableMask == LINE_FOLLOW_FULL_MASK)
    {
        controller->state = LINE_FOLLOW_CTRL_A_MARK;
        return;
    }
    if (activeCount > LINE_FOLLOW_MAX_NORMAL_ACTIVE_BITS)
    {
        controller->state = LINE_FOLLOW_CTRL_WIDE;
        return;
    }

    controller->state = LINE_FOLLOW_CTRL_TRACK;
    controller->baseForwardCommand = LINE_FOLLOW_INITIAL_FORWARD_CMD;
    demand = ((int32_t)errorX100 * controller->kpNumerator) /
             controller->kpDenominator;
    controller->steerDemand = LineFollowControl_Clamp(
        demand,
        (int16_t)(-controller->steerDemandLimit),
        controller->steerDemandLimit);
}

const char *LineFollowControl_StateName(const LineFollowController_t *controller)
{
    if (controller == 0)
    {
        return "NONE";
    }

    switch (controller->state)
    {
        case LINE_FOLLOW_CTRL_TRACK:  return "TRACK";
        case LINE_FOLLOW_CTRL_LOST:   return "LOST_HOLD";
        case LINE_FOLLOW_CTRL_A_MARK: return "A_MARK_HOLD";
        case LINE_FOLLOW_CTRL_WIDE:   return "WIDE_HOLD";
        default:                      return "UNCAL_HOLD";
    }
}

const char *LineFollowControl_SteerName(const LineFollowController_t *controller)
{
    if ((controller == 0) || (controller->state != LINE_FOLLOW_CTRL_TRACK))
    {
        return "HOLD";
    }
    if (controller->steerDemand < 0)
    {
        return "LEFT";
    }
    if (controller->steerDemand > 0)
    {
        return "RIGHT";
    }
    return "STRAIGHT";
}
