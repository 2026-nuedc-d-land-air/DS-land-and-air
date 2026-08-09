#include "app_ladrc.h"

#define LADRC_OUTPUT_LIMIT_PERCENT 30

static int16_t Ladrc_ClampOutput(int32_t output)
{
    if (output > LADRC_OUTPUT_LIMIT_PERCENT)
    {
        output = LADRC_OUTPUT_LIMIT_PERCENT;
    }
    else if (output < -LADRC_OUTPUT_LIMIT_PERCENT)
    {
        output = -LADRC_OUTPUT_LIMIT_PERCENT;
    }
    return (int16_t)output;
}

void Ladrc_Init(LadrcController_t *controller,
                int16_t kpNumerator,
                int16_t kpDenominator)
{
    if (controller == 0)
    {
        return;
    }

    controller->targetTenthsDeg = 0;
    controller->measuredTenthsDeg = 0;
    controller->estimateTenthsDeg = 0;
    controller->errorTenthsDeg = 0;
    controller->outputPercent = 0;
    controller->kpNumerator = kpNumerator;
    controller->kpDenominator = (kpDenominator == 0) ? 1 : kpDenominator;
    controller->enabled = 1U;
}

void Ladrc_Reset(LadrcController_t *controller,
                 int16_t targetTenthsDeg,
                 int16_t measuredTenthsDeg)
{
    if (controller == 0)
    {
        return;
    }

    controller->targetTenthsDeg = targetTenthsDeg;
    controller->measuredTenthsDeg = measuredTenthsDeg;
    controller->estimateTenthsDeg = measuredTenthsDeg;
    controller->errorTenthsDeg = (int16_t)(targetTenthsDeg - measuredTenthsDeg);
    controller->outputPercent = 0;
}

void Ladrc_SetEnabled(LadrcController_t *controller, uint8_t enabled)
{
    if (controller == 0)
    {
        return;
    }

    controller->enabled = (enabled != 0U) ? 1U : 0U;
    if (controller->enabled == 0U)
    {
        controller->outputPercent = 0;
    }
}

int16_t Ladrc_Update(LadrcController_t *controller,
                     int16_t targetTenthsDeg,
                     int16_t measuredTenthsDeg)
{
    int32_t output;

    if (controller == 0)
    {
        return 0;
    }

    controller->targetTenthsDeg = targetTenthsDeg;
    controller->measuredTenthsDeg = measuredTenthsDeg;
    controller->estimateTenthsDeg = measuredTenthsDeg;
    controller->errorTenthsDeg = (int16_t)(targetTenthsDeg - measuredTenthsDeg);

    if (controller->enabled == 0U)
    {
        controller->outputPercent = 0;
        return 0;
    }

    output = ((int32_t)controller->errorTenthsDeg *
              controller->kpNumerator) / controller->kpDenominator;
    controller->outputPercent = Ladrc_ClampOutput(output);
    return controller->outputPercent;
}

int16_t Ladrc_GetOutputPercent(const LadrcController_t *controller)
{
    if (controller == 0)
    {
        return 0;
    }
    return controller->outputPercent;
}
