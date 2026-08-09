/**
 ******************************************************************************
 * @file    speed_ladrc.c
 * @brief   Single-wheel linear ADRC speed controller implementation.
 ******************************************************************************
 */

#include "speed_ladrc.h"

static float SpeedLadrc_Clamp(float value, float lower, float upper)
{
    if (value > upper)
    {
        return upper;
    }
    if (value < lower)
    {
        return lower;
    }
    return value;
}

void SpeedLadrc_Init(SpeedLadrc_t *controller,
                     float b0,
                     float wc,
                     float wo,
                     float outputMinPercent,
                     float outputMaxPercent,
                     float outputStepLimitPercent)
{
    if (controller == 0)
    {
        return;
    }

    controller->z1 = 0.0f;
    controller->z2 = 0.0f;
    controller->b0 = (b0 > 1.0f) ? b0 : 1.0f;
    controller->wc = (wc > 0.1f) ? wc : 0.1f;
    controller->wo = (wo > 0.1f) ? wo : 0.1f;
    controller->outputPercent = 0.0f;
    controller->outputMinPercent = outputMinPercent;
    controller->outputMaxPercent = outputMaxPercent;
    controller->outputStepLimitPercent = (outputStepLimitPercent > 0.0f) ?
                                         outputStepLimitPercent : outputMaxPercent;
}

void SpeedLadrc_Reset(SpeedLadrc_t *controller, float measuredCountsPerSecond)
{
    if (controller == 0)
    {
        return;
    }

    controller->z1 = measuredCountsPerSecond;
    controller->z2 = 0.0f;
    controller->outputPercent = 0.0f;
}

float SpeedLadrc_Update(SpeedLadrc_t *controller,
                        float targetCountsPerSecond,
                        float measuredCountsPerSecond,
                        float samplePeriodSeconds)
{
    float beta1;
    float beta2;
    float observerError;
    float virtualControl;
    float requestedOutput;
    float lowerRateLimit;
    float upperRateLimit;

    if (controller == 0)
    {
        return 0.0f;
    }

    samplePeriodSeconds = SpeedLadrc_Clamp(samplePeriodSeconds, 0.005f, 0.050f);
    beta1 = 2.0f * controller->wo;
    beta2 = controller->wo * controller->wo;

    /* z1=wheel speed; z2=lumped motor/load disturbance. */
    observerError = controller->z1 - measuredCountsPerSecond;
    controller->z1 += samplePeriodSeconds *
                      (controller->z2 - beta1 * observerError +
                       controller->b0 * controller->outputPercent);
    controller->z2 += samplePeriodSeconds * (-beta2 * observerError);

    virtualControl = controller->wc *
                     (targetCountsPerSecond - controller->z1);
    requestedOutput = (virtualControl - controller->z2) / controller->b0;
    requestedOutput = SpeedLadrc_Clamp(requestedOutput,
                                        controller->outputMinPercent,
                                        controller->outputMaxPercent);

    lowerRateLimit = controller->outputPercent - controller->outputStepLimitPercent;
    upperRateLimit = controller->outputPercent + controller->outputStepLimitPercent;
    controller->outputPercent = SpeedLadrc_Clamp(requestedOutput,
                                                  lowerRateLimit,
                                                  upperRateLimit);
    controller->outputPercent = SpeedLadrc_Clamp(controller->outputPercent,
                                                  controller->outputMinPercent,
                                                  controller->outputMaxPercent);
    return controller->outputPercent;
}
