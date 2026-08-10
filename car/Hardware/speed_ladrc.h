/**
 ******************************************************************************
 * @file    speed_ladrc.h
 * @brief   Single-wheel linear ADRC speed controller.
 *
 * The state observer is deliberately local to a wheel.  It estimates wheel
 * speed and a lumped disturbance; it is not the vehicle line observer.
 ******************************************************************************
 */

#ifndef __SPEED_LADRC_H
#define __SPEED_LADRC_H

#include "stm32f10x.h"

typedef struct
{
    float z1;
    float z2;
    float b0;
    float wc;
    float wo;
    float outputPercent;
    float outputMinPercent;
    float outputMaxPercent;
    float outputStepLimitPercent;
} SpeedLadrc_t;

void SpeedLadrc_Init(SpeedLadrc_t *controller,
                     float b0,
                     float wc,
                     float wo,
                     float outputMinPercent,
                     float outputMaxPercent,
                     float outputStepLimitPercent);
void SpeedLadrc_Reset(SpeedLadrc_t *controller, float measuredCountsPerSecond);
float SpeedLadrc_Update(SpeedLadrc_t *controller,
                        float targetCountsPerSecond,
                        float measuredCountsPerSecond,
                        float samplePeriodSeconds);

#endif /* __SPEED_LADRC_H */
