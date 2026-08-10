#ifndef __APP_LADRC_H
#define __APP_LADRC_H

#include "stm32f10x.h"

typedef struct
{
    int16_t targetTenthsDeg;
    int16_t measuredTenthsDeg;
    int16_t estimateTenthsDeg;
    int16_t errorTenthsDeg;
    int16_t outputPercent;
    int16_t kpNumerator;
    int16_t kpDenominator;
    uint8_t enabled;
} LadrcController_t;

void Ladrc_Init(LadrcController_t *controller,
                int16_t kpNumerator,
                int16_t kpDenominator);
void Ladrc_Reset(LadrcController_t *controller,
                 int16_t targetTenthsDeg,
                 int16_t measuredTenthsDeg);
void Ladrc_SetEnabled(LadrcController_t *controller, uint8_t enabled);
int16_t Ladrc_Update(LadrcController_t *controller,
                     int16_t targetTenthsDeg,
                     int16_t measuredTenthsDeg);
int16_t Ladrc_GetOutputPercent(const LadrcController_t *controller);

#endif /* __APP_LADRC_H */
