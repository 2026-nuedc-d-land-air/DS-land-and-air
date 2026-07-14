#ifndef __BSP_MECANUM_H
#define __BSP_MECANUM_H

#include "stm32f10x.h"

typedef enum
{
    MECANUM_WHEEL_LF = 0,
    MECANUM_WHEEL_RF,
    MECANUM_WHEEL_LR,
    MECANUM_WHEEL_RR
} MecanumWheel_t;

void Mecanum_InitOff(void);
void Mecanum_Enable(uint8_t enable);
void Mecanum_SetWheel(MecanumWheel_t wheel, int16_t speedPercent);
void Mecanum_StopAll(void);
void Mecanum_Update(uint32_t nowMs);

#endif /* __BSP_MECANUM_H */
