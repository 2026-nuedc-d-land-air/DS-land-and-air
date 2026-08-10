#ifndef __BSP_L298N_H
#define __BSP_L298N_H

#include "stm32f10x.h"

typedef enum
{
    L298N_WHEEL_LF = 0,
    L298N_WHEEL_RF,
    L298N_WHEEL_LR,
    L298N_WHEEL_RR
} L298N_Wheel_t;

typedef enum
{
    L298N_STAGE_BOOT_PAUSE = 0,
    L298N_STAGE_LF_FORWARD,
    L298N_STAGE_LF_REVERSE,
    L298N_STAGE_RF_FORWARD,
    L298N_STAGE_RF_REVERSE,
    L298N_STAGE_LR_FORWARD,
    L298N_STAGE_LR_REVERSE,
    L298N_STAGE_RR_FORWARD,
    L298N_STAGE_RR_REVERSE,
    L298N_STAGE_WHEEL_PAUSE
} L298N_TestStage_t;

extern volatile L298N_TestStage_t g_l298nTestStage;
extern volatile uint32_t g_l298nTestCycle;

void L298N_Init(void);
void L298N_SetWheel(L298N_Wheel_t wheel, int16_t speedPercent);
void L298N_StopAll(void);
void L298N_SetFrontEnableStatic(uint8_t leftEnable, uint8_t rightEnable);
void L298N_RunDirectionTestLoop(void);

#endif /* __BSP_L298N_H */
