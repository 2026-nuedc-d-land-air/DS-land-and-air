#ifndef __APP_WHEEL_TEST_H
#define __APP_WHEEL_TEST_H

#include "stm32f10x.h"

typedef enum
{
    WHEEL_TEST_STAGE_IDLE = 0,
    WHEEL_TEST_STAGE_LEFT,
    WHEEL_TEST_STAGE_RIGHT,
    WHEEL_TEST_STAGE_BOTH,
    WHEEL_TEST_STAGE_PAUSE
} WheelTest_Stage_t;

typedef struct
{
    uint32_t runId;
    WheelTest_Stage_t stage;
    uint32_t elapsedMs;
    int16_t deltaLeft;
    int16_t deltaRight;
    int32_t totalLeft;
    int32_t totalRight;
    uint8_t detectedLeft;
    uint8_t detectedRight;
    uint8_t stagePassed;
} WheelTest_Telemetry_t;

extern volatile WheelTest_Telemetry_t g_wheelTestTelemetry;

void WheelTest_Init(void);
void WheelTest_RunCycle(void);

#endif /* __APP_WHEEL_TEST_H */
