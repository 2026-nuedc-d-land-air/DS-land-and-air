#include "app_wheel_test.h"

#include "Delay.h"
#include "bsp_diag_uart.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"

#define WHEEL_TEST_PWM_PERCENT       35
#define WHEEL_TEST_SAMPLE_MS         100U
#define WHEEL_TEST_RUN_MS            2000U
#define WHEEL_TEST_SETTLE_MS         200U
#define WHEEL_TEST_STAGE_PAUSE_MS    800U
#define WHEEL_TEST_CYCLE_PAUSE_MS    1500U
#define WHEEL_TEST_MIN_COUNTS        20

volatile WheelTest_Telemetry_t g_wheelTestTelemetry = {0};

static int32_t WheelTest_Abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static const char *WheelTest_StageName(WheelTest_Stage_t stage)
{
    switch (stage)
    {
        case WHEEL_TEST_STAGE_LEFT:  return "LEFT";
        case WHEEL_TEST_STAGE_RIGHT: return "RIGHT";
        case WHEEL_TEST_STAGE_BOTH:  return "BOTH";
        case WHEEL_TEST_STAGE_PAUSE: return "PAUSE";
        default:                     return "IDLE";
    }
}

static void WheelTest_UpdateTelemetry(uint32_t elapsedMs)
{
    g_wheelTestTelemetry.elapsedMs = elapsedMs;
    g_wheelTestTelemetry.deltaLeft = g_encoderL.count;
    g_wheelTestTelemetry.deltaRight = g_encoderR.count;
    g_wheelTestTelemetry.totalLeft = g_encoderL.totalCount;
    g_wheelTestTelemetry.totalRight = g_encoderR.totalCount;
    g_wheelTestTelemetry.detectedLeft =
        (WheelTest_Abs32(g_encoderL.totalCount) >= WHEEL_TEST_MIN_COUNTS) ? 1U : 0U;
    g_wheelTestTelemetry.detectedRight =
        (WheelTest_Abs32(g_encoderR.totalCount) >= WHEEL_TEST_MIN_COUNTS) ? 1U : 0U;
}

static void WheelTest_LogSample(void)
{
    DiagUart_WriteString("WHEEL,");
    DiagUart_WriteUInt32(g_wheelTestTelemetry.runId);
    DiagUart_WriteChar(',');
    DiagUart_WriteString(WheelTest_StageName(g_wheelTestTelemetry.stage));
    DiagUart_WriteChar(',');
    DiagUart_WriteUInt32(g_wheelTestTelemetry.elapsedMs);
    DiagUart_WriteChar(',');
    DiagUart_WriteInt32(g_wheelTestTelemetry.deltaLeft);
    DiagUart_WriteChar(',');
    DiagUart_WriteInt32(g_wheelTestTelemetry.deltaRight);
    DiagUart_WriteChar(',');
    DiagUart_WriteInt32(g_wheelTestTelemetry.totalLeft);
    DiagUart_WriteChar(',');
    DiagUart_WriteInt32(g_wheelTestTelemetry.totalRight);
    DiagUart_WriteString("\r\n");
}

static void WheelTest_LogResult(void)
{
    DiagUart_WriteString("RESULT,");
    DiagUart_WriteUInt32(g_wheelTestTelemetry.runId);
    DiagUart_WriteChar(',');
    DiagUart_WriteString(WheelTest_StageName(g_wheelTestTelemetry.stage));
    DiagUart_WriteChar(',');
    DiagUart_WriteUInt32(g_wheelTestTelemetry.detectedLeft);
    DiagUart_WriteChar(',');
    DiagUart_WriteUInt32(g_wheelTestTelemetry.detectedRight);
    DiagUart_WriteChar(',');
    DiagUart_WriteUInt32(g_wheelTestTelemetry.stagePassed);
    DiagUart_WriteString("\r\n");
}

static void WheelTest_RunStage(WheelTest_Stage_t stage,
                               int16_t speedLeft,
                               int16_t speedRight,
                               uint8_t expectLeft,
                               uint8_t expectRight)
{
    uint32_t elapsedMs;

    g_wheelTestTelemetry.stage = stage;
    g_wheelTestTelemetry.stagePassed = 0U;
    Encoder_Reset();
    WheelTest_UpdateTelemetry(0U);

    Motor_SetSpeedBoth(speedLeft, speedRight);

    for (elapsedMs = WHEEL_TEST_SAMPLE_MS;
         elapsedMs <= WHEEL_TEST_RUN_MS;
         elapsedMs += WHEEL_TEST_SAMPLE_MS)
    {
        Delay_ms(WHEEL_TEST_SAMPLE_MS);
        Encoder_Update(WHEEL_TEST_SAMPLE_MS);
        WheelTest_UpdateTelemetry(elapsedMs);
        WheelTest_LogSample();
    }

    Motor_SetSpeedBoth(0, 0);
    Delay_ms(WHEEL_TEST_SETTLE_MS);
    Encoder_Update(WHEEL_TEST_SETTLE_MS);
    WheelTest_UpdateTelemetry(WHEEL_TEST_RUN_MS + WHEEL_TEST_SETTLE_MS);

    g_wheelTestTelemetry.stagePassed =
        ((!expectLeft || g_wheelTestTelemetry.detectedLeft) &&
         (!expectRight || g_wheelTestTelemetry.detectedRight)) ? 1U : 0U;
    WheelTest_LogResult();

    g_wheelTestTelemetry.stage = WHEEL_TEST_STAGE_PAUSE;
    Delay_ms(WHEEL_TEST_STAGE_PAUSE_MS);
}

void WheelTest_Init(void)
{
    DiagUart_Init(115200U);
    Motor_Init();
    Encoder_Init();
    Motor_Enable(1U);

    DiagUart_WriteString("run_id,stage,time_ms,enc_l_delta,enc_r_delta,enc_l_total,enc_r_total\r\n");
    DiagUart_WriteString("Wheel test: TIM5=left encoder, TIM3=right encoder, TIM2_CH3/4=PWM\r\n");
}

void WheelTest_RunCycle(void)
{
    g_wheelTestTelemetry.runId++;

    WheelTest_RunStage(WHEEL_TEST_STAGE_LEFT,
                       WHEEL_TEST_PWM_PERCENT, 0, 1U, 0U);
    WheelTest_RunStage(WHEEL_TEST_STAGE_RIGHT,
                       0, WHEEL_TEST_PWM_PERCENT, 0U, 1U);
    WheelTest_RunStage(WHEEL_TEST_STAGE_BOTH,
                       WHEEL_TEST_PWM_PERCENT, WHEEL_TEST_PWM_PERCENT, 1U, 1U);

    Motor_SetSpeedBoth(0, 0);
    g_wheelTestTelemetry.stage = WHEEL_TEST_STAGE_PAUSE;
    Delay_ms(WHEEL_TEST_CYCLE_PAUSE_MS);
}
