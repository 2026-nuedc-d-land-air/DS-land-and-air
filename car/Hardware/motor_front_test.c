#include "motor_front_test.h"

#include "bsp_diag_uart.h"
#include "bsp_l298n.h"
#include "Delay.h"
#include "motor_diag_log.h"

#define FRONT_TEST_SPEED       60
#define FRONT_TEST_RUN_MS      1000U
#define FRONT_TEST_SHORT_MS    1000U
#define FRONT_TEST_WHEEL_MS    2000U

static uint32_t s_frontTestTimeMs = 0U;

static void MotorFrontTest_Pause(uint32_t durationMs)
{
    Delay_ms(durationMs);
    s_frontTestTimeMs += durationMs;
}

static void MotorFrontTest_RunStage(L298N_TestStage_t stage,
                                    L298N_Wheel_t wheel,
                                    int16_t command,
                                    uint32_t pauseMs)
{
    L298N_StopAll();
    L298N_SetFrontEnableStatic((wheel == L298N_WHEEL_LF) ? 1U : 0U,
                               (wheel == L298N_WHEEL_RF) ? 1U : 0U);
    g_l298nTestStage = stage;
    L298N_SetWheel(wheel, command);
    MotorDiagLog_Record(s_frontTestTimeMs,
                        MOTOR_DIAG_EVENT_STAGE_BEGIN,
                        wheel, command);

    MotorFrontTest_Pause(FRONT_TEST_RUN_MS);
    L298N_StopAll();
    L298N_SetFrontEnableStatic(0U, 0U);
    MotorDiagLog_Record(s_frontTestTimeMs,
                        MOTOR_DIAG_EVENT_STAGE_END,
                        wheel, 0);
    MotorFrontTest_Pause(pauseMs);
}

void MotorFrontTest_Run(void)
{
    s_frontTestTimeMs = 0U;
    L298N_StopAll();
    L298N_SetFrontEnableStatic(0U, 0U);
    MotorDiagLog_Record(s_frontTestTimeMs, MOTOR_DIAG_EVENT_STOP,
                        (L298N_Wheel_t)255U, 0);

    DiagUart_WriteString("front_test.begin expected=LF_POS,LF_NEG,RF_POS,RF_NEG rear=OFF\r\n");
    MotorFrontTest_Pause(FRONT_TEST_SHORT_MS);

    MotorFrontTest_RunStage(L298N_STAGE_LF_FORWARD, L298N_WHEEL_LF,
                            FRONT_TEST_SPEED, FRONT_TEST_SHORT_MS);
    MotorFrontTest_RunStage(L298N_STAGE_LF_REVERSE, L298N_WHEEL_LF,
                            -FRONT_TEST_SPEED, FRONT_TEST_WHEEL_MS);
    MotorFrontTest_RunStage(L298N_STAGE_RF_FORWARD, L298N_WHEEL_RF,
                            FRONT_TEST_SPEED, FRONT_TEST_SHORT_MS);
    MotorFrontTest_RunStage(L298N_STAGE_RF_REVERSE, L298N_WHEEL_RF,
                            -FRONT_TEST_SPEED, FRONT_TEST_WHEEL_MS);

    L298N_StopAll();
    L298N_SetFrontEnableStatic(0U, 0U);
    MotorDiagLog_Record(s_frontTestTimeMs, MOTOR_DIAG_EVENT_STOP,
                        (L298N_Wheel_t)255U, 0);
    MotorDiagLog_Freeze();
    DiagUart_WriteString("front_test.end motors=OFF log=FROZEN send=E\r\n");
}
