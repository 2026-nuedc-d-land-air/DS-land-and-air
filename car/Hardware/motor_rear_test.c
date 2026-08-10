#include "motor_rear_test.h"

#include "bsp_diag_uart.h"
#include "bsp_l298n.h"
#include "Delay.h"
#include "motor_diag_log.h"

#define REAR_TEST_SPEED       60
#define REAR_TEST_RUN_MS      2000U
#define REAR_TEST_SHORT_MS    1000U

static uint32_t s_rearTestTimeMs = 0U;

static void MotorRearTest_Pause(uint32_t durationMs)
{
    Delay_ms(durationMs);
    s_rearTestTimeMs += durationMs;
}

static void MotorRearTest_RunStage(L298N_TestStage_t stage,
                                   L298N_Wheel_t wheel,
                                   int16_t command,
                                   uint32_t pauseMs)
{
    L298N_StopAll();
    g_l298nTestStage = stage;
    L298N_SetWheel(wheel, command);
    MotorDiagLog_Record(s_rearTestTimeMs,
                        MOTOR_DIAG_EVENT_STAGE_BEGIN,
                        wheel, command);

    MotorRearTest_Pause(REAR_TEST_RUN_MS);
    L298N_StopAll();
    MotorDiagLog_Record(s_rearTestTimeMs,
                        MOTOR_DIAG_EVENT_STAGE_END,
                        wheel, 0);
    MotorRearTest_Pause(pauseMs);
}

void MotorRearTest_Run(void)
{
    s_rearTestTimeMs = 0U;
    L298N_StopAll();
    MotorDiagLog_Record(s_rearTestTimeMs, MOTOR_DIAG_EVENT_STOP,
                        (L298N_Wheel_t)255U, 0);

    DiagUart_WriteString("rear_test.begin expected=LR_NEG,LR_POS,RR_NEG,RR_POS front=OFF\r\n");
    MotorRearTest_Pause(REAR_TEST_SHORT_MS);

    MotorRearTest_RunStage(L298N_STAGE_LR_REVERSE, L298N_WHEEL_LR,
                           -REAR_TEST_SPEED, REAR_TEST_SHORT_MS);
    MotorRearTest_RunStage(L298N_STAGE_LR_FORWARD, L298N_WHEEL_LR,
                           REAR_TEST_SPEED, REAR_TEST_SHORT_MS);
    MotorRearTest_RunStage(L298N_STAGE_RR_REVERSE, L298N_WHEEL_RR,
                           -REAR_TEST_SPEED, REAR_TEST_SHORT_MS);
    MotorRearTest_RunStage(L298N_STAGE_RR_FORWARD, L298N_WHEEL_RR,
                           REAR_TEST_SPEED, REAR_TEST_SHORT_MS);

    L298N_StopAll();
    MotorDiagLog_Record(s_rearTestTimeMs, MOTOR_DIAG_EVENT_STOP,
                        (L298N_Wheel_t)255U, 0);
    MotorDiagLog_Freeze();
    DiagUart_WriteString("rear_test.end motors=OFF log=FROZEN send=E\r\n");
}
