#include "motor_tb6612_test.h"

#include "bsp_diag_uart.h"
#include "bsp_l298n.h"
#include "bsp_motor.h"
#include "Delay.h"
#include "motor_diag_log.h"

#define TB6612_TEST_SPEED       100
#define TB6612_TEST_RUN_MS      2000U
#define TB6612_TEST_PAUSE_MS    1000U
#define TB6612_TEST_WHEEL_MS    2000U

static uint32_t s_testTimeMs = 0U;

static void MotorTb6612Test_Pause(uint32_t durationMs)
{
    Delay_ms(durationMs);
    s_testTimeMs += durationMs;
}

static void MotorTb6612Test_Stop(void)
{
    Motor_Coast(MOTOR_LEFT);
    Motor_Coast(MOTOR_RIGHT);
}

static void MotorTb6612Test_RunStage(Motor_Index_t motor,
                                     L298N_Wheel_t wheel,
                                     int16_t command,
                                     uint32_t pauseMs)
{
    MotorTb6612Test_Stop();
    Motor_SetSpeed(motor, command);
    MotorDiagLog_Record(s_testTimeMs,
                        MOTOR_DIAG_EVENT_STAGE_BEGIN,
                        wheel, command);

    MotorTb6612Test_Pause(TB6612_TEST_RUN_MS);
    MotorTb6612Test_Stop();
    MotorDiagLog_Record(s_testTimeMs,
                        MOTOR_DIAG_EVENT_STAGE_END,
                        wheel, 0);
    MotorTb6612Test_Pause(pauseMs);
}

void MotorTb6612Test_Run(void)
{
    s_testTimeMs = 0U;
    L298N_StopAll();
    Motor_Enable(1U);
    MotorTb6612Test_Stop();
    MotorDiagLog_Record(0U, MOTOR_DIAG_EVENT_STOP,
                        (L298N_Wheel_t)255U, 0);

    DiagUart_WriteString("tb6612_test.begin expected=LEFT_POS,LEFT_NEG,RIGHT_POS,RIGHT_NEG rear=OFF\r\n");
    MotorTb6612Test_Pause(TB6612_TEST_PAUSE_MS);

    MotorTb6612Test_RunStage(MOTOR_LEFT, L298N_WHEEL_LF,
                             TB6612_TEST_SPEED, TB6612_TEST_PAUSE_MS);
    MotorTb6612Test_RunStage(MOTOR_LEFT, L298N_WHEEL_LF,
                             -TB6612_TEST_SPEED, TB6612_TEST_WHEEL_MS);
    MotorTb6612Test_RunStage(MOTOR_RIGHT, L298N_WHEEL_RF,
                             TB6612_TEST_SPEED, TB6612_TEST_PAUSE_MS);
    MotorTb6612Test_RunStage(MOTOR_RIGHT, L298N_WHEEL_RF,
                             -TB6612_TEST_SPEED, TB6612_TEST_WHEEL_MS);

    MotorTb6612Test_Stop();
    Motor_Enable(0U);
    MotorDiagLog_Record(s_testTimeMs, MOTOR_DIAG_EVENT_STOP,
                        (L298N_Wheel_t)255U, 0);
    MotorDiagLog_Freeze();
    DiagUart_WriteString("tb6612_test.end motors=OFF log=FROZEN send=E\r\n");
}
