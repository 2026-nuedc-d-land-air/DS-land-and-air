/**
 ******************************************************************************
 * @file    main_servo_uart_test.c
 * @brief   UART4 bus-servo pose-sequence qualification image.
 *
 * UART4: PC10=TX -> servo-controller RX, PC11=RX <- servo-controller TX.
 * The test is deliberately idle after reset.  Send T on the J-Link CDC
 * diagnostic port (COM13, 115200 8N1) to run the pose sequence.
 ******************************************************************************
 */

#include "app_servo_test.h"
#include "bsp_diag_uart.h"
#include "bsp_motor_safe.h"
#include "bsp_robot_uart.h"

#define DIAG_BAUDRATE                  115200U
#define SERVO_BAUDRATE                 115200U
#define SERVO_POSE_INTERVAL_MS         2500U

static volatile uint32_t s_uptimeMs = 0U;

static const uint8_t s_sequenceCommands[] =
{
    'I', /* INITIAL */
    'G', /* GRAB */
    'O', /* OPEN */
    'U', /* LIFT */
    'D', /* LOWER */
    'K'  /* STACK */
};

static uint8_t s_sequenceActive = 0U;
static uint8_t s_sequenceIndex = 0U;
static uint32_t s_nextPoseMs = 0U;

void SysTick_Handler(void)
{
    ++s_uptimeMs;
}

static uint8_t ServoUartTest_RestoreSysTick(void)
{
    /* ServoTest inserts 3 ms frame gaps with Delay_ms(), which temporarily
     * owns SysTick.  Restore the 1 ms scheduler immediately after a pose. */
    return (SysTick_Config(SystemCoreClock / 1000U) == 0U) ? 1U : 0U;
}

static void ServoUartTest_PrintHelp(void)
{
    DiagUart_WriteString(
        "SERVO_DIAG T=sequence I/G/O/U/D/K=one_pose S=stop H=help\r\n");
}

static void ServoUartTest_StartSequence(uint32_t nowMs)
{
    s_sequenceActive = 1U;
    s_sequenceIndex = 0U;
    s_nextPoseMs = nowMs;
    DiagUart_WriteString("SERVO_SEQUENCE,BEGIN,poses=INITIAL_GRAB_OPEN_LIFT_LOWER_STACK,interval_ms=");
    DiagUart_WriteUInt32(SERVO_POSE_INTERVAL_MS);
    DiagUart_WriteString("\r\n");
}

static void ServoUartTest_StopSequence(void)
{
    s_sequenceActive = 0U;
    ServoTest_StopAll();
    (void)ServoUartTest_RestoreSysTick();
    DiagUart_WriteString("SERVO_SEQUENCE,STOP\r\n");
}

static void ServoUartTest_RunPose(uint8_t command, uint32_t nowMs)
{
    DiagUart_WriteString("SERVO_SEQUENCE,POSE,");
    DiagUart_WriteChar((char)command);
    DiagUart_WriteString(",at_ms=");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString("\r\n");

    ServoTest_HandleCommand('D', command);
    if (ServoUartTest_RestoreSysTick() == 0U)
    {
        MotorSafe_ForceOff();
        while (1)
        {
        }
    }
}

static void ServoUartTest_PollDiagCommand(uint32_t nowMs)
{
    char command;

    while (DiagUart_TryReadChar(&command) != 0U)
    {
        if ((command == '\r') || (command == '\n') || (command == ' '))
        {
            continue;
        }

        if ((command >= 'a') && (command <= 'z'))
        {
            command = (char)(command - 'a' + 'A');
        }

        if (command == 'T')
        {
            ServoUartTest_StartSequence(nowMs);
        }
        else if (command == 'S')
        {
            ServoUartTest_StopSequence();
        }
        else if ((command == 'I') || (command == 'G') ||
                 (command == 'O') || (command == 'U') ||
                 (command == 'D') || (command == 'K') ||
                 (command == 'Q'))
        {
            s_sequenceActive = 0U;
            ServoUartTest_RunPose((uint8_t)command, nowMs);
        }
        else
        {
            ServoUartTest_PrintHelp();
        }
    }
}

static void ServoUartTest_Update(uint32_t nowMs)
{
    if ((s_sequenceActive == 0U) ||
        ((int32_t)(nowMs - s_nextPoseMs) < 0))
    {
        return;
    }

    if (s_sequenceIndex >= (uint8_t)sizeof(s_sequenceCommands))
    {
        s_sequenceActive = 0U;
        DiagUart_WriteString("SERVO_SEQUENCE,END\r\n");
        return;
    }

    ServoUartTest_RunPose(s_sequenceCommands[s_sequenceIndex], nowMs);
    ++s_sequenceIndex;
    s_nextPoseMs = nowMs + SERVO_POSE_INTERVAL_MS;
}

int main(void)
{
    MotorSafe_InitOff();
    SystemCoreClockUpdate();
    DiagUart_Init(DIAG_BAUDRATE);
    RobotUart_ServoInit(SERVO_BAUDRATE);

    if (SysTick_Config(SystemCoreClock / 1000U) != 0U)
    {
        MotorSafe_ForceOff();
        while (1)
        {
        }
    }

    ServoTest_Init();
    DiagUart_WriteString("BOOT,servo_uart_test,servo_uart=UART4,tx=PC10,rx=PC11,baud=115200,motors_safe=1\r\n");
    ServoUartTest_PrintHelp();

    while (1)
    {
        ServoUartTest_PollDiagCommand(s_uptimeMs);
        ServoUartTest_Update(s_uptimeMs);
        ServoTest_PollServoResponse();
        MotorSafe_ForceOff();
    }
}
