/**
 ******************************************************************************
 * @file    main_line_follow_mission.c
 * @brief   Competition line-follow field-test entry point.
 ******************************************************************************
 */

#include "app_line_follow_mission.h"
#include "bsp_diag_uart.h"
#include "bsp_motor_safe.h"

#define DIAG_BAUDRATE 115200U

static volatile uint32_t s_uptimeMs;

void SysTick_Handler(void)
{
    ++s_uptimeMs;
}

int main(void)
{
    char command;

    SystemCoreClockUpdate();
    MotorSafe_InitOff();
    DiagUart_Init(DIAG_BAUDRATE);

    if (SysTick_Config(SystemCoreClock / 1000U) != 0U)
    {
        DiagUart_WriteString("LF,event=fault,reason=SYSTICK,motors_safe=1\r\n");
        while (1)
        {
            MotorSafe_ForceOff();
        }
    }

    LineFollowMission_Init(s_uptimeMs);
    while (1)
    {
        while (DiagUart_TryReadChar(&command) != 0U)
        {
            LineFollowMission_HandleCommand(command, s_uptimeMs);
        }
        LineFollowMission_Update(s_uptimeMs);
    }
}
