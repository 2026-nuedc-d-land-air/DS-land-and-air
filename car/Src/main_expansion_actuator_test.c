/**
 ******************************************************************************
 * @file    main_expansion_actuator_test.c
 * @brief   扩展 TB6612 与双 A4988 的安全手动触发测试入口。
 ******************************************************************************
 */

#include "app_expansion_actuator_test.h"
#include "bsp_a4988.h"
#include "bsp_aux_tb6612.h"
#include "bsp_diag_uart.h"

#define DIAG_BAUDRATE 115200U

static volatile uint32_t s_uptimeMs = 0U;

void SysTick_Handler(void)
{
    ++s_uptimeMs;
}

int main(void)
{
    char command;

    SystemCoreClockUpdate();
    AuxTb6612_Init();
    A4988_Init();
    DiagUart_Init(DIAG_BAUDRATE);

    if (SysTick_Config(SystemCoreClock / 1000U) != 0U)
    {
        DiagUart_WriteString("ACT,fault,systick_config\r\n");
        while (1)
        {
            AuxTb6612_StopAll();
            A4988_StopAll();
        }
    }

    ExpansionActuatorTest_Init(s_uptimeMs);
    while (1)
    {
        while (DiagUart_TryReadChar(&command) != 0U)
        {
            ExpansionActuatorTest_HandleCommand(command, s_uptimeMs);
        }
        ExpansionActuatorTest_Update(s_uptimeMs);
    }
}
