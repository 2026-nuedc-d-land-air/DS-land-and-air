/**
 ******************************************************************************
 * @file    main_four_wheel_tb6612_test.c
 * @brief   Safe isolated four-wheel, two-TB6612 qualification entry.
 ******************************************************************************
 */

#include "app_four_wheel_tb6612_test.h"
#include "bsp_aux_tb6612.h"
#include "bsp_diag_uart.h"
#include "bsp_motor.h"

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
    Motor_Init();
    Motor_Enable(0U);
    AuxTb6612_Init();
    DiagUart_Init(DIAG_BAUDRATE);

    if (SysTick_Config(SystemCoreClock / 1000U) != 0U)
    {
        while (1)
        {
            AuxTb6612_StopAll();
            Motor_Coast(MOTOR_LEFT);
            Motor_Coast(MOTOR_RIGHT);
            Motor_Enable(0U);
        }
    }

    FourWheelTb6612Test_Init(s_uptimeMs);
    while (1)
    {
        while (DiagUart_TryReadChar(&command) != 0U)
        {
            FourWheelTb6612Test_HandleCommand(command, s_uptimeMs);
        }
        FourWheelTb6612Test_Update(s_uptimeMs);
    }
}
