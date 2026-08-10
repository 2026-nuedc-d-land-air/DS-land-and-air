/**
 ******************************************************************************
 * @file    main_lora_protocol_test.c
 * @brief   Safe V2.2 LoRa UART5 bench test; all motor outputs stay disabled.
 ******************************************************************************
 */

#include "app_lora_protocol_test.h"
#include "bsp_diag_uart.h"
#include "bsp_motor_safe.h"
#include "bsp_robot_uart.h"

#define DIAG_BAUDRATE 115200U
#ifndef LORA_UART_BAUDRATE
#define LORA_UART_BAUDRATE 115200U
#endif

static volatile uint32_t s_uptimeMs;

void SysTick_Handler(void)
{
    ++s_uptimeMs;
}

int main(void)
{
    uint8_t radioByte;
    uint16_t uartErrors;
    char command;

    SystemCoreClockUpdate();
    MotorSafe_InitOff();
    DiagUart_Init(DIAG_BAUDRATE);
    RobotUart_NanoInit(LORA_UART_BAUDRATE);

    if (SysTick_Config(SystemCoreClock / 1000U) != 0U)
    {
        DiagUart_WriteString("LORA,event=fault,reason=SYSTICK,motors_safe=1\r\n");
        while (1)
        {
            MotorSafe_ForceOff();
        }
    }

    LoraProtocolTest_Init(s_uptimeMs);
    while (1)
    {
        while (RobotUart_NanoTryReadByte(&radioByte) != 0U)
        {
            LoraProtocolTest_HandleRadioByte(radioByte, s_uptimeMs);
        }

        uartErrors = RobotUart_NanoConsumeErrorFlags();
        if (uartErrors != 0U)
        {
            LoraProtocolTest_HandleUartErrors(uartErrors, s_uptimeMs);
        }

        while (DiagUart_TryReadChar(&command) != 0U)
        {
            LoraProtocolTest_HandleCommand(command, s_uptimeMs);
        }

        LoraProtocolTest_Update(s_uptimeMs);
        MotorSafe_ForceOff();
    }
}
