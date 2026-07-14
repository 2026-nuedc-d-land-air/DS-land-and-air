/**
 ******************************************************************************
 * @file    main_bluetooth_mecanum_test.c
 * @brief   Bluetooth button telemetry and four-wheel direction preview.
 ******************************************************************************
 */

#include "app_bluetooth_motor_test.h"
#include "bsp_diag_uart.h"
#include "bsp_mecanum.h"
#include "bsp_motor_safe.h"
#include "bsp_robot_uart.h"

#ifndef BLUETOOTH_BAUDRATE
#define BLUETOOTH_BAUDRATE 9600U
#endif

#define DIAG_BAUDRATE 115200U

static volatile uint32_t s_uptimeMs = 0U;

void SysTick_Handler(void)
{
    ++s_uptimeMs;
}

int main(void)
{
    uint8_t bluetoothByte;

    /* Clamp every motor pin before any UART or timer initialization. */
    MotorSafe_InitOff();
    SystemCoreClockUpdate();
    Mecanum_InitOff();
    DiagUart_Init(DIAG_BAUDRATE);
    RobotUart_BluetoothInit(BLUETOOTH_BAUDRATE);

    if (SysTick_Config(SystemCoreClock / 1000U) != 0U)
    {
        while (1)
        {
            Mecanum_Enable(0U);
        }
    }

    DiagUart_WriteString("BOOT,bluetooth_mecanum_test,baud=");
    DiagUart_WriteUInt32(BLUETOOTH_BAUDRATE);
    DiagUart_WriteString(",wheel_order=LF_RF_LR_RR\r\n");
    BluetoothMotorTest_Init(s_uptimeMs);

    /* Discard any bytes produced while the Bluetooth module powers up. */
    while (RobotUart_BluetoothTryReadByte(&bluetoothByte) != 0U)
    {
    }

    while (1)
    {
        while (RobotUart_BluetoothTryReadByte(&bluetoothByte) != 0U)
        {
            BluetoothMotorTest_HandleByte('B', bluetoothByte, s_uptimeMs);
        }

        BluetoothMotorTest_Update(s_uptimeMs);
        Mecanum_Update(s_uptimeMs);
    }
}
