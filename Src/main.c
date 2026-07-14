/**
 ******************************************************************************
 * @file    main.c
 * @brief   Jetson Nano UART5 bidirectional link test with motors forced off.
 ******************************************************************************
 */

#include "app_nano_uart_test.h"
#include "app_bluetooth_motor_test.h"
#include "app_servo_test.h"
#include "bsp_diag_uart.h"
#include "bsp_motor_safe.h"
#include "bsp_robot_uart.h"
#include "nano_uart_diag_log.h"

#define DIAG_BAUDRATE 115200U
#define NANO_BAUDRATE 115200U
#define SERVO_BAUDRATE 115200U
#define BLUETOOTH_BAUDRATE 9600U

static volatile uint32_t s_uptimeMs = 0U;

void SysTick_Handler(void)
{
    ++s_uptimeMs;
}

int main(void)
{
    uint8_t nanoByte;

    uint8_t bluetoothByte;
    uint16_t nanoErrorFlags;

    SystemCoreClockUpdate();
    /* Keep every motor PWM/direction/STBY output low during UART testing. */
    MotorSafe_InitOff();

    DiagUart_Init(DIAG_BAUDRATE);
    RobotUart_NanoInit(NANO_BAUDRATE);
    RobotUart_BluetoothInit(BLUETOOTH_BAUDRATE);
    RobotUart_ServoInit(SERVO_BAUDRATE);
    ServoTest_StopAll();

    if (SysTick_Config(SystemCoreClock / 1000U) != 0U)
    {
        DiagUart_WriteString("FAULT,SYSTICK_CONFIG\r\n");
        RobotUart_NanoWriteString("FAULT,SYSTICK_CONFIG\r\n");
        while (1)
        {
            MotorSafe_ForceOff();
        }
    }

    DiagUart_WriteString("BOOT,nano_uart_test,nano_baud=");
    DiagUart_WriteUInt32(NANO_BAUDRATE);
    DiagUart_WriteString(",diag_baud=");
    DiagUart_WriteUInt32(DIAG_BAUDRATE);
    DiagUart_WriteString(",nano_pins=PC12_PD2,bluetooth_pins=PD8_PD9,diag_pins=PB6_PB7,motors_safe=1,servo_stop_sent=1\r\n");
    NanoUartDiagLog_Init(s_uptimeMs);
    NanoUartTest_Init(s_uptimeMs);
    BluetoothMotorTest_Init(s_uptimeMs);

    while (1)
    {
        while (RobotUart_NanoTryReadByte(&nanoByte) != 0U)
        {
            NanoUartTest_HandleByte(nanoByte, s_uptimeMs);
        }

        while (RobotUart_BluetoothTryReadByte(&bluetoothByte) != 0U)
        {
            BluetoothMotorTest_HandleByte('B', bluetoothByte, s_uptimeMs);
        }

        nanoErrorFlags = RobotUart_NanoConsumeErrorFlags();
        if (nanoErrorFlags != 0U)
        {
            NanoUartTest_HandleUartErrors(nanoErrorFlags, s_uptimeMs);
        }

        NanoUartDiagLog_PollCommand(s_uptimeMs);
        NanoUartTest_Update(s_uptimeMs);
        BluetoothMotorTest_Update(s_uptimeMs);
        MotorSafe_ForceOff();
    }
}
