#include "app_gray_tracking_test.h"
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
        while (1)
        {
            MotorSafe_ForceOff();
        }
    }

    GrayTrackingTest_Init(s_uptimeMs);
    while (1)
    {
        while (DiagUart_TryReadChar(&command) != 0U)
        {
            GrayTrackingTest_HandleCommand(command, s_uptimeMs);
        }
        GrayTrackingTest_Update(s_uptimeMs);
    }
}
