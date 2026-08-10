/**
 ******************************************************************************
 * @file    main_front_tb6612_direct_test.c
 * @brief   Isolated front TB6612 direction check based on bcd5521.
 *
 * The test drives PA2/PA3 as GPIO high levels instead of TIM2 PWM.  This
 * preserves the verified 2026-07-13 direction order while excluding the rear
 * driver and the TIM2 alternate-function path from the diagnosis.
 ******************************************************************************
 */

#include "bsp_aux_tb6612.h"
#include "bsp_diag_uart.h"
#include "bsp_motor.h"
#include "Delay.h"

#define FRONT_TEST_RUN_MS       1200U
#define FRONT_TEST_STOP_MS       700U

static void FrontTest_WriteUInt(uint32_t value)
{
    DiagUart_WriteUInt32(value);
}

static void FrontTest_Stop(void)
{
    /* Drive the GPIO latch low before returning PA2/PA3 to TIM2 control. */
    GPIO_ResetBits(MOTOR_PWM_GPIO_PORT, MOTOR_PWMA_PIN | MOTOR_PWMB_PIN);
    Motor_RestorePwmAf();
    Motor_Coast(MOTOR_LEFT);
    Motor_Coast(MOTOR_RIGHT);
    Motor_Enable(0U);
    AuxTb6612_StopAll();
}

static void FrontTest_LogStage(const char *stage)
{
    DiagUart_WriteString("FRONT,event,stage_begin,name,");
    DiagUart_WriteString(stage);
    DiagUart_WriteString(",front_stby,");
    FrontTest_WriteUInt(Motor_GetStbyLevel());
    DiagUart_WriteString(",pwma_gpio,");
    FrontTest_WriteUInt(Motor_GetPwmGpioLevel(MOTOR_LEFT));
    DiagUart_WriteString(",pwmb_gpio,");
    FrontTest_WriteUInt(Motor_GetPwmGpioLevel(MOTOR_RIGHT));
    DiagUart_WriteString(",ain_bits,");
    FrontTest_WriteUInt(Motor_GetDirBits(MOTOR_LEFT));
    DiagUart_WriteString(",bin_bits,");
    FrontTest_WriteUInt(Motor_GetDirBits(MOTOR_RIGHT));
    DiagUart_WriteString("\r\n");
}

static void FrontTest_RunStage(Motor_Index_t motor,
                               int16_t rawSign,
                               const char *stage)
{
    FrontTest_Stop();
    Motor_ForceGpioFull(motor, rawSign);
    FrontTest_LogStage(stage);
    Delay_ms(FRONT_TEST_RUN_MS);
    FrontTest_Stop();
    DiagUart_WriteString("FRONT,event,stage_stop,name,");
    DiagUart_WriteString(stage);
    DiagUart_WriteString("\r\n");
    Delay_ms(FRONT_TEST_STOP_MS);
}

static void FrontTest_RunSequence(void)
{
    DiagUart_WriteString("FRONT,event,sequence_begin,wheels_must_be_raised=1\r\n");

    /* Verified bcd5521 order: LF backward/forward, RF forward/backward. */
    FrontTest_RunStage(MOTOR_LEFT, -1, "LF_BACKWARD");
    FrontTest_RunStage(MOTOR_LEFT, 1, "LF_FORWARD");
    FrontTest_RunStage(MOTOR_RIGHT, 1, "RF_FORWARD");
    FrontTest_RunStage(MOTOR_RIGHT, -1, "RF_BACKWARD");

    FrontTest_Stop();
    DiagUart_WriteString("FRONT,event,sequence_done,front_stby,0,rear_stby,0\r\n");
}

int main(void)
{
    char command;

    SystemCoreClockUpdate();
    Motor_Init();
    AuxTb6612_Init();
    FrontTest_Stop();
    DiagUart_Init(115200U);

    DiagUart_WriteString("FRONT,boot,fw=front_tb6612_direct_test,safe_idle=1,diag=usart1\r\n");
    DiagUart_WriteString("FRONT,config,pwma=pa2,pwmb=pa3,ain1=pe2,ain2=pe3,bin1=pe4,bin2=pe5,stby=pe6\r\n");
    DiagUart_WriteString("FRONT,commands,G=run_direction_sequence,S=stop,H=help\r\n");

    while (1)
    {
        if (DiagUart_TryReadChar(&command) == 0U)
        {
            continue;
        }

        switch (command)
        {
            case 'G':
            case 'g':
                FrontTest_RunSequence();
                break;
            case 'S':
            case 's':
                FrontTest_Stop();
                DiagUart_WriteString("FRONT,event,stopped,front_stby,0,rear_stby,0\r\n");
                break;
            case 'H':
            case 'h':
            case '?':
                DiagUart_WriteString("FRONT,commands,G=run_direction_sequence,S=stop,H=help\r\n");
                break;
            default:
                break;
        }
    }
}
