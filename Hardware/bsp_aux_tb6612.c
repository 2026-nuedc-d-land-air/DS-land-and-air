/**
 ******************************************************************************
 * @file    bsp_aux_tb6612.c
 * @brief   Added TB6612 with TIM3 PWM and rear-wheel phase observation.
 ******************************************************************************
 */

#include "bsp_aux_tb6612.h"

static uint8_t s_auxEnabled;
static uint8_t s_lastEncoderA;
static uint8_t s_lastEncoderB;
static volatile uint16_t s_encoderTransitionsA;
static volatile uint16_t s_encoderTransitionsB;

static uint16_t AuxTb6612_PercentToCompare(int16_t rawPercent)
{
    uint16_t duty;

    if (rawPercent < 0)
    {
        rawPercent = (int16_t)-rawPercent;
    }
    if (rawPercent > 100)
    {
        rawPercent = 100;
    }
    duty = (uint16_t)rawPercent;
    return (uint16_t)(((uint32_t)duty * AUX_TB6612_PWM_PERIOD) / 100U);
}

static void AuxTb6612_SetDirectionA(int16_t rawPercent)
{
    if (rawPercent > 0)
    {
        GPIO_SetBits(AUX_TB6612_AIN_GPIO_PORT, AUX_TB6612_AIN1_PIN);
        GPIO_ResetBits(AUX_TB6612_AIN_GPIO_PORT, AUX_TB6612_AIN2_PIN);
    }
    else if (rawPercent < 0)
    {
        GPIO_ResetBits(AUX_TB6612_AIN_GPIO_PORT, AUX_TB6612_AIN1_PIN);
        GPIO_SetBits(AUX_TB6612_AIN_GPIO_PORT, AUX_TB6612_AIN2_PIN);
    }
    else
    {
        GPIO_ResetBits(AUX_TB6612_AIN_GPIO_PORT,
                       AUX_TB6612_AIN1_PIN | AUX_TB6612_AIN2_PIN);
    }
}

static void AuxTb6612_SetDirectionB(int16_t rawPercent)
{
    if (rawPercent > 0)
    {
        GPIO_SetBits(AUX_TB6612_BIN_GPIO_PORT, AUX_TB6612_BIN1_PIN);
        GPIO_ResetBits(AUX_TB6612_BIN_GPIO_PORT, AUX_TB6612_BIN2_PIN);
    }
    else if (rawPercent < 0)
    {
        GPIO_ResetBits(AUX_TB6612_BIN_GPIO_PORT, AUX_TB6612_BIN1_PIN);
        GPIO_SetBits(AUX_TB6612_BIN_GPIO_PORT, AUX_TB6612_BIN2_PIN);
    }
    else
    {
        GPIO_ResetBits(AUX_TB6612_BIN_GPIO_PORT,
                       AUX_TB6612_BIN1_PIN | AUX_TB6612_BIN2_PIN);
    }
}

static uint8_t AuxTb6612_ReadEncoderState(AuxTb6612Motor_t motor)
{
    uint16_t input = AUX_TB6612_ENC_GPIO_PORT->IDR;

    if (motor == AUX_TB6612_MOTOR_A)
    {
        return (uint8_t)(((input & AUX_TB6612_LEFT_ENC_A_PIN) != 0U ? 2U : 0U) |
                         ((input & AUX_TB6612_LEFT_ENC_B_PIN) != 0U ? 1U : 0U));
    }
    return (uint8_t)(((input & AUX_TB6612_RIGHT_ENC_A_PIN) != 0U ? 2U : 0U) |
                     ((input & AUX_TB6612_RIGHT_ENC_B_PIN) != 0U ? 1U : 0U));
}

static void AuxTb6612_GpioInit(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(AUX_TB6612_AIN_GPIO_CLK |
                           AUX_TB6612_BIN_GPIO_CLK |
                           AUX_TB6612_STBY_GPIO_CLK |
                           AUX_TB6612_PWM_GPIO_CLK |
                           AUX_TB6612_ENC_GPIO_CLK |
                           RCC_APB2Periph_AFIO, ENABLE);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;

    gpio.GPIO_Pin = AUX_TB6612_AIN1_PIN | AUX_TB6612_AIN2_PIN;
    GPIO_Init(AUX_TB6612_AIN_GPIO_PORT, &gpio);
    GPIO_ResetBits(AUX_TB6612_AIN_GPIO_PORT,
                   AUX_TB6612_AIN1_PIN | AUX_TB6612_AIN2_PIN);

    gpio.GPIO_Pin = AUX_TB6612_BIN1_PIN | AUX_TB6612_BIN2_PIN;
    GPIO_Init(AUX_TB6612_BIN_GPIO_PORT, &gpio);
    GPIO_ResetBits(AUX_TB6612_BIN_GPIO_PORT,
                   AUX_TB6612_BIN1_PIN | AUX_TB6612_BIN2_PIN);

    gpio.GPIO_Pin = AUX_TB6612_STBY_PIN;
    GPIO_Init(AUX_TB6612_STBY_GPIO_PORT, &gpio);
    GPIO_ResetBits(AUX_TB6612_STBY_GPIO_PORT, AUX_TB6612_STBY_PIN);

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
    gpio.GPIO_Pin = AUX_TB6612_PWMA_PIN | AUX_TB6612_PWMB_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(AUX_TB6612_PWM_GPIO_PORT, &gpio);

    gpio.GPIO_Pin = AUX_TB6612_LEFT_ENC_A_PIN |
                    AUX_TB6612_LEFT_ENC_B_PIN |
                    AUX_TB6612_RIGHT_ENC_A_PIN |
                    AUX_TB6612_RIGHT_ENC_B_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(AUX_TB6612_ENC_GPIO_PORT, &gpio);
}

static void AuxTb6612_PwmInit(void)
{
    TIM_TimeBaseInitTypeDef timebase;
    TIM_OCInitTypeDef outputCompare;

    RCC_APB1PeriphClockCmd(AUX_TB6612_TIM_CLK, ENABLE);
    TIM_TimeBaseStructInit(&timebase);
    timebase.TIM_Period = AUX_TB6612_PWM_PERIOD - 1U;
    timebase.TIM_Prescaler = 0U;
    timebase.TIM_ClockDivision = TIM_CKD_DIV1;
    timebase.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(AUX_TB6612_TIM, &timebase);

    TIM_OCStructInit(&outputCompare);
    outputCompare.TIM_OCMode = TIM_OCMode_PWM1;
    outputCompare.TIM_OutputState = TIM_OutputState_Enable;
    outputCompare.TIM_Pulse = 0U;
    outputCompare.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC3Init(AUX_TB6612_TIM, &outputCompare);
    TIM_OC4Init(AUX_TB6612_TIM, &outputCompare);
    TIM_OC3PreloadConfig(AUX_TB6612_TIM, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(AUX_TB6612_TIM, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(AUX_TB6612_TIM, ENABLE);
    TIM_Cmd(AUX_TB6612_TIM, ENABLE);
}

void AuxTb6612_Init(void)
{
    s_auxEnabled = 0U;
    s_encoderTransitionsA = 0U;
    s_encoderTransitionsB = 0U;
    AuxTb6612_GpioInit();
    AuxTb6612_PwmInit();
    s_lastEncoderA = AuxTb6612_ReadEncoderState(AUX_TB6612_MOTOR_A);
    s_lastEncoderB = AuxTb6612_ReadEncoderState(AUX_TB6612_MOTOR_B);
    AuxTb6612_StopAll();
}

void AuxTb6612_Enable(uint8_t enable)
{
    if (enable != 0U)
    {
        GPIO_SetBits(AUX_TB6612_STBY_GPIO_PORT, AUX_TB6612_STBY_PIN);
        s_auxEnabled = 1U;
    }
    else
    {
        GPIO_ResetBits(AUX_TB6612_STBY_GPIO_PORT, AUX_TB6612_STBY_PIN);
        s_auxEnabled = 0U;
    }
}

uint8_t AuxTb6612_IsEnabled(void)
{
    return s_auxEnabled;
}

void AuxTb6612_SetRawSpeed(AuxTb6612Motor_t motor, int16_t rawPercent)
{
    uint16_t compare;

    if (rawPercent > 100)
    {
        rawPercent = 100;
    }
    if (rawPercent < -100)
    {
        rawPercent = -100;
    }
    compare = AuxTb6612_PercentToCompare(rawPercent);

    if (motor == AUX_TB6612_MOTOR_A)
    {
        AuxTb6612_SetDirectionA(rawPercent);
        TIM_SetCompare3(AUX_TB6612_TIM, compare);
    }
    else
    {
        AuxTb6612_SetDirectionB(rawPercent);
        TIM_SetCompare4(AUX_TB6612_TIM, compare);
    }
}

void AuxTb6612_RestorePwmAf(void)
{
    GPIO_InitTypeDef gpio;

    TIM_SetCompare3(AUX_TB6612_TIM, 0U);
    TIM_SetCompare4(AUX_TB6612_TIM, 0U);

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = AUX_TB6612_PWMA_PIN | AUX_TB6612_PWMB_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(AUX_TB6612_PWM_GPIO_PORT, &gpio);
}

void AuxTb6612_ForceGpioFull(AuxTb6612Motor_t motor, int16_t rawSign)
{
    GPIO_InitTypeDef gpio;
    uint16_t pin;

    /* Kill timer duty first, then take the pin as plain GPIO high = 100% on. */
    TIM_SetCompare3(AUX_TB6612_TIM, 0U);
    TIM_SetCompare4(AUX_TB6612_TIM, 0U);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin = AUX_TB6612_PWMA_PIN | AUX_TB6612_PWMB_PIN;
    GPIO_Init(AUX_TB6612_PWM_GPIO_PORT, &gpio);
    GPIO_ResetBits(AUX_TB6612_PWM_GPIO_PORT,
                   AUX_TB6612_PWMA_PIN | AUX_TB6612_PWMB_PIN);

    if (motor == AUX_TB6612_MOTOR_A)
    {
        AuxTb6612_SetDirectionA(rawSign);
        AuxTb6612_SetDirectionB(0);
        pin = AUX_TB6612_PWMA_PIN;
    }
    else
    {
        AuxTb6612_SetDirectionA(0);
        AuxTb6612_SetDirectionB(rawSign);
        pin = AUX_TB6612_PWMB_PIN;
    }

    GPIO_SetBits(AUX_TB6612_PWM_GPIO_PORT, pin);
    AuxTb6612_Enable(1U);
}

void AuxTb6612_StopAll(void)
{
    TIM_SetCompare3(AUX_TB6612_TIM, 0U);
    TIM_SetCompare4(AUX_TB6612_TIM, 0U);
    GPIO_ResetBits(AUX_TB6612_PWM_GPIO_PORT,
                   AUX_TB6612_PWMA_PIN | AUX_TB6612_PWMB_PIN);
    AuxTb6612_RestorePwmAf();
    GPIO_ResetBits(AUX_TB6612_AIN_GPIO_PORT,
                   AUX_TB6612_AIN1_PIN | AUX_TB6612_AIN2_PIN);
    GPIO_ResetBits(AUX_TB6612_BIN_GPIO_PORT,
                   AUX_TB6612_BIN1_PIN | AUX_TB6612_BIN2_PIN);
    AuxTb6612_Enable(0U);
}

void AuxTb6612_EncoderPoll(void)
{
    uint8_t currentA = AuxTb6612_ReadEncoderState(AUX_TB6612_MOTOR_A);
    uint8_t currentB = AuxTb6612_ReadEncoderState(AUX_TB6612_MOTOR_B);

    if (currentA != s_lastEncoderA)
    {
        ++s_encoderTransitionsA;
        s_lastEncoderA = currentA;
    }
    if (currentB != s_lastEncoderB)
    {
        ++s_encoderTransitionsB;
        s_lastEncoderB = currentB;
    }
}

uint16_t AuxTb6612_GetEncoderTransitions(AuxTb6612Motor_t motor)
{
    return (motor == AUX_TB6612_MOTOR_A) ?
           s_encoderTransitionsA : s_encoderTransitionsB;
}

uint16_t AuxTb6612_GetPwmCompare(AuxTb6612Motor_t motor)
{
    if (motor == AUX_TB6612_MOTOR_A)
    {
        return (uint16_t)TIM_GetCapture3(AUX_TB6612_TIM);
    }
    return (uint16_t)TIM_GetCapture4(AUX_TB6612_TIM);
}

uint8_t AuxTb6612_GetDirBits(AuxTb6612Motor_t motor)
{
    if (motor == AUX_TB6612_MOTOR_A)
    {
        uint16_t idr = AUX_TB6612_AIN_GPIO_PORT->IDR;
        return (uint8_t)(((idr & AUX_TB6612_AIN1_PIN) != 0U ? 2U : 0U) |
                         ((idr & AUX_TB6612_AIN2_PIN) != 0U ? 1U : 0U));
    }
    else
    {
        uint16_t idr = AUX_TB6612_BIN_GPIO_PORT->IDR;
        return (uint8_t)(((idr & AUX_TB6612_BIN1_PIN) != 0U ? 2U : 0U) |
                         ((idr & AUX_TB6612_BIN2_PIN) != 0U ? 1U : 0U));
    }
}

uint8_t AuxTb6612_GetPwmGpioLevel(AuxTb6612Motor_t motor)
{
    uint16_t idr = AUX_TB6612_PWM_GPIO_PORT->IDR;
    uint16_t pin = (motor == AUX_TB6612_MOTOR_A) ?
                   AUX_TB6612_PWMA_PIN : AUX_TB6612_PWMB_PIN;
    return ((idr & pin) != 0U) ? 1U : 0U;
}
