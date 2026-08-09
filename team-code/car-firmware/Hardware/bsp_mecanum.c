#include "bsp_mecanum.h"

#define MECANUM_PWM_PERIOD 3600U
#define MECANUM_SOFT_PWM_PERIOD_MS 20U

/*
 * The front wheels need higher startup duty under the assembled-car load;
 * the rear wheels are reduced for balance. Keep zero as zero so every stop
 * path remains an immediate stop.
 */
#define MECANUM_LF_PWM_GAIN_NUMERATOR  3U
#define MECANUM_LF_PWM_GAIN_DENOMINATOR 2U
#define MECANUM_LF_PWM_MIN_PERCENT      24U
#define MECANUM_RF_PWM_GAIN_NUMERATOR  3U
#define MECANUM_RF_PWM_GAIN_DENOMINATOR 2U
#define MECANUM_RF_PWM_MIN_PERCENT      24U
#define MECANUM_REAR_PWM_GAIN_NUMERATOR  3U
#define MECANUM_REAR_PWM_GAIN_DENOMINATOR 4U

static uint8_t s_mecanumEnabled = 0U;
static uint8_t s_rearLeftDuty = 0U;
static uint8_t s_rearRightDuty = 0U;

static uint16_t Mecanum_SpeedToCompare(int16_t speedPercent)
{
    uint16_t magnitude;

    if (speedPercent > 100)
    {
        speedPercent = 100;
    }
    else if (speedPercent < -100)
    {
        speedPercent = -100;
    }

    magnitude = (uint16_t)((speedPercent < 0) ? -speedPercent : speedPercent);
    return (uint16_t)(((uint32_t)magnitude * MECANUM_PWM_PERIOD) / 100U);
}

static int16_t Mecanum_ApplyWheelPwmCompensation(MecanumWheel_t wheel,
                                                   int16_t speedPercent)
{
    int16_t sign;
    uint16_t magnitude;
    uint16_t gainNumerator;
    uint16_t gainDenominator;
    uint16_t minimumPercent;

    if (speedPercent == 0)
    {
        return speedPercent;
    }

    if (wheel == MECANUM_WHEEL_LF)
    {
        gainNumerator = MECANUM_LF_PWM_GAIN_NUMERATOR;
        gainDenominator = MECANUM_LF_PWM_GAIN_DENOMINATOR;
        minimumPercent = MECANUM_LF_PWM_MIN_PERCENT;
    }
    else if (wheel == MECANUM_WHEEL_RF)
    {
        gainNumerator = MECANUM_RF_PWM_GAIN_NUMERATOR;
        gainDenominator = MECANUM_RF_PWM_GAIN_DENOMINATOR;
        minimumPercent = MECANUM_RF_PWM_MIN_PERCENT;
    }
    else if ((wheel == MECANUM_WHEEL_LR) ||
             (wheel == MECANUM_WHEEL_RR))
    {
        gainNumerator = MECANUM_REAR_PWM_GAIN_NUMERATOR;
        gainDenominator = MECANUM_REAR_PWM_GAIN_DENOMINATOR;
        minimumPercent = 0U;
    }
    else
    {
        return speedPercent;
    }

    sign = (speedPercent < 0) ? -1 : 1;
    magnitude = (uint16_t)((speedPercent < 0) ? -speedPercent : speedPercent);
    magnitude = (uint16_t)(((uint32_t)magnitude * gainNumerator) /
                           gainDenominator);
    if ((minimumPercent != 0U) && (magnitude < minimumPercent))
    {
        magnitude = minimumPercent;
    }
    if (magnitude > 100U)
    {
        magnitude = 100U;
    }

    return (int16_t)(sign * (int16_t)magnitude);
}

static void Mecanum_InitTimer(TIM_TypeDef *timer,
                              uint8_t useChannelsOneTwo)
{
    TIM_TimeBaseInitTypeDef timeBase;
    TIM_OCInitTypeDef outputCompare;

    TIM_TimeBaseStructInit(&timeBase);
    timeBase.TIM_Period = MECANUM_PWM_PERIOD - 1U;
    timeBase.TIM_Prescaler = 0U;
    timeBase.TIM_ClockDivision = TIM_CKD_DIV1;
    timeBase.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(timer, &timeBase);

    TIM_OCStructInit(&outputCompare);
    outputCompare.TIM_OCMode = TIM_OCMode_PWM1;
    outputCompare.TIM_OutputState = TIM_OutputState_Enable;
    outputCompare.TIM_Pulse = 0U;
    outputCompare.TIM_OCPolarity = TIM_OCPolarity_High;

    if (useChannelsOneTwo != 0U)
    {
        TIM_OC1Init(timer, &outputCompare);
        TIM_OC2Init(timer, &outputCompare);
        TIM_OC1PreloadConfig(timer, TIM_OCPreload_Enable);
        TIM_OC2PreloadConfig(timer, TIM_OCPreload_Enable);
    }
    else
    {
        TIM_OC3Init(timer, &outputCompare);
        TIM_OC4Init(timer, &outputCompare);
        TIM_OC3PreloadConfig(timer, TIM_OCPreload_Enable);
        TIM_OC4PreloadConfig(timer, TIM_OCPreload_Enable);
    }

    TIM_ARRPreloadConfig(timer, ENABLE);
    TIM_Cmd(timer, ENABLE);
}

static void Mecanum_SetDirection(MecanumWheel_t wheel, int16_t speedPercent)
{
    if (speedPercent == 0)
    {
        switch (wheel)
        {
            case MECANUM_WHEEL_LF:
                GPIO_ResetBits(GPIOE, GPIO_Pin_2 | GPIO_Pin_3);
                break;
            case MECANUM_WHEEL_RF:
                GPIO_ResetBits(GPIOE, GPIO_Pin_4 | GPIO_Pin_5);
                break;
            case MECANUM_WHEEL_LR:
                GPIO_ResetBits(GPIOC, GPIO_Pin_6);
                GPIO_ResetBits(GPIOG, GPIO_Pin_1);
                break;
            case MECANUM_WHEEL_RR:
                GPIO_ResetBits(GPIOE, GPIO_Pin_9 | GPIO_Pin_7);
                break;
            default:
                break;
        }
        return;
    }

    switch (wheel)
    {
        case MECANUM_WHEEL_LF:
            if (speedPercent > 0)
            {
                /* Verified physical forward: PE2=0, PE3=1. */
                GPIO_ResetBits(GPIOE, GPIO_Pin_2);
                GPIO_SetBits(GPIOE, GPIO_Pin_3);
            }
            else
            {
                GPIO_SetBits(GPIOE, GPIO_Pin_2);
                GPIO_ResetBits(GPIOE, GPIO_Pin_3);
            }
            break;

        case MECANUM_WHEEL_RF:
            if (speedPercent > 0)
            {
                /* Current physical forward: PE4=0, PE5=1. */
                GPIO_ResetBits(GPIOE, GPIO_Pin_4);
                GPIO_SetBits(GPIOE, GPIO_Pin_5);
            }
            else
            {
                GPIO_SetBits(GPIOE, GPIO_Pin_4);
                GPIO_ResetBits(GPIOE, GPIO_Pin_5);
            }
            break;

        case MECANUM_WHEEL_LR:
            if (speedPercent > 0)
            {
                /* Current physical forward: PC6=1, PG1=0. */
                GPIO_SetBits(GPIOC, GPIO_Pin_6);
                GPIO_ResetBits(GPIOG, GPIO_Pin_1);
            }
            else
            {
                GPIO_ResetBits(GPIOC, GPIO_Pin_6);
                GPIO_SetBits(GPIOG, GPIO_Pin_1);
            }
            break;

        case MECANUM_WHEEL_RR:
            if (speedPercent > 0)
            {
                /* Current physical forward: PE9=0, PE7=1. */
                GPIO_ResetBits(GPIOE, GPIO_Pin_9);
                GPIO_SetBits(GPIOE, GPIO_Pin_7);
            }
            else
            {
                GPIO_SetBits(GPIOE, GPIO_Pin_9);
                GPIO_ResetBits(GPIOE, GPIO_Pin_7);
            }
            break;

        default:
            break;
    }
}

static void Mecanum_SetCompare(MecanumWheel_t wheel, uint16_t compare)
{
    switch (wheel)
    {
        case MECANUM_WHEEL_LF: TIM_SetCompare3(TIM2, compare); break;
        case MECANUM_WHEEL_RF: TIM_SetCompare4(TIM2, compare); break;
        case MECANUM_WHEEL_LR:
            s_rearLeftDuty = (uint8_t)(((uint32_t)compare * 100U) /
                                       MECANUM_PWM_PERIOD);
            break;
        case MECANUM_WHEEL_RR:
            s_rearRightDuty = (uint8_t)(((uint32_t)compare * 100U) /
                                        MECANUM_PWM_PERIOD);
            break;
        default: break;
    }
}

void Mecanum_InitOff(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOC |
                           RCC_APB2Periph_GPIOE |
                           RCC_APB2Periph_GPIOG |
                           RCC_APB2Periph_AFIO, ENABLE);

    GPIO_ResetBits(GPIOA, GPIO_Pin_2 | GPIO_Pin_3 |
                          GPIO_Pin_6 | GPIO_Pin_7);
    GPIO_ResetBits(GPIOE, GPIO_Pin_2 | GPIO_Pin_3 |
                          GPIO_Pin_4 | GPIO_Pin_5 |
                          GPIO_Pin_6 | GPIO_Pin_7 |
                          GPIO_Pin_9);
    GPIO_ResetBits(GPIOC, GPIO_Pin_6);
    GPIO_ResetBits(GPIOG, GPIO_Pin_1);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 |
                    GPIO_Pin_4 | GPIO_Pin_5 |
                    GPIO_Pin_6 | GPIO_Pin_7 |
                    GPIO_Pin_9;
    GPIO_Init(GPIOE, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOC, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOG, &gpio);

    Mecanum_InitTimer(TIM2, 0U);
    s_rearLeftDuty = 0U;
    s_rearRightDuty = 0U;
    s_mecanumEnabled = 0U;
    Mecanum_StopAll();
    Mecanum_Enable(0U);
}

void Mecanum_Enable(uint8_t enable)
{
    if (enable != 0U)
    {
        GPIO_SetBits(GPIOE, GPIO_Pin_6);
        s_mecanumEnabled = 1U;
    }
    else
    {
        s_mecanumEnabled = 0U;
        GPIO_ResetBits(GPIOE, GPIO_Pin_6);
        GPIO_ResetBits(GPIOA, GPIO_Pin_6 | GPIO_Pin_7);
        Mecanum_StopAll();
    }
}

void Mecanum_SetWheel(MecanumWheel_t wheel, int16_t speedPercent)
{
    if (wheel > MECANUM_WHEEL_RR)
    {
        return;
    }

    speedPercent = Mecanum_ApplyWheelPwmCompensation(wheel, speedPercent);
    Mecanum_SetDirection(wheel, speedPercent);
    Mecanum_SetCompare(wheel, Mecanum_SpeedToCompare(speedPercent));
}

void Mecanum_SetWheelRaw(MecanumWheel_t wheel, int16_t speedPercent)
{
    if (wheel > MECANUM_WHEEL_RR)
    {
        return;
    }

    /* Calibration must measure the requested duty, not the normal trim. */
    Mecanum_SetDirection(wheel, speedPercent);
    Mecanum_SetCompare(wheel, Mecanum_SpeedToCompare(speedPercent));
}

void Mecanum_StopAll(void)
{
    Mecanum_SetWheel(MECANUM_WHEEL_LF, 0);
    Mecanum_SetWheel(MECANUM_WHEEL_RF, 0);
    Mecanum_SetWheel(MECANUM_WHEEL_LR, 0);
    Mecanum_SetWheel(MECANUM_WHEEL_RR, 0);
}

void Mecanum_Update(uint32_t nowMs)
{
    uint8_t phase;

    if (s_mecanumEnabled == 0U)
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_6 | GPIO_Pin_7);
        return;
    }

    phase = (uint8_t)(nowMs % MECANUM_SOFT_PWM_PERIOD_MS);

    /* 20 ms period gives rear GPIO PWM 5% resolution for low-duty sweeps. */
    if ((uint16_t)phase * 5U < s_rearLeftDuty)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_6);
    }
    else
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_6);
    }

    if ((uint16_t)phase * 5U < s_rearRightDuty)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_7);
    }
    else
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_7);
    }
}
