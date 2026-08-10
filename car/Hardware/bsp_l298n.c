#include "bsp_l298n.h"

#include "Delay.h"

/*
 * PWM / 方向映射：
 * LF: PA0 TIM5_CH1, PC7/PE13 (IN1/IN2)
 * RF: PA1 TIM5_CH2, PE14/PE10 (IN3/IN4)
 * LR: PA6 TIM3_CH1, PC6/PG1 (IN1/IN2)
 * RR: PA7 TIM3_CH2, PE9/PE7 (IN3/IN4)
 * 使用PWM时必须拔掉对应的ENA/ENB跳线帽。
 */

#define L298N_PWM_PERIOD          3600U
#define L298N_TEST_SPEED          60
#define L298N_RUN_TIME_MS         2000U
#define L298N_DIRECTION_PAUSE_MS  1000U
#define L298N_WHEEL_PAUSE_MS      3000U
#define L298N_BOOT_PAUSE_MS       3000U

typedef struct
{
    GPIO_TypeDef *in1Port;
    uint16_t in1Pin;
    GPIO_TypeDef *in2Port;
    uint16_t in2Pin;
} L298N_DirectionPins_t;

static const L298N_DirectionPins_t s_directionPins[4] =
{
    {GPIOC, GPIO_Pin_7,  GPIOE, GPIO_Pin_13},
    {GPIOE, GPIO_Pin_14, GPIOE, GPIO_Pin_10},
    {GPIOC, GPIO_Pin_6,  GPIOG, GPIO_Pin_1},
    {GPIOE, GPIO_Pin_9,  GPIOE, GPIO_Pin_7}
};

volatile L298N_TestStage_t g_l298nTestStage = L298N_STAGE_BOOT_PAUSE;
volatile uint32_t g_l298nTestCycle = 0U;

static uint16_t L298N_SpeedToCompare(int16_t speedPercent)
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
    return (uint16_t)(((uint32_t)magnitude * L298N_PWM_PERIOD) / 100U);
}

static void L298N_SetPwm(L298N_Wheel_t wheel, uint16_t compare)
{
    if (compare > L298N_PWM_PERIOD)
    {
        compare = L298N_PWM_PERIOD;
    }

    switch (wheel)
    {
        case L298N_WHEEL_LF: TIM_SetCompare1(TIM5, compare); break;
        case L298N_WHEEL_RF: TIM_SetCompare2(TIM5, compare); break;
        case L298N_WHEEL_LR: TIM_SetCompare1(TIM3, compare); break;
        case L298N_WHEEL_RR: TIM_SetCompare2(TIM3, compare); break;
        default: break;
    }
}

static void L298N_InitPwmTimer(TIM_TypeDef *timer)
{
    TIM_TimeBaseInitTypeDef timeBase;
    TIM_OCInitTypeDef outputCompare;

    timeBase.TIM_Period = L298N_PWM_PERIOD - 1U;
    timeBase.TIM_Prescaler = 0U;
    timeBase.TIM_ClockDivision = TIM_CKD_DIV1;
    timeBase.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(timer, &timeBase);

    TIM_OCStructInit(&outputCompare);
    outputCompare.TIM_OCMode = TIM_OCMode_PWM1;
    outputCompare.TIM_OutputState = TIM_OutputState_Enable;
    outputCompare.TIM_Pulse = 0U;
    outputCompare.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(timer, &outputCompare);
    TIM_OC2Init(timer, &outputCompare);
    TIM_OC3Init(timer, &outputCompare);
    TIM_OC1PreloadConfig(timer, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(timer, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(timer, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(timer, ENABLE);
    TIM_Cmd(timer, ENABLE);
}

void L298N_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM5, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOC |
                           RCC_APB2Periph_GPIOE |
                           RCC_APB2Periph_GPIOG |
                           RCC_APB2Periph_AFIO, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 |
                    GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 |
                    GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_Init(GPIOE, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOC, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOG, &gpio);

    L298N_InitPwmTimer(TIM5);
    L298N_InitPwmTimer(TIM3);
    L298N_StopAll();
}

void L298N_SetWheel(L298N_Wheel_t wheel, int16_t speedPercent)
{
    const L298N_DirectionPins_t *pins;

    if (wheel > L298N_WHEEL_RR)
    {
        return;
    }

    pins = &s_directionPins[wheel];

    if (speedPercent > 0)
    {
        GPIO_SetBits(pins->in1Port, pins->in1Pin);
        GPIO_ResetBits(pins->in2Port, pins->in2Pin);
    }
    else if (speedPercent < 0)
    {
        GPIO_ResetBits(pins->in1Port, pins->in1Pin);
        GPIO_SetBits(pins->in2Port, pins->in2Pin);
    }
    else
    {
        GPIO_ResetBits(pins->in1Port, pins->in1Pin);
        GPIO_ResetBits(pins->in2Port, pins->in2Pin);
    }

    L298N_SetPwm(wheel, L298N_SpeedToCompare(speedPercent));
}

void L298N_StopAll(void)
{
    L298N_SetWheel(L298N_WHEEL_LF, 0);
    L298N_SetWheel(L298N_WHEEL_LR, 0);
    L298N_SetWheel(L298N_WHEEL_RF, 0);
    L298N_SetWheel(L298N_WHEEL_RR, 0);
}

void L298N_SetFrontEnableStatic(uint8_t leftEnable, uint8_t rightEnable)
{
    GPIO_InitTypeDef gpio;

    TIM_Cmd(TIM5, DISABLE);

    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    GPIO_ResetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1);
    if (leftEnable != 0U)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_0);
    }
    if (rightEnable != 0U)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_1);
    }
}

static void L298N_RunOneStage(L298N_TestStage_t stage,
                             L298N_Wheel_t wheel,
                             int16_t speedPercent,
                             uint32_t stopTimeMs)
{
    L298N_StopAll();
    g_l298nTestStage = stage;
    L298N_SetWheel(wheel, speedPercent);
    Delay_ms(L298N_RUN_TIME_MS);
    L298N_StopAll();
    if (stopTimeMs != 0U)
    {
        Delay_ms(stopTimeMs);
    }
}

void L298N_RunDirectionTestLoop(void)
{
    g_l298nTestCycle++;
    g_l298nTestStage = L298N_STAGE_BOOT_PAUSE;
    L298N_StopAll();
    Delay_ms(L298N_BOOT_PAUSE_MS);

    L298N_RunOneStage(L298N_STAGE_LF_REVERSE, L298N_WHEEL_LF,
                     -L298N_TEST_SPEED, L298N_WHEEL_PAUSE_MS);
    L298N_RunOneStage(L298N_STAGE_RF_REVERSE, L298N_WHEEL_RF,
                     -L298N_TEST_SPEED, L298N_WHEEL_PAUSE_MS);
    L298N_RunOneStage(L298N_STAGE_LR_REVERSE, L298N_WHEEL_LR,
                     -L298N_TEST_SPEED, L298N_WHEEL_PAUSE_MS);
    L298N_RunOneStage(L298N_STAGE_RR_REVERSE, L298N_WHEEL_RR,
                     -L298N_TEST_SPEED, L298N_WHEEL_PAUSE_MS);
}
