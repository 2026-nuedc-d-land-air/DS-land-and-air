/**
  * @file    bsp_led_pwm.c
  * @brief   LED PWM亮度控制模块实现
  * @note    使用TIM3中断实现软件PWM (10kHz)
  */

#include "bsp_led_pwm.h"

/*====================================================================================*/
/*                                  配置参数                                           */
/*====================================================================================*/

/* 亮度调节等级: 75 -> 90 -> 100 -> 15 -> 30 -> 45 -> 60 -> 75 */
static const uint8_t s_brightnessLevels[] = {75, 90, 100, 15, 30, 45, 60};
#define BRIGHTNESS_LEVEL_COUNT (sizeof(s_brightnessLevels) / sizeof(s_brightnessLevels[0]))

/*====================================================================================*/
/*                                  私有变量                                           */
/*====================================================================================*/

/* LED状态 */
static uint8_t s_ledBrightness[LED_COUNT] = {50, 80, 50};   /* 当前PWM亮度 */
static uint8_t s_ledEnabled[LED_COUNT] = {0, 0, 0};         /* PWM开关状态 */
static uint8_t s_pwmCounter = 0;                             /* PWM计数器 */

/* 保存的亮度值 (正常页面使用) */
static uint8_t s_savedBrightness = 75;  /* 默认亮度 */

/* 调节模式 */
static BrightnessMode_t s_adjustMode = BRIGHTNESS_MODE_NORMAL;
static uint8_t s_currentLevelIdx = 0;   /* 当前亮度等级索引 */

/* 闪烁控制 (指示灯) */
static uint8_t s_indicatorBlinkCount = 0;
static uint16_t s_indicatorTimer = 0;
static uint8_t s_indicatorPhase = 0;    /* 0=灭, 1=亮 */

/* 终点流水灯效果控制 */
static uint8_t s_finishEffectActive = 0;    /* 终点效果是否激活 */
static uint8_t s_finishRound = 0;           /* 当前轮次 (0-1, 共2轮) */
static uint8_t s_finishLedIdx = 0;          /* 当前LED索引 (0=绿, 1=红, 2=指示) */
static uint8_t s_finishPhase = 0;           /* 0=亮, 1=灭 */
static uint16_t s_finishTimer = 0;          /* 流水灯计时器 */

/* 避障提示灯效控制 (两两交替) */
static uint8_t s_obstacleEffectActive = 0;
static uint8_t s_obstacleStep = 0;
static uint8_t s_obstacleRound = 0;
static uint16_t s_obstacleTimerMs = 0;

/* 终点效果参数 - 快速流水灯，持续整个蜂鸣器时间 */
#define FINISH_ON_CALLS         1       /* 亮的调用次数 (1*50ms=50ms) */
#define FINISH_OFF_CALLS        1       /* 灭的调用次数 (1*50ms=50ms) - 快速切换 */
#define FINISH_ROUNDS           3       /* 3轮，约1.8秒，覆盖蜂鸣器三声 */

/* 避障灯效参数 */
#define OBSTACLE_STEP_MS        200U
#define OBSTACLE_ROUNDS         1U

/* 前向声明 */
static void LED_FinishEffectUpdate(uint32_t deltaMs);
static void LED_ObstacleEffectUpdate(uint32_t deltaMs);

static void (*s_uiRefreshHook)(void) = 0;

/*====================================================================================*/
/*                                  GPIO操作                                           */
/*====================================================================================*/

static void LED_GPIO_On(LED_Index_t led)
{
    switch (led)
    {
        case LED_GREEN:     GPIO_ResetBits(LED1_GPIO_PORT, LED1_GPIO_PIN); break;
        case LED_RED:       GPIO_ResetBits(LED2_GPIO_PORT, LED2_GPIO_PIN); break;
        case LED_INDICATOR: GPIO_ResetBits(LED3_GPIO_PORT, LED3_GPIO_PIN); break;
        default: break;
    }
}

static void LED_GPIO_Off(LED_Index_t led)
{
    switch (led)
    {
        case LED_GREEN:     GPIO_SetBits(LED1_GPIO_PORT, LED1_GPIO_PIN); break;
        case LED_RED:       GPIO_SetBits(LED2_GPIO_PORT, LED2_GPIO_PIN); break;
        case LED_INDICATOR: GPIO_SetBits(LED3_GPIO_PORT, LED3_GPIO_PIN); break;
        default: break;
    }
}

/*====================================================================================*/
/*                                  TIM6 软件PWM                                       */
/*====================================================================================*/

void LED_TIM6_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    
    /* 10kHz 中断 (0.1ms) */
    /* TIM6时钟 = 72MHz (APB1=36MHz, x2=72MHz) */
    TIM_TimeBaseStructure.TIM_Period = 100 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
    
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    TIM_Cmd(TIM6, ENABLE);
}

void TIM6_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
        
        s_pwmCounter++;
        if (s_pwmCounter >= LED_PWM_PERIOD) s_pwmCounter = 0;
        
        for (int i = 0; i < LED_COUNT; i++)
        {
            if (s_ledEnabled[i] && s_pwmCounter < s_ledBrightness[i])
                LED_GPIO_On((LED_Index_t)i);
            else
                LED_GPIO_Off((LED_Index_t)i);
        }
    }
}

/*====================================================================================*/
/*                                  初始化                                             */
/*====================================================================================*/

void LED_PWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(LED1_GPIO_CLK | LED2_GPIO_CLK | LED3_GPIO_CLK, ENABLE);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    GPIO_InitStructure.GPIO_Pin = LED1_GPIO_PIN; GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = LED2_GPIO_PIN; GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = LED3_GPIO_PIN; GPIO_Init(LED3_GPIO_PORT, &GPIO_InitStructure);
    
    LED_GPIO_Off(LED_GREEN);
    LED_GPIO_Off(LED_RED);
    LED_GPIO_Off(LED_INDICATOR);
    
    LED_TIM6_Init();
}

/*====================================================================================*/
/*                                  接口函数                                           */
/*====================================================================================*/

void LED_SetBrightness(LED_Index_t led, uint8_t brightness)
{
    if (led < LED_COUNT && !s_finishEffectActive && !s_obstacleEffectActive)
    {
        s_ledBrightness[led] = (brightness > 100) ? 100 : brightness;
    }
}

uint8_t LED_GetBrightness(LED_Index_t led)
{
    return (led < LED_COUNT) ? s_ledBrightness[led] : 0;
}

void LED_Switch(LED_Index_t led, uint8_t state)
{
    if (led < LED_COUNT && !s_finishEffectActive && !s_obstacleEffectActive)
    {
        s_ledEnabled[led] = state ? 1 : 0;
    }
}

uint8_t LED_GetState(LED_Index_t led)
{
    return (led < LED_COUNT) ? s_ledEnabled[led] : 0;
}

void LED_IndicatorBlink2(void)
{
    if (s_finishEffectActive || s_obstacleEffectActive)
    {
        return;
    }
    s_indicatorBlinkCount = 2;
    s_indicatorTimer = 0;
    s_indicatorPhase = 1; /* 开始亮 */
    s_ledBrightness[LED_INDICATOR] = 80; /* 固定亮度 */
    s_ledEnabled[LED_INDICATOR] = 1;
}

/*====================================================================================*/
/*                                  调节模式                                           */
/*====================================================================================*/

void LED_EnterAdjustMode(void)
{
    s_adjustMode = BRIGHTNESS_MODE_ADJUST;
    /* 两个灯都临时开启 */
    s_ledEnabled[LED_GREEN] = 1;
    s_ledEnabled[LED_RED] = 1;
    
    /* 设置为当前保存的亮度 */
    s_ledBrightness[LED_GREEN] = s_savedBrightness;
    s_ledBrightness[LED_RED] = s_savedBrightness;
    
    /* 指示灯也同步开启点亮，方便观察 */
    s_ledEnabled[LED_INDICATOR] = 1;
    s_ledBrightness[LED_INDICATOR] = s_savedBrightness;
}

void LED_ExitAdjustMode(void)
{
    s_adjustMode = BRIGHTNESS_MODE_NORMAL;
    /* 退出后，亮度已由 savedBrightness 记录 */
    s_ledEnabled[LED_INDICATOR] = 0;
}

void LED_ChangeBrightnessStep(void)
{
    if (s_adjustMode != BRIGHTNESS_MODE_ADJUST) return;
    
    s_currentLevelIdx = (s_currentLevelIdx + 1) % BRIGHTNESS_LEVEL_COUNT;
    s_savedBrightness = s_brightnessLevels[s_currentLevelIdx];
    
    /* 实时更新显示 */
    s_ledBrightness[LED_GREEN] = s_savedBrightness;
    s_ledBrightness[LED_RED] = s_savedBrightness;
    s_ledBrightness[LED_INDICATOR] = s_savedBrightness;
}

void LED_AdjustModeUpdate(uint32_t deltaMs)
{
    /* 0. 处理终点流水灯效果 (优先级最高) */
    if (s_finishEffectActive)
    {
        LED_FinishEffectUpdate(deltaMs);
        return;  /* 终点效果期间不处理其他逻辑 */
    }

    /* 1. 避障提示灯效 */
    if (s_obstacleEffectActive)
    {
        LED_ObstacleEffectUpdate(deltaMs);
        return;
    }
    
    /* 2. 处理指示灯闪烁 (页面切换用 - E1闪烁两次) */
    if (s_indicatorBlinkCount > 0)
    {
        s_indicatorTimer += deltaMs;
        if (s_indicatorTimer >= 100) /* 100ms 翻转一次 */
        {
            s_indicatorTimer = 0;
            s_indicatorPhase = !s_indicatorPhase;
            s_ledEnabled[LED_INDICATOR] = s_indicatorPhase;
            if (s_indicatorPhase == 0) s_indicatorBlinkCount--;
        }
    }
    else if (s_adjustMode == BRIGHTNESS_MODE_NORMAL)
    {
        /* 正常模式下如果没有闪烁任务，由页面逻辑控制E1，这里默认关闭 */
        // s_ledEnabled[LED_INDICATOR] = 0; 
    }
    
    /* 3. 在调节模式中，确保灯常亮以便观察亮度变化 */
    if (s_adjustMode == BRIGHTNESS_MODE_ADJUST)
    {
        s_ledEnabled[LED_GREEN] = 1;
        s_ledEnabled[LED_RED] = 1;
        s_ledEnabled[LED_INDICATOR] = 1;
    }
}

BrightnessMode_t LED_GetAdjustMode(void)
{
    return s_adjustMode;
}

uint8_t LED_GetSavedBrightness(LED_Index_t led)
{
    /* 用户要求把调整好的亮度传递给后续页面 */
    return s_savedBrightness;
}

/*====================================================================================*/
/*                                  避障提示灯效                                       */
/*====================================================================================*/

void LED_StartObstacleEffect(void)
{
    if (s_finishEffectActive)
    {
        return;
    }

    s_obstacleEffectActive = 1;
    s_obstacleStep = 0;
    s_obstacleRound = 0;
    s_obstacleTimerMs = 0;
    s_indicatorBlinkCount = 0;
    s_indicatorPhase = 0;

    s_ledBrightness[LED_GREEN] = s_savedBrightness;
    s_ledBrightness[LED_RED] = s_savedBrightness;
    s_ledBrightness[LED_INDICATOR] = s_savedBrightness;

    switch (s_obstacleStep)
    {
        case 0:
            s_ledEnabled[LED_GREEN] = 1;
            s_ledEnabled[LED_RED] = 1;
            s_ledEnabled[LED_INDICATOR] = 0;
            break;
        case 1:
            s_ledEnabled[LED_GREEN] = 1;
            s_ledEnabled[LED_RED] = 0;
            s_ledEnabled[LED_INDICATOR] = 1;
            break;
        default:
            s_ledEnabled[LED_GREEN] = 0;
            s_ledEnabled[LED_RED] = 1;
            s_ledEnabled[LED_INDICATOR] = 1;
            break;
    }
}

uint8_t LED_IsObstacleEffectPlaying(void)
{
    return s_obstacleEffectActive;
}

void LED_BindUIRefresh(void (*refreshFn)(void))
{
    s_uiRefreshHook = refreshFn;
}

/*====================================================================================*/
/*                                  终点流水灯效果                                     */
/*====================================================================================*/

/**
  * @brief  启动终点流水灯效果
  */
void LED_StartFinishEffect(void)
{
    s_finishEffectActive = 1;
    s_obstacleEffectActive = 0;
    s_finishRound = 0;
    s_finishLedIdx = 0;
    s_finishPhase = 0;      /* 从亮开始 */
    s_finishTimer = 0;
    
    /* 关闭所有LED */
    s_ledEnabled[LED_GREEN] = 0;
    s_ledEnabled[LED_RED] = 0;
    s_ledEnabled[LED_INDICATOR] = 0;
    
    /* 第一个LED直接100%亮度点亮 */
    s_ledBrightness[LED_GREEN] = 100;
    s_ledBrightness[LED_RED] = 100;
    s_ledBrightness[LED_INDICATOR] = 100;
    s_ledEnabled[LED_GREEN] = 1;
}

/**
  * @brief  检查终点效果是否正在播放
  */
uint8_t LED_IsFinishEffectPlaying(void)
{
    return s_finishEffectActive;
}

/**
  * @brief  终点流水灯效果更新 (在LED_AdjustModeUpdate中调用)
  * @note   每个LED直接亮灭闪烁，按顺序执行，共2轮
  *         适配50ms主循环调用周期
  */
static void LED_FinishEffectUpdate(uint32_t deltaMs)
{
    (void)deltaMs;  /* 不使用deltaMs，直接按调用次数计算 */
    
    if (!s_finishEffectActive) return;
    
    s_finishTimer++;
    
    if (s_finishPhase == 0)
    {
        /* 亮的阶段 */
        if (s_finishTimer >= FINISH_ON_CALLS)
        {
            s_finishTimer = 0;
            s_finishPhase = 1;
            
            /* 关闭当前LED */
            s_ledEnabled[s_finishLedIdx] = 0;
        }
    }
    else
    {
        /* 灭的阶段 */
        if (s_finishTimer >= FINISH_OFF_CALLS)
        {
            s_finishTimer = 0;
            s_finishPhase = 0;
            
            /* 切换到下一个LED */
            s_finishLedIdx++;
            
            if (s_finishLedIdx >= LED_COUNT)
            {
                /* 当前轮次完成，进入下一轮 */
                s_finishRound++;
                
                if (s_finishRound >= FINISH_ROUNDS)
                {
                    /* 所有轮次完成，结束效果 */
                    s_finishEffectActive = 0;
                    s_ledEnabled[LED_GREEN] = 0;
                    s_ledEnabled[LED_RED] = 0;
                    s_ledEnabled[LED_INDICATOR] = 0;
                    return;
                }
                
                /* 重置为第一个LED */
                s_finishLedIdx = 0;
            }
            
            /* 点亮下一个LED (100%亮度) */
            s_ledEnabled[s_finishLedIdx] = 1;
        }
    }
}

static void LED_ObstacleEffectUpdate(uint32_t deltaMs)
{
    if (!s_obstacleEffectActive)
    {
        return;
    }

    s_obstacleTimerMs = (uint16_t)(s_obstacleTimerMs + deltaMs);
    if (s_obstacleTimerMs < OBSTACLE_STEP_MS)
    {
        return;
    }

    s_obstacleTimerMs = 0;
    s_obstacleStep++;
    if (s_obstacleStep >= 3U)
    {
        s_obstacleStep = 0;
        s_obstacleRound++;
    }

    s_ledBrightness[LED_GREEN] = s_savedBrightness;
    s_ledBrightness[LED_RED] = s_savedBrightness;
    s_ledBrightness[LED_INDICATOR] = s_savedBrightness;

    if (s_obstacleRound >= OBSTACLE_ROUNDS)
    {
        s_obstacleEffectActive = 0;
        if (s_uiRefreshHook)
        {
            s_uiRefreshHook();
        }
        return;
    }

    switch (s_obstacleStep)
    {
        case 0:
            s_ledEnabled[LED_GREEN] = 1;
            s_ledEnabled[LED_RED] = 1;
            s_ledEnabled[LED_INDICATOR] = 0;
            break;
        case 1:
            s_ledEnabled[LED_GREEN] = 1;
            s_ledEnabled[LED_RED] = 0;
            s_ledEnabled[LED_INDICATOR] = 1;
            break;
        default:
            s_ledEnabled[LED_GREEN] = 0;
            s_ledEnabled[LED_RED] = 1;
            s_ledEnabled[LED_INDICATOR] = 1;
            break;
    }
}
