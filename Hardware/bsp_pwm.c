/**
  * @file    bsp_pwm.c
  * @brief   PWM管理模块实现
  * @note    使用TIM4产生PWM信号
  *          PB8 - TIM4_CH3
  */

#include "bsp_pwm.h"

/*====================================================================================*/
/*                                  私有变量                                           */
/*====================================================================================*/

static uint16_t s_pwmPeriod = 0;    /* PWM周期值 */
static uint8_t s_currentDuty = 0;   /* 当前占空比 */

/*====================================================================================*/
/*                                  初始化函数                                         */
/*====================================================================================*/

/**
  * @brief  PWM初始化 (TIM4_CH3 - PB8)
  * @param  freq: PWM频率 (Hz)
  */
void PWM_Init(uint16_t freq)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    /* 使能时钟 */
    RCC_APB2PeriphClockCmd(PWM_BUZZER_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(PWM_BUZZER_TIM_CLK, ENABLE);
    
    /* 配置GPIO - PB8 复用推挽输出 */
    GPIO_InitStructure.GPIO_Pin = PWM_BUZZER_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PWM_BUZZER_GPIO_PORT, &GPIO_InitStructure);
    
    /* 计算周期值: 72MHz / freq */
    /* 使用预分频72，计数频率1MHz */
    s_pwmPeriod = 1000000 / freq;
    
    /* TIM4时基配置 */
    TIM_TimeBaseStructure.TIM_Period = s_pwmPeriod - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;       /* 72MHz / 72 = 1MHz */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(PWM_BUZZER_TIM, &TIM_TimeBaseStructure);
    
    /* TIM4_CH3 PWM模式配置 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;                  /* 初始占空比0 */
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;  /* 低电平有效 */
    TIM_OC3Init(PWM_BUZZER_TIM, &TIM_OCInitStructure);
    
    /* 使能预装载 */
    TIM_OC3PreloadConfig(PWM_BUZZER_TIM, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(PWM_BUZZER_TIM, ENABLE);
    
    /* 使能TIM4 */
    TIM_Cmd(PWM_BUZZER_TIM, ENABLE);
    
    /* 初始状态：关闭 */
    s_currentDuty = 0;
    PWM_SetDuty(0);
}

/*====================================================================================*/
/*                                  控制函数                                           */
/*====================================================================================*/

/**
  * @brief  设置PWM占空比
  * @param  duty: 占空比 (0-100)
  */
void PWM_SetDuty(uint8_t duty)
{
    uint16_t pulse;
    
    if (duty > 100) duty = 100;
    s_currentDuty = duty;
    
    /* 计算脉冲宽度 */
    pulse = (uint16_t)((uint32_t)s_pwmPeriod * duty / 100);
    
    /* 设置比较值 */
    TIM_SetCompare3(PWM_BUZZER_TIM, pulse);
}

/**
  * @brief  设置PWM频率
  * @param  freq: 频率 (Hz)
  */
void PWM_SetFreq(uint16_t freq)
{
    if (freq < 100) freq = 100;
    if (freq > 20000) freq = 20000;
    
    /* 重新计算周期值 */
    s_pwmPeriod = 1000000 / freq;
    
    /* 更新自动重装载值 */
    TIM_SetAutoreload(PWM_BUZZER_TIM, s_pwmPeriod - 1);
    
    /* 重新设置占空比 */
    PWM_SetDuty(s_currentDuty);
}

/**
  * @brief  启动PWM输出
  */
void PWM_Start(void)
{
    TIM_Cmd(PWM_BUZZER_TIM, ENABLE);
}

/**
  * @brief  停止PWM输出
  */
void PWM_Stop(void)
{
    PWM_SetDuty(0);
}

/**
  * @brief  获取当前占空比
  * @retval 占空比 (0-100)
  */
uint8_t PWM_GetDuty(void)
{
    return s_currentDuty;
}
