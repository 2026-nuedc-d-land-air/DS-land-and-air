/**
  * @file    bsp_systick.c
  * @brief   系统滴答定时器 - 精准计时模块
  * @note    使用 TIM1 实现 1ms 中断，提供精准的时间基准
  *          
  *          时钟配置：
  *          - 系统时钟: 72MHz (假设使用外部8MHz晶振 + PLL)
  *          - APB2时钟: 72MHz
  *          - TIM1时钟: 72MHz (APB2预分频=1，定时器时钟不倍频)
  *          - 定时器配置: 72MHz / 72 / 1000 = 1ms
  */

#include "bsp_systick.h"
#include "bsp_key2.h"

/*====================================================================================*/
/*                                  全局变量                                           */
/*====================================================================================*/

volatile SysTick_Time_t g_sysTime = {0, 0, 0, 0, 0};

/*====================================================================================*/
/*                                  初始化函数                                         */
/*====================================================================================*/

/**
  * @brief  初始化系统滴答定时器 (TIM1, 1ms中断)
  */
void SysTick_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    /* 使能 TIM1 时钟 (APB2) */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    
    /* 定时器配置: 1ms 中断 */
    /* TIM1 时钟 = 72MHz (APB2=72MHz) */
    /* 计数频率 = 72MHz / (PSC+1) = 72MHz / 72 = 1MHz */
    /* 中断周期 = 1MHz / ARR = 1MHz / 1000 = 1kHz = 1ms */
    TIM_TimeBaseStructure.TIM_Period = 1000 - 1;        /* 自动重装载值 */
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;       /* 预分频器 */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;    /* TIM1高级定时器特有 */
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    
    /* 清除更新中断标志 */
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    
    /* 使能更新中断 */
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    
    /* 配置 NVIC - TIM1更新中断 */
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  /* 最高优先级 */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /* 使能定时器 */
    TIM_Cmd(TIM1, ENABLE);
}

/*====================================================================================*/
/*                                  中断服务函数                                       */
/*====================================================================================*/

/**
  * @brief  TIM1 更新中断服务函数 - 每1ms执行一次
  */
void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        
        /* 更新总毫秒数 */
        g_sysTime.totalMs++;
        
        /* 更新毫秒 */
        g_sysTime.milliseconds++;
        
        if (g_sysTime.milliseconds >= 1000)
        {
            g_sysTime.milliseconds = 0;
            g_sysTime.seconds++;
            
            if (g_sysTime.seconds >= 60)
            {
                g_sysTime.seconds = 0;
                g_sysTime.minutes++;
                
                if (g_sysTime.minutes >= 60)
                {
                    g_sysTime.minutes = 0;
                    g_sysTime.hours++;
                }
            }
        }
        
        /* KEY2 按键扫描 (每1ms调用) */
        Key2_Scan();
    }
}

/*====================================================================================*/
/*                                  功能函数                                           */
/*====================================================================================*/

/**
  * @brief  获取当前毫秒数
  * @return 从启动到现在的总毫秒数
  */
uint32_t SysTick_GetMs(void)
{
    return g_sysTime.totalMs;
}

/**
  * @brief  获取格式化的时间字符串
  * @param  buf: 输出缓冲区 (至少6字节: "MM:SS\0")
  */
void SysTick_GetTimeString(char *buf)
{
    uint32_t min = g_sysTime.minutes;
    uint32_t sec = g_sysTime.seconds;
    
    /* 格式化为 "MM:SS" */
    buf[0] = '0' + (min / 10) % 10;
    buf[1] = '0' + min % 10;
    buf[2] = ':';
    buf[3] = '0' + sec / 10;
    buf[4] = '0' + sec % 10;
    buf[5] = '\0';
}

/**
  * @brief  重置计时器
  */
void SysTick_Reset(void)
{
    /* 禁用中断，防止竞态条件 */
    __disable_irq();
    
    g_sysTime.milliseconds = 0;
    g_sysTime.seconds = 0;
    g_sysTime.minutes = 0;
    g_sysTime.hours = 0;
    g_sysTime.totalMs = 0;
    
    __enable_irq();
}
