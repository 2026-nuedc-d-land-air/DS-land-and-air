/**
  * @file    bsp_key.c
  * @brief   按键驱动 (PC5)
  * @note    支持短按/长按检测，非阻塞式状态机实现
  *          外部上拉+电容消抖，按下接地
  *          使用 bsp_systick 的 TIM2 计时器获取精准时间
  */

#include "bsp_key.h"
#include "bsp_systick.h"
#include "Delay.h"

/*====================================================================================*/
/*                                  私有变量                                           */
/*====================================================================================*/

static KeyState_t s_keyState = KEY_STATE_IDLE;  // 按键状态机
static uint32_t s_pressStartTime = 0;           // 按下开始时间 (使用SysTick_GetMs)

/*====================================================================================*/
/*                                  初始化函数                                         */
/*====================================================================================*/

/**
  * @brief  按键初始化
  * @note   配置PC5为上拉输入
  */
void Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* 使能GPIOC时钟 */
    RCC_APB2PeriphClockCmd(KEY_GPIO_CLK, ENABLE);
    
    /* 配置PC5为上拉输入 */
    GPIO_InitStructure.GPIO_Pin = KEY_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   // 内部上拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(KEY_GPIO_PORT, &GPIO_InitStructure);
    
    /* 初始化状态 */
    s_keyState = KEY_STATE_IDLE;
    s_pressStartTime = 0;
}

/*====================================================================================*/
/*                                  按键扫描                                           */
/*====================================================================================*/

/**
  * @brief  获取按键当前电平状态
  * @retval 0:按下(低电平), 1:释放(高电平)
  */
uint8_t Key_GetLevel(void)
{
    return GPIO_ReadInputDataBit(KEY_GPIO_PORT, KEY_GPIO_PIN);
}

/**
  * @brief  带时间参数的按键扫描 (使用SysTick精准计时)
  * @param  deltaMs: 未使用，保留兼容性
  * @retval 按键事件
  * @note   使用 l 等级判断: l > 4 为长按，否则为短按
  */
KeyEvent_t Key_ScanWithTime(uint32_t deltaMs)
{
    KeyEvent_t event = KEY_EVENT_NONE;
    uint8_t keyLevel = Key_GetLevel();
    uint32_t currentTime = SysTick_GetMs();  /* 使用TIM2精准计时器 */
    uint8_t lpLevel;
    
    (void)deltaMs;  /* 不再使用deltaMs累加，直接用SysTick */
    
    switch (s_keyState)
    {
        case KEY_STATE_IDLE:
            /* 空闲状态：检测按下 */
            if (keyLevel == 0)  // 按下(低电平)
            {
                s_keyState = KEY_STATE_DEBOUNCE;
                s_pressStartTime = currentTime;
            }
            break;
            
        case KEY_STATE_DEBOUNCE:
            /* 消抖状态：等待消抖时间 */
            if (currentTime - s_pressStartTime >= KEY_DEBOUNCE_TIME)
            {
                if (keyLevel == 0)  // 仍然按下
                {
                    s_keyState = KEY_STATE_PRESSED;
                    s_pressStartTime = currentTime;
                }
                else  // 抖动，回到空闲
                {
                    s_keyState = KEY_STATE_IDLE;
                }
            }
            break;
            
        case KEY_STATE_PRESSED:
        case KEY_STATE_LONG_WAIT:
            /* 按下状态：等待释放，松手时根据 l 等级判断 */
            if (keyLevel == 1)  // 释放
            {
                /* 计算 l 等级 (1-9) */
                lpLevel = Key_GetLongPressLevel();
                
                /* l > 4 为长按，否则为短按 */
                if (lpLevel > 4)
                {
                    event = KEY_EVENT_LONG;
                }
                else
                {
                    event = KEY_EVENT_SHORT;
                }
                s_keyState = KEY_STATE_IDLE;
            }
            else
            {
                /* 仍然按下，更新状态用于显示 l 等级 */
                if (currentTime - s_pressStartTime >= KEY_LONG_TIME)
                {
                    s_keyState = KEY_STATE_LONG_WAIT;
                }
            }
            break;
            
        default:
            s_keyState = KEY_STATE_IDLE;
            break;
    }
    
    return event;
}

/**
  * @brief  按键扫描（非阻塞状态机）- 使用默认时间间隔
  * @retval 按键事件
  */
KeyEvent_t Key_Scan(void)
{
    return Key_ScanWithTime(50);  // 默认50ms
}

/**
  * @brief  按键状态机复位
  */
void Key_Reset(void)
{
    s_keyState = KEY_STATE_IDLE;
    s_pressStartTime = 0;
}

/**
  * @brief  获取当前状态机状态
  */
uint8_t Key_GetState(void)
{
    return (uint8_t)s_keyState;
}

/**
  * @brief  获取按键按下持续时间 (使用SysTick精准计时)
  * @retval 按下时间(ms)，未按下返回0
  */
uint32_t Key_GetPressTime(void)
{
    if (s_keyState == KEY_STATE_PRESSED || s_keyState == KEY_STATE_LONG_WAIT)
    {
        return SysTick_GetMs() - s_pressStartTime;
    }
    return 0;
}

/**
  * @brief  获取长按计时等级 (1-9) (使用TIM2精准计时)
  * @retval 0=未按下, 1-9=按下等级 (l > 4 判定为长按)
  */
uint8_t Key_GetLongPressLevel(void)
{
    uint32_t pressTime;
    uint8_t level;
    
    /* 按下或长按等待状态都返回等级 */
    if (s_keyState != KEY_STATE_PRESSED && s_keyState != KEY_STATE_LONG_WAIT)
    {
        return 0;
    }
    
    pressTime = SysTick_GetMs() - s_pressStartTime;
    
    /* 每800ms增加一级，从1开始，最大9 */
    level = (uint8_t)(pressTime / KEY_LONG_TIME);
    if (level < 1) level = 1;
    if (level > 9) level = 9;
    
    return level;
}
