/**
  * @file    bsp_key2.c
  * @brief   PC4按键驱动实现 (亮度调节按键)
  */

#include "bsp_key2.h"

/*====================================================================================*/
/*                                  私有变量                                           */
/*====================================================================================*/

static uint8_t s_keyState = 0;          /* 当前按键状态 */
static uint8_t s_lastKeyState = 0;      /* 上次按键状态 */
static uint16_t s_debounceCounter = 0;  /* 消抖计数器 */
static uint16_t s_pressCounter = 0;     /* 按下时间计数器 */
static Key2Event_t s_keyEvent = KEY2_EVENT_NONE;
static uint8_t s_longPressTriggered = 0;    /* 长按已触发标志 */

/*====================================================================================*/
/*                                  函数实现                                           */
/*====================================================================================*/

/**
  * @brief  KEY2初始化
  */
void Key2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* 使能GPIO时钟 */
    RCC_APB2PeriphClockCmd(KEY2_GPIO_CLK, ENABLE);
    
    /* 配置为上拉输入 */
    GPIO_InitStructure.GPIO_Pin = KEY2_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(KEY2_GPIO_PORT, &GPIO_InitStructure);
    
    /* 初始化状态 */
    s_keyState = 0;
    s_lastKeyState = 0;
    s_debounceCounter = 0;
    s_pressCounter = 0;
    s_keyEvent = KEY2_EVENT_NONE;
    s_longPressTriggered = 0;
}

/**	
  * @brief  读取按键原始状态
  */
static uint8_t Key2_ReadRaw(void)
{
    if (GPIO_ReadInputDataBit(KEY2_GPIO_PORT, KEY2_GPIO_PIN) == KEY2_PRESSED_LEVEL)
    {
        return 1;   /* 按下 */
    }
    return 0;       /* 未按下 */
}

/**
  * @brief  KEY2扫描 (需要周期性调用, 建议1ms)
  */
void Key2_Scan(void)
{
    uint8_t rawState = Key2_ReadRaw();
    
    /* 消抖处理 */
    if (rawState != s_lastKeyState)
    {
        s_debounceCounter = 0;
        s_lastKeyState = rawState;
    }
    else
    {
        if (s_debounceCounter < KEY2_DEBOUNCE_MS)
        {
            s_debounceCounter++;
            if (s_debounceCounter >= KEY2_DEBOUNCE_MS)
            {
                /* 消抖完成, 更新状态 */
                if (rawState != s_keyState)
                {
                    if (rawState)
                    {
                        /* 按下 */
                        s_keyState = 1;
                        s_pressCounter = 0;
                        s_longPressTriggered = 0;
                    }
                    else
                    {
                        /* 释放 */
                        s_keyState = 0;
                        
                        /* 判断短按 (未触发长按时) */
                        if (!s_longPressTriggered && s_pressCounter < KEY2_LONG_PRESS_MS)
                        {
                            s_keyEvent = KEY2_EVENT_SHORT;
                        }
                        
                        s_pressCounter = 0;
                    }
                }
            }
        }
    }
    
    /* 按下计时 */
    if (s_keyState)
    {
        if (s_pressCounter < 0xFFFF)
        {
            s_pressCounter++;
        }
        
        /* 长按检测 */
        if (!s_longPressTriggered && s_pressCounter >= KEY2_LONG_PRESS_MS)
        {
            s_longPressTriggered = 1;
            s_keyEvent = KEY2_EVENT_LONG;
        }
    }
}

/**
  * @brief  获取KEY2事件
  */
Key2Event_t Key2_GetEvent(void)
{
    return s_keyEvent;
}

/**
  * @brief  清除KEY2事件
  */
void Key2_ClearEvent(void)
{
    s_keyEvent = KEY2_EVENT_NONE;
}

/**
  * @brief  检查KEY2是否按下
  */
uint8_t Key2_IsPressed(void)
{
    return s_keyState;
}
