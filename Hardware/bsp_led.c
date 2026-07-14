/**
  * @file    bsp_led.c
  * @brief   LED驱动模块实现
  * @note    LED1G (绿色) - PB9, LED2R (红色) - PE0
  *          低电平点亮 (LED连接到VCC)
  */

#include "bsp_led.h"

/**
  * @brief  LED初始化
  * @note   配置PB9和PE0为推挽输出，默认关闭LED
  */
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* 使能GPIO时钟 */
    RCC_APB2PeriphClockCmd(LED1_GPIO_CLK | LED2_GPIO_CLK, ENABLE);
    
    /* 配置LED1 - PB9 */
    GPIO_InitStructure.GPIO_Pin = LED1_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    /* 推挽输出 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);
    
    /* 配置LED2 - PE0 */
    GPIO_InitStructure.GPIO_Pin = LED2_GPIO_PIN;
    GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure);
    
    /* 默认关闭LED (输出高电平) */
    LED1_OFF();
    LED2_OFF();
}

/**
  * @brief  LED1 绿色控制
  * @param  state: 0-关闭, 1-点亮
  */
void LED1_Set(uint8_t state)
{
    if (state)
    {
        LED1_ON();
    }
    else
    {
        LED1_OFF();
    }
}

/**
  * @brief  LED2 红色控制
  * @param  state: 0-关闭, 1-点亮
  */
void LED2_Set(uint8_t state)
{
    if (state)
    {
        LED2_ON();
    }
    else
    {
        LED2_OFF();
    }
}

/**
  * @brief  所有LED控制
  * @param  state: 0-全部关闭, 1-全部点亮
  */
void LED_SetAll(uint8_t state)
{
    LED1_Set(state);
    LED2_Set(state);
}

/**
  * @brief  获取LED1状态
  * @retval 0-关闭, 1-点亮
  * @note   低电平点亮，所以读取到0表示亮
  */
uint8_t LED1_GetState(void)
{
    return (GPIO_ReadOutputDataBit(LED1_GPIO_PORT, LED1_GPIO_PIN) == Bit_RESET) ? 1 : 0;
}

/**
  * @brief  获取LED2状态
  * @retval 0-关闭, 1-点亮
  * @note   低电平点亮，所以读取到0表示亮
  */
uint8_t LED2_GetState(void)
{
    return (GPIO_ReadOutputDataBit(LED2_GPIO_PORT, LED2_GPIO_PIN) == Bit_RESET) ? 1 : 0;
}
