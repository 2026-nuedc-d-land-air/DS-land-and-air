/**
  * @file    bsp_led.h
  * @brief   LED驱动模块
  * @note    LED1G (绿色) - PB9, LED2R (红色) - PE0
  *          低电平点亮 (LED连接到VCC)
  */

#ifndef __BSP_LED_H
#define __BSP_LED_H

#include "stm32f10x.h"

/*====================================================================================*/
/*                                  引脚定义                                           */
/*====================================================================================*/

/* LED1 绿色 - PB9 */
#define LED1_GPIO_PORT      GPIOB
#define LED1_GPIO_PIN       GPIO_Pin_9
#define LED1_GPIO_CLK       RCC_APB2Periph_GPIOB

/* LED2 红色 - PE0 (根据电路图修正) */
#define LED2_GPIO_PORT      GPIOE
#define LED2_GPIO_PIN      GPIO_Pin_0
#define LED2_GPIO_CLK       RCC_APB2Periph_GPIOE

/*====================================================================================*/
/*                                  宏定义                                             */
/*====================================================================================*/

/* 低电平点亮 */
#define LED1_ON()       GPIO_ResetBits(LED1_GPIO_PORT, LED1_GPIO_PIN)
#define LED1_OFF()      GPIO_SetBits(LED1_GPIO_PORT, LED1_GPIO_PIN)
#define LED1_TOGGLE()   (LED1_GPIO_PORT->ODR ^= LED1_GPIO_PIN)

#define LED2_ON()       GPIO_ResetBits(LED2_GPIO_PORT, LED2_GPIO_PIN)
#define LED2_OFF()      GPIO_SetBits(LED2_GPIO_PORT, LED2_GPIO_PIN)
#define LED2_TOGGLE()   (LED2_GPIO_PORT->ODR ^= LED2_GPIO_PIN)

/*====================================================================================*/
/*                                  函数声明                                           */
/*====================================================================================*/

/**
  * @brief  LED初始化
  * @note   配置PB9和PE0为推挽输出，默认关闭LED
  */
void LED_Init(void);

/**
  * @brief  LED1 绿色控制
  * @param  state: 0-关闭, 1-点亮
  */
void LED1_Set(uint8_t state);

/**
  * @brief  LED2 红色控制
  * @param  state: 0-关闭, 1-点亮
  */
void LED2_Set(uint8_t state);

/**
  * @brief  所有LED控制
  * @param  state: 0-全部关闭, 1-全部点亮
  */
void LED_SetAll(uint8_t state);

/**
  * @brief  获取LED1状态
  * @retval 0-关闭, 1-点亮
  */
uint8_t LED1_GetState(void);

/**
  * @brief  获取LED2状态
  * @retval 0-关闭, 1-点亮
  */
uint8_t LED2_GetState(void);

#endif /* __BSP_LED_H */
