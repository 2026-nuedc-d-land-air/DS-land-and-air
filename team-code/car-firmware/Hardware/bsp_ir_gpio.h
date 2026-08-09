/**
 * @file    bsp_ir_gpio.h
 * @brief   8路红外循迹 GPIO 直接读取驱动
 * @note    使用 PF1-PF4, PF6-PF9 (跳过 PF5)
 *          读取时间 < 1μs，比 I2C 快 1000 倍！
 */

#ifndef __BSP_IR_GPIO_H
#define __BSP_IR_GPIO_H

#include "stm32f10x.h"

/*====================================================================================*/
/*                              引脚定义                                               */
/*====================================================================================*/

/* 使用 GPIOF */
#define IR_GPIO_PORT        GPIOF
#define IR_GPIO_CLK         RCC_APB2Periph_GPIOF

/* 8路传感器引脚 (从左到右: L1-L8) */
/* 注意：跳过 PF5 */
#define IR_PIN_L1           GPIO_Pin_1      /* PF1 - 最左 */
#define IR_PIN_L2           GPIO_Pin_2      /* PF2 */
#define IR_PIN_L3           GPIO_Pin_3      /* PF3 */
#define IR_PIN_L4           GPIO_Pin_4      /* PF4 */
#define IR_PIN_L5           GPIO_Pin_6      /* PF6 - 跳过PF5 */
#define IR_PIN_L6           GPIO_Pin_7      /* PF7 */
#define IR_PIN_L7           GPIO_Pin_8      /* PF8 */
#define IR_PIN_L8           GPIO_Pin_9      /* PF9 - 最右 */

/* 所有引脚掩码 */
#define IR_ALL_PINS         (IR_PIN_L1 | IR_PIN_L2 | IR_PIN_L3 | IR_PIN_L4 | \
                             IR_PIN_L5 | IR_PIN_L6 | IR_PIN_L7 | IR_PIN_L8)

/*====================================================================================*/
/*                              函数声明                                               */
/*====================================================================================*/

/**
 * @brief  初始化 GPIO 循迹模块
 */
void IR_GPIO_Init(void);

/**
 * @brief  读取 8 路循迹传感器
 * @return 8 位数据，每位代表一个传感器
 *         bit7=L1(最左), bit0=L8(最右)
 *         0=检测到黑线, 1=白色/无线
 * @note   读取时间 < 1μs
 */
uint8_t IR_GPIO_Read(void);

/**
 * @brief  读取单个传感器
 * @param  index: 0-7 (L1-L8)
 * @return 0=检测到黑线, 1=白色
 */
uint8_t IR_GPIO_ReadSingle(uint8_t index);

#endif /* __BSP_IR_GPIO_H */
