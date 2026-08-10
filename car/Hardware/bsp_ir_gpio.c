/**
 * @file    bsp_ir_gpio.c
 * @brief   8路红外循迹 GPIO 直接读取驱动
 * @note    读取时间 < 1μs，比 I2C 快 1000 倍！
 */

#include "bsp_ir_gpio.h"

/*====================================================================================*/
/*                              引脚映射表                                             */
/*====================================================================================*/

/* 引脚到位置的映射 (用于快速读取) */
static const uint16_t s_pinMap[8] = {
    IR_PIN_L1,  /* bit7 - 最左 */
    IR_PIN_L2,  /* bit6 */
    IR_PIN_L3,  /* bit5 */
    IR_PIN_L4,  /* bit4 */
    IR_PIN_L5,  /* bit3 */
    IR_PIN_L6,  /* bit2 */
    IR_PIN_L7,  /* bit1 */
    IR_PIN_L8   /* bit0 - 最右 */
};

/*====================================================================================*/
/*                              初始化函数                                             */
/*====================================================================================*/

void IR_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* 使能 GPIOF 时钟 */
    RCC_APB2PeriphClockCmd(IR_GPIO_CLK, ENABLE);
    
    /* 配置为上拉输入 */
    GPIO_InitStructure.GPIO_Pin = IR_ALL_PINS;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  /* 上拉输入 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(IR_GPIO_PORT, &GPIO_InitStructure);
}

/*====================================================================================*/
/*                              读取函数                                               */
/*====================================================================================*/

/**
 * @brief  读取 8 路循迹传感器 (超快速版本)
 * @return 8 位数据
 *         bit7=L1(最左), bit0=L8(最右)
 *         0=检测到黑线, 1=白色/无线
 * @note   直接读取 IDR 寄存器，时间 < 1μs
 */
uint8_t IR_GPIO_Read(void)
{
    uint16_t portData;
    uint8_t result = 0;
    
    /* 一次性读取整个端口 */
    portData = IR_GPIO_PORT->IDR;
    
    /* 提取各个引脚状态并组合成 8 位 */
    /* PF1-PF4 对应 bit7-bit4 */
    if (portData & GPIO_Pin_1) result |= 0x80;  /* L1 -> bit7 */
    if (portData & GPIO_Pin_2) result |= 0x40;  /* L2 -> bit6 */
    if (portData & GPIO_Pin_3) result |= 0x20;  /* L3 -> bit5 */
    if (portData & GPIO_Pin_4) result |= 0x10;  /* L4 -> bit4 */
    
    /* PF6-PF9 对应 bit3-bit0 (跳过 PF5) */
    if (portData & GPIO_Pin_6) result |= 0x08;  /* L5 -> bit3 */
    if (portData & GPIO_Pin_7) result |= 0x04;  /* L6 -> bit2 */
    if (portData & GPIO_Pin_8) result |= 0x02;  /* L7 -> bit1 */
    if (portData & GPIO_Pin_9) result |= 0x01;  /* L8 -> bit0 */
    
    return result;
}

/**
 * @brief  读取单个传感器
 * @param  index: 0-7 (L1-L8)
 * @return 0=检测到黑线, 1=白色
 */
uint8_t IR_GPIO_ReadSingle(uint8_t index)
{
    if (index >= 8) return 1;
    
    return (IR_GPIO_PORT->IDR & s_pinMap[index]) ? 1 : 0;
}
