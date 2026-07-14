/**
  * @file    HW_I2C.c
  * @brief   STM32F10x 硬件I2C2驱动 (标准库版本)
  * @note    用于循迹模块，PB10(SCL), PB11(SDA)
  *          可与软件I2C(MyI2C)动态切换使用
  */

#include "stm32f10x.h"
#include "HW_I2C.h"
#include "Delay.h"

/**
 * @brief I2C通信超时时间定义
 * @details 用于设置I2C总线通信的最大等待时间，
 *          防止在通信异常时程序陷入死循环
 */
/**
 * @file HW_I2C.c
 * @brief 硬件I2C驱动程序
 * 
 * @details 本文件实现了STM32的硬件I2C通信功能，包括：
 *          - I2C总线初始化配置
 *          - I2C主机模式下的数据发送与接收
 *          - I2C从机地址寻址
 *          - I2C通信超时处理机制
 *          - I2C错误检测与恢复
 * 
 * @note 设计思路：
 *       1. 采用硬件I2C外设，相比软件模拟I2C具有更高的通信效率
 *       2. 实现超时机制防止I2C总线死锁
 *       3. 提供统一的读写接口，便于上层应用调用
 *       4. 支持7位从机地址模式
 * 
 * @author [作者]
 * @version 1.0
 * @date [日期]
 */
/* I2C超时时间 */
#define I2C_TIMEOUT     10000

/* 硬件I2C是否已初始化标志 */
static uint8_t HW_I2C_Initialized = 0;

/**
  * @brief  初始化硬件I2C2
  * @note   PB10-SCL, PB11-SDA, 400kHz
  */
void HW_I2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;
    
    /* 使能时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    
    /* 配置PB10(SCL), PB11(SDA)为复用开漏输出 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;     // 复用开漏
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* I2C2配置 */
    I2C_DeInit(I2C2);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;           // 主机地址(不重要)
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 400000;          // 400kHz
    I2C_Init(I2C2, &I2C_InitStructure);
    
    /* 使能I2C2 */
    I2C_Cmd(I2C2, ENABLE);
    
    HW_I2C_Initialized = 1;
}

/**
  * @brief  关闭硬件I2C2，释放GPIO给软件I2C使用
  */
void HW_I2C_DeInit(void)
{
    /* 关闭I2C2 */
    I2C_Cmd(I2C2, DISABLE);
    I2C_DeInit(I2C2);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, DISABLE);
    
    HW_I2C_Initialized = 0;
}

/**
  * @brief  检查I2C事件，带超时
  * @param  I2C_EVENT: 要检查的事件
  * @retval 0:成功, 1:超时
  */
static uint8_t HW_I2C_WaitEvent(uint32_t I2C_EVENT)
{
    uint32_t timeout = I2C_TIMEOUT;
    while (I2C_CheckEvent(I2C2, I2C_EVENT) != SUCCESS)
    {
        if (--timeout == 0)
        {
            return 1;  // 超时
        }
    }
    return 0;
}

/**
  * @brief  检查标志位，带超时
  * @param  I2C_FLAG: 要检查的标志
  * @retval 0:成功, 1:超时
  */
static uint8_t HW_I2C_WaitFlag(uint32_t I2C_FLAG)
{
    uint32_t timeout = I2C_TIMEOUT;
    while (I2C_GetFlagStatus(I2C2, I2C_FLAG) == SET)
    {
        if (--timeout == 0)
        {
            return 1;  // 超时
        }
    }
    return 0;
}

/**
  * @brief  硬件I2C读取一个字节（带寄存器地址）
  * @param  SlaveAddr: 从机地址(7位，不含读写位)
  * @param  RegAddr: 寄存器地址
  * @param  Data: 读取的数据存放地址
  * @retval 0:成功, 非0:失败
  */
uint8_t HW_I2C_ReadByte(uint8_t SlaveAddr, uint8_t RegAddr, uint8_t *Data)
{
    /* 确保I2C已初始化 */
    if (!HW_I2C_Initialized)
    {
        HW_I2C_Init();
    }
    
    /* 等待总线空闲 */
    if (HW_I2C_WaitFlag(I2C_FLAG_BUSY)) return 1;
    
    /* 发送起始条件 */
    I2C_GenerateSTART(I2C2, ENABLE);
    if (HW_I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return 2;
    
    /* 发送从机地址(写) */
    I2C_Send7bitAddress(I2C2, SlaveAddr << 1, I2C_Direction_Transmitter);
    if (HW_I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) return 3;
    
    /* 发送寄存器地址 */
    I2C_SendData(I2C2, RegAddr);
    if (HW_I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return 4;
    
    /* 重新发送起始条件 */
    I2C_GenerateSTART(I2C2, ENABLE);
    if (HW_I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return 5;
    
    /* 发送从机地址(读) */
    I2C_Send7bitAddress(I2C2, SlaveAddr << 1, I2C_Direction_Receiver);
    if (HW_I2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) return 6;
    
    /* 关闭应答(只读一个字节) */
    I2C_AcknowledgeConfig(I2C2, DISABLE);
    
    /* 发送停止条件 */
    I2C_GenerateSTOP(I2C2, ENABLE);
    
    /* 等待数据接收完成 */
    if (HW_I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)) return 7;
    
    /* 读取数据 */
    *Data = I2C_ReceiveData(I2C2);
    
    /* 重新使能应答 */
    I2C_AcknowledgeConfig(I2C2, ENABLE);
    
    return 0;
}

/**
  * @brief  硬件I2C写入一个字节（带寄存器地址）
  * @param  SlaveAddr: 从机地址(7位，不含读写位)
  * @param  RegAddr: 寄存器地址
  * @param  Data: 要写入的数据
  * @retval 0:成功, 非0:失败
  */
uint8_t HW_I2C_WriteByte(uint8_t SlaveAddr, uint8_t RegAddr, uint8_t Data)
{
    /* 确保I2C已初始化 */
    if (!HW_I2C_Initialized)
    {
        HW_I2C_Init();
    }
    
    /* 等待总线空闲 */
    if (HW_I2C_WaitFlag(I2C_FLAG_BUSY)) return 1;
    
    /* 发送起始条件 */
    I2C_GenerateSTART(I2C2, ENABLE);
    if (HW_I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return 2;
    
    /* 发送从机地址(写) */
    I2C_Send7bitAddress(I2C2, SlaveAddr << 1, I2C_Direction_Transmitter);
    if (HW_I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) return 3;
    
    /* 发送寄存器地址 */
    I2C_SendData(I2C2, RegAddr);
    if (HW_I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return 4;
    
    /* 发送数据 */
    I2C_SendData(I2C2, Data);
    if (HW_I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return 5;
    
    /* 发送停止条件 */
    I2C_GenerateSTOP(I2C2, ENABLE);
    
    return 0;
}

/**
  * @brief  切换到硬件I2C模式
  * @note   将GPIO配置为复用开漏，重新初始化I2C2外设
  */
void HW_I2C_Enable(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;
    
    /* 使能时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    
    /* 配置PB10(SCL), PB11(SDA)为复用开漏输出 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* 重新初始化I2C2配置 */
    I2C_DeInit(I2C2);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 400000;
    I2C_Init(I2C2, &I2C_InitStructure);
    
    /* 使能I2C2 */
    I2C_Cmd(I2C2, ENABLE);
    
    HW_I2C_Initialized = 1; 
}

/**
  * @brief  切换到软件I2C模式
  * @note   关闭I2C2外设，将GPIO配置为普通开漏
  */
void HW_I2C_Disable(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* 关闭I2C2 */
    I2C_Cmd(I2C2, DISABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, DISABLE);
    
    /* 配置PB10(SCL), PB11(SDA)为普通开漏输出 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* 释放总线 */
    GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);
    
    HW_I2C_Initialized = 0;
}

/**
  * @brief  I2C总线恢复
  * @note   当I2C总线卡死时调用，发送9个时钟脉冲
  */
void HW_I2C_BusRecovery(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    uint8_t i;
    
    /* 先关闭I2C2 */
    I2C_Cmd(I2C2, DISABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, DISABLE);
    
    /* 配置为普通推挽输出 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  // SCL
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;  // SDA
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* 发送9个时钟脉冲 */
    GPIO_SetBits(GPIOB, GPIO_Pin_11);  // SDA高
    for (i = 0; i < 9; i++)
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_10);
        Delay_us(5);
        GPIO_SetBits(GPIOB, GPIO_Pin_10);
        Delay_us(5);
    }
    
    /* 发送停止条件 */
    GPIO_ResetBits(GPIOB, GPIO_Pin_11);  // SDA低
    Delay_us(5);
    GPIO_SetBits(GPIOB, GPIO_Pin_10);    // SCL高
    Delay_us(5);
    GPIO_SetBits(GPIOB, GPIO_Pin_11);    // SDA高
    Delay_us(5);
    
    /* 重新初始化硬件I2C */
    HW_I2C_Init();
}
