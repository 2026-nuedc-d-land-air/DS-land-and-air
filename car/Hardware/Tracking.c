#include "stm32f10x.h"
#include "Tracking.h"
#include "Delay.h"
#include "IOI2C.h"
#include "HW_I2C.h"

/*
 * 循迹模块驱动 - 使用IOI2C底层函数
 * 
 * 硬件连接：
 * SCL: PB10
 * SDA: PB11
 * 
 * 设备信息：
 * I2C地址: 0x12 (7位地址)
 * 
 * 注意：与陀螺仪/OLED共用I2C总线，需要切换I2C驱动
 */

#define TRACKING_ADDR    0x12    // 循迹模块I2C地址（7位）

// 全局变量：保存最新的循迹数据
static uint8_t g_tracking_data = 0xFF;

/*====================================================================================*/
/*                                  初始化函数                                         */
/*====================================================================================*/

/**
  * 函    数：循迹模块初始化
  * 参    数：无
  * 返 回 值：无
  * 说    明：初始化I2C总线，并发送模块初始化命令
  */
void Tracking_Init(void)
{
    // I2C引脚初始化（使用IOI2C的初始化函数）
    IIC_Init();
    
    Delay_ms(100);  // 等待模块上电稳定
    
    // 尝试发送初始化命令（根据雅博姆模块协议）
    IIC_Start();
    IIC_Send_Byte(TRACKING_ADDR << 1);      // 发送写地址
    if (IIC_Wait_Ack() == 0) {
        IIC_Send_Byte(0x00);                // 寄存器地址
        IIC_Wait_Ack();
        IIC_Send_Byte(0x01);                // 启用/复位命令
        IIC_Wait_Ack();
    }
    IIC_Stop();
    
    Delay_ms(10);
}

/*====================================================================================*/
/*                                  读取函数                                           */
/*====================================================================================*/

/**
  * 函    数：直接读取循迹数据
  * 参    数：无
  * 返 回 值：8位循迹数据，每位代表一个传感器状态
  * 说    明：直接读取模式，不指定寄存器地址
  */
uint8_t Tracking_ReadDirect(void)
{
    uint8_t data = 0xFF;
    
    IIC_Start();
    IIC_Send_Byte((TRACKING_ADDR << 1) | 1);    // 发送读地址
    if (IIC_Wait_Ack() == 0) {
        data = IIC_Read_Byte(0);                // 读取数据，发送NACK
    }
    IIC_Stop();
    
    return data;
}

/**
  * 函    数：从指定寄存器读取循迹数据
  * 参    数：reg 寄存器地址
  * 返 回 值：读取到的数据
  * 说    明：先写寄存器地址，再读取数据
  */
uint8_t Tracking_ReadReg(uint8_t reg)
{
    uint8_t data = 0xFF;
    
    // 第一步：发送寄存器地址
    IIC_Start();
    IIC_Send_Byte(TRACKING_ADDR << 1);          // 发送写地址
    if (IIC_Wait_Ack() == 0) {
        IIC_Send_Byte(reg);                     // 发送寄存器地址
        IIC_Wait_Ack();
    }
    IIC_Stop();
    
    Delay_us(10);
    
    // 第二步：读取数据
    IIC_Start();
    IIC_Send_Byte((TRACKING_ADDR << 1) | 1);    // 发送读地址
    if (IIC_Wait_Ack() == 0) {
        data = IIC_Read_Byte(0);                // 读取数据，发送NACK
    }
    IIC_Stop();
    
    return data;
}

/**
  * 函    数：使用高级接口读取循迹数据
  * 参    数：reg 寄存器地址
  * 返 回 值：读取到的数据
  * 说    明：使用IOI2C提供的i2cRead高级函数
  */
uint8_t Tracking_ReadByI2CFunc(uint8_t reg)
{
    uint8_t data = 0xFF;
    i2cRead(TRACKING_ADDR, reg, 1, &data);
    return data;
}

/**
  * 函    数：读取循迹数据（默认接口）
  * 参    数：无
  * 返 回 值：8位循迹数据
  * 说    明：默认使用直接读取模式
  */
uint8_t Tracking_Read(void)
{
    return Tracking_ReadDirect();
}

/**
  * 函    数：带总线切换的循迹数据读取（IOI2C方式）
  * 参    数：无
  * 返 回 值：8位循迹数据
  * 说    明：使用IOI2C读取，适用于与其他设备共用I2C总线的场景
  *           亚博智能循迹模块需要读取寄存器0x30
  */
uint8_t Tracking_ReadWithSwitch(void)
{
    uint8_t data;
    
    // 1. 初始化IOI2C
    IIC_Init();
    Delay_ms(2);  // 等待总线稳定
    
    // 2. 读取循迹数据（从寄存器0x30读取）
    data = Tracking_ReadReg(0x30);
    g_tracking_data = data;
    
    // 3. 释放IOI2C总线
    IIC_ReleaseBus();
    
    return data;
}

/**
  * 函    数：获取最新的循迹数据（不进行I2C通信）
  * 参    数：无
  * 返 回 值：上次读取的8位循迹数据
  */
uint8_t Tracking_GetLastData(void)
{
    return g_tracking_data;
}

/*====================================================================================*/
/*                          硬件I2C读取函数（推荐使用）                                */
/*====================================================================================*/

/**
  * 函    数：使用硬件I2C读取循迹数据
  * 参    数：无
  * 返 回 值：8位循迹数据，每位代表一个传感器状态
  * 说    明：使用STM32硬件I2C2外设，时序精确
  *           从寄存器0x30读取数据（官方协议）
  */
uint8_t Tracking_ReadHW(void)
{
    uint8_t data = 0xFF;
    uint8_t result;
    
    result = HW_I2C_ReadByte(TRACKING_ADDR, 0x30, &data);
    
    if (result == 0) {
        g_tracking_data = data;
    }
    
    return data;
}

/**
  * 函    数：使用硬件I2C读取循迹数据（带总线切换）
  * 参    数：无
  * 返 回 值：8位循迹数据
  * 说    明：先切换到硬件I2C模式，读取后切换回软件I2C模式
  *           适用于与陀螺仪/OLED共用I2C总线的场景
  */
uint8_t Tracking_ReadHW_WithSwitch(void)
{
    uint8_t data = 0xFF;
    uint8_t result;
    
    // 1. 切换到硬件I2C模式
    HW_I2C_Enable();
    Delay_us(100);  // 等待模式切换稳定
    
    // 2. 使用硬件I2C读取数据
    result = HW_I2C_ReadByte(TRACKING_ADDR, 0x30, &data);
    
    if (result == 0) {
        g_tracking_data = data;
    }
    
    // 3. 切换回软件I2C模式（IOI2C）
    HW_I2C_Disable();
    Delay_us(100);  // 等待模式切换稳定
    
    // 4. 释放总线，让其他设备可以使用
    IIC_ReleaseBus();
    
    return data;
}

/**
  * 函    数：解析循迹数据到8个独立变量
  * 参    数：data - 原始8位数据
  *           x1~x8 - 8个传感器状态输出指针
  * 返 回 值：无
  * 说    明：bit7=X1(最左), bit0=X8(最右)
  *           1=检测到黑线, 0=未检测到
  */
void Tracking_ParseData(uint8_t data, uint8_t *x1, uint8_t *x2, uint8_t *x3, uint8_t *x4,
                        uint8_t *x5, uint8_t *x6, uint8_t *x7, uint8_t *x8)
{
    *x1 = (data >> 7) & 0x01;
    *x2 = (data >> 6) & 0x01;
    *x3 = (data >> 5) & 0x01;
    *x4 = (data >> 4) & 0x01;
    *x5 = (data >> 3) & 0x01;
    *x6 = (data >> 2) & 0x01;
    *x7 = (data >> 1) & 0x01;
    *x8 = (data >> 0) & 0x01;
}

/**
  * 函    数：使用硬件I2C读取并解析循迹数据
  * 参    数：x1~x8 - 8个传感器状态输出指针
  * 返 回 值：无
  * 说    明：一站式函数，读取+解析
  */
void Tracking_DealIRData_HW(uint8_t *x1, uint8_t *x2, uint8_t *x3, uint8_t *x4,
                            uint8_t *x5, uint8_t *x6, uint8_t *x7, uint8_t *x8)
{
    uint8_t data = Tracking_ReadHW();
    Tracking_ParseData(data, x1, x2, x3, x4, x5, x6, x7, x8);
}
