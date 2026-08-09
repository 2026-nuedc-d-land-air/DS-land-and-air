#ifndef __TRACKING_H
#define __TRACKING_H

#include "stm32f10x.h"

/*====================================================================================*/
/*                                  函数声明                                           */
/*====================================================================================*/

/**
  * 函    数：循迹模块初始化
  * 参    数：无
  * 返 回 值：无
  */
void Tracking_Init(void);

/**
  * 函    数：读取循迹数据（默认接口）
  * 参    数：无
  * 返 回 值：8位循迹数据
  */
uint8_t Tracking_Read(void);

/**
  * 函    数：直接读取循迹数据
  * 参    数：无
  * 返 回 值：8位循迹数据
  */
uint8_t Tracking_ReadDirect(void);

/**
  * 函    数：从指定寄存器读取循迹数据
  * 参    数：reg 寄存器地址
  * 返 回 值：读取到的数据
  */
uint8_t Tracking_ReadReg(uint8_t reg);

/**
  * 函    数：使用高级接口读取循迹数据
  * 参    数：reg 寄存器地址
  * 返 回 值：读取到的数据
  */
uint8_t Tracking_ReadByI2CFunc(uint8_t reg);

/**
  * 函    数：带总线切换的循迹数据读取
  * 参    数：无
  * 返 回 值：8位循迹数据
  * 说    明：从MyI2C切换到IOI2C，读取后再切换回去
  */
uint8_t Tracking_ReadWithSwitch(void);

/**
  * 函    数：获取最新的循迹数据（不进行I2C通信）
  * 参    数：无
  * 返 回 值：上次读取的8位循迹数据
  */
uint8_t Tracking_GetLastData(void);

/*====================================================================================*/
/*                          硬件I2C读取函数（推荐使用）                                */
/*====================================================================================*/

/**
  * 函    数：使用硬件I2C读取循迹数据
  * 参    数：无
  * 返 回 值：8位循迹数据
  * 说    明：使用STM32硬件I2C2外设，时序精确
  */
uint8_t Tracking_ReadHW(void);

/**
  * 函    数：使用硬件I2C读取循迹数据（带总线切换）
  * 参    数：无
  * 返 回 值：8位循迹数据
  * 说    明：先切换到硬件I2C，读取后切换回软件I2C
  */
uint8_t Tracking_ReadHW_WithSwitch(void);

/**
  * 函    数：解析循迹数据到8个独立变量
  * 参    数：data - 原始8位数据
  *           x1~x8 - 8个传感器状态输出指针
  * 返 回 值：无
  */
void Tracking_ParseData(uint8_t data, uint8_t *x1, uint8_t *x2, uint8_t *x3, uint8_t *x4,
                        uint8_t *x5, uint8_t *x6, uint8_t *x7, uint8_t *x8);

/**
  * 函    数：使用硬件I2C读取并解析循迹数据
  * 参    数：x1~x8 - 8个传感器状态输出指针
  * 返 回 值：无
  */
void Tracking_DealIRData_HW(uint8_t *x1, uint8_t *x2, uint8_t *x3, uint8_t *x4,
                            uint8_t *x5, uint8_t *x6, uint8_t *x7, uint8_t *x8);

#endif
