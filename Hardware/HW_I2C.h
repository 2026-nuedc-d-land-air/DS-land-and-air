/**
  * @file    HW_I2C.h
  * @brief   STM32F10x 硬件I2C2驱动头文件
  * @note    用于循迹模块，可与软件I2C动态切换
  */

#ifndef __HW_I2C_H
#define __HW_I2C_H

#include "stm32f10x.h"

/* 初始化和反初始化 */
void HW_I2C_Init(void);
void HW_I2C_DeInit(void);

/* 读写函数 */
uint8_t HW_I2C_ReadByte(uint8_t SlaveAddr, uint8_t RegAddr, uint8_t *Data);
uint8_t HW_I2C_WriteByte(uint8_t SlaveAddr, uint8_t RegAddr, uint8_t Data);

/* 模式切换 */
void HW_I2C_Enable(void);   // 切换到硬件I2C模式
void HW_I2C_Disable(void);  // 切换到软件I2C模式

/* 总线恢复 */
void HW_I2C_BusRecovery(void);

#endif /* __HW_I2C_H */
