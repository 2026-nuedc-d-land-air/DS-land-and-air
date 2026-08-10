#ifndef __JY301P_H
#define __JY301P_H

#include "stm32f10x.h"
#include "wit_c_sdk.h"
#include <stdint.h>

/*=== 维特九轴陀螺仪 I2C 配置 ===*/
/* 完全使用官方 wit_c_sdk 库 */

/*=== 设备地址 ===*/
#define WIT_I2C_ADDR        0x50    // 7位设备地址

/*=== 数据更新标志 ===*/
#define ACC_UPDATE      0x01
#define GYRO_UPDATE     0x02
#define ANGLE_UPDATE    0x04
#define MAG_UPDATE      0x08

/*=== 数据结构 ===*/
typedef struct {
    float acc[3];       // 加速度 X, Y, Z (g)
    float gyro[3];      // 角速度 X, Y, Z (°/s)
    float angle[3];     // 角度 Roll, Pitch, Yaw (°)
    int16_t mag[3];     // 磁场 X, Y, Z
} JY301P_Data;

extern JY301P_Data g_jy301p_data;

/*=== 函数声明 ===*/
void JY301P_Init(void);
void JY301P_Update(void);
void JY301P_DisplayOnOLED(void);
uint8_t JY301P_GetDataUpdateFlag(void);
void JY301P_ClearDataUpdateFlag(uint8_t flag);

#endif
