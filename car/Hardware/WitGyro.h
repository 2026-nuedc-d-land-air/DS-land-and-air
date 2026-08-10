#ifndef __WITGYRO_H
#define __WITGYRO_H

#include "stm32f10x.h"

/* 维特九轴陀螺仪 I2C 设备地址 (7位地址) */
#define WIT_GYRO_ADDR       0x50    // 默认I2C地址

/* 寄存器地址定义 */


#define WIT_REG_AX0         0x05    // 加速度X0偏
#define WIT_REG_AX1         0x06    // 加速度Y0偏
#define WIT_REG_AZ1         0x07    // 加速度Z0偏
#define WIT_REG_GX0         0x08    // 角速度X0偏
#define WIT_REG_GY0         0x09    // 角速度Y0偏
#define WIT_REG_GZ0         0x0A    // 角速度Z0偏
#define WIT_REG_HX0         0x0B    // 磁场X0偏
#define WIT_REG_HY0         0x0C    // 磁场Y0偏    
#define WIT_REG_HZ0         0x0D    // 磁场Z0偏
#define WIT_REG_ADDR        0x1A    //设备地址
#define WIT_REG_NGX         0x1C    //磁场X校准范围
#define WIT_REG_NGY         0x1D    //磁场Y校准范围
#define WIT_REG_NGZ         0x1E    //磁场Z校准范围
#define WIT_REG_FILTK       0x25    // 滤波系数
#define WIT_REG_ACCFIl      0x2A    // 加速度滤波
#define WIT_REG_READADDR    0x27    // 读取起始地址



#define WIT_REG_AX          0x34    // 加速度X
#define WIT_REG_AY          0x35    // 加速度Y
#define WIT_REG_AZ          0x36    // 加速度Z
#define WIT_REG_GX          0x37    // 角速度X
#define WIT_REG_GY          0x38    // 角速度Y
#define WIT_REG_GZ          0x39    // 角速度Z
#define WIT_REG_HX          0x3A    // 磁场X
#define WIT_REG_HY          0x3B    // 磁场Y
#define WIT_REG_HZ          0x3C    // 磁场Z
#define WIT_REG_ROLL        0x3D    // 横滚角
#define WIT_REG_PITCH       0x3E    // 俯仰角
#define WIT_REG_YAW         0x3F    // 偏航角
#define WIT_REG_TEMP        0x40    // 温度


/* 陀螺仪数据结构体 */
typedef struct {
    float accX, accY, accZ;         // 加速度 (g)
    float gyroX, gyroY, gyroZ;      // 角速度 (°/s)
    float roll, pitch, yaw;         // 角度 (°)
    int16_t magX, magY, magZ;       // 磁场 (原始值)
    float temperature;              // 温度 (°C)
} WitGyro_Data_t;

/* 函数声明 */
void WitGyro_Init(void);
uint8_t WitGyro_ReadData(WitGyro_Data_t *data);
uint8_t WitGyro_ReadAngle(float *roll, float *pitch, float *yaw);
uint8_t WitGyro_ReadAcc(float *accX, float *accY, float *accZ);
uint8_t WitGyro_ReadGyro(float *gyroX, float *gyroY, float *gyroZ);
void WitGyro_DisplayOnOLED(WitGyro_Data_t *data);

#endif
