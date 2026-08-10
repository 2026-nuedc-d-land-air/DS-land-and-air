#include "JY301P.h"
#include "IOI2C.h"
#include "OLED.h"
#include "Delay.h"
#include "wit_c_sdk.h"

/*
 * 维特九轴陀螺仪 I2C 驱动
 * 完全使用官方 wit_c_sdk 库
 * I2C引脚: PB10(SCL), PB11(SDA) - 与OLED共用
 * 设备地址: 0x50 (7位地址)
 * 使用IOI2C驱动（推挽模式）
 */

JY301P_Data g_jy301p_data;
static volatile uint8_t s_cDataUpdate = 0;

/*=== I2C适配函数（适配IOI2C.c到官方SDK） ===*/

/**
 * @brief  I2C写入函数（官方SDK回调格式）
 * @param  ucAddr: 设备地址（8位写地址）
 * @param  ucReg: 寄存器地址
 * @param  p_ucVal: 数据指针
 * @param  uiLen: 数据长度
 * @retval 1:成功, 0:失败
 */
static int32_t JY301P_I2cWriteFunc(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen)
{
    uint32_t i;
    
    IIC_Start();
    IIC_Send_Byte(ucAddr);         // 发送设备写地址
    if (IIC_Wait_Ack() != 0) {
        IIC_Stop();
        return 0;
    }
    
    IIC_Send_Byte(ucReg);          // 发送寄存器地址
    if (IIC_Wait_Ack() != 0) {
        IIC_Stop();
        return 0;
    }
    
    for (i = 0; i < uiLen; i++) {
        IIC_Send_Byte(p_ucVal[i]);
        if (IIC_Wait_Ack() != 0) {
            IIC_Stop();
            return 0;
        }
    }
    
    IIC_Stop();
    return 1;
}

/**
 * @brief  I2C读取函数（官方SDK回调格式）
 * @param  ucAddr: 设备地址（8位写地址）
 * @param  ucReg: 寄存器地址
 * @param  p_ucVal: 数据缓冲区
 * @param  uiLen: 读取长度
 * @retval 1:成功, 0:失败
 */
static int32_t JY301P_I2cReadFunc(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen)
{
    uint32_t i;
    
    IIC_Start();
    IIC_Send_Byte(ucAddr);         // 发送设备写地址
    if (IIC_Wait_Ack() != 0) {
        IIC_Stop();
        return 0;
    }
    
    IIC_Send_Byte(ucReg);          // 发送寄存器地址
    if (IIC_Wait_Ack() != 0) {
        IIC_Stop();
        return 0;
    }
    
    IIC_Start();                   // 重复起始
    IIC_Send_Byte(ucAddr + 1);     // 发送设备读地址
    if (IIC_Wait_Ack() != 0) {
        IIC_Stop();
        return 0;
    }
    
    for (i = 0; i < uiLen; i++) {
        if (i + 1 == uiLen) {
            p_ucVal[i] = IIC_Read_Byte(0);  // 最后一个字节发送NACK
        } else {
            p_ucVal[i] = IIC_Read_Byte(1);  // 其他字节发送ACK
        }
    }
    
    IIC_Stop();
    return 1;
}

/**
 * @brief  延时函数（官方SDK回调格式）
 */
static void WitDelayMs(uint16_t ms)
{
    Delay_ms(ms);
}

/**
 * @brief  数据更新回调函数（官方SDK回调格式）
 */
static void SensorDataUpdate(uint32_t uiReg, uint32_t uiRegNum)
{
    int i;
    
    for (i = 0; i < (int)uiRegNum; i++) {
        switch (uiReg + i) {
            case AX:
            case AY:
            case AZ:
                s_cDataUpdate |= ACC_UPDATE;
                break;
            case GX:
            case GY:
            case GZ:
                s_cDataUpdate |= GYRO_UPDATE;
                break;
            case HX:
            case HY:
            case HZ:
                s_cDataUpdate |= MAG_UPDATE;
                break;
            case Roll:
            case Pitch:
            case Yaw:
                s_cDataUpdate |= ANGLE_UPDATE;
                break;
            default:
                break;
        }
    }
}

/**
 * @brief  初始化陀螺仪（使用官方SDK）
 */
void JY301P_Init(void)
{
    // 初始化官方SDK，使用I2C协议，设备地址0x50
    WitInit(WIT_PROTOCOL_I2C, WIT_I2C_ADDR);
    
    // 注册I2C读写函数
    WitI2cFuncRegister(JY301P_I2cWriteFunc, JY301P_I2cReadFunc);
    
    // 注册数据更新回调
    WitRegisterCallBack(SensorDataUpdate);
    
    // 注册延时函数
    WitDelayMsRegister(WitDelayMs);
    
    // 清零数据
    s_cDataUpdate = 0;
    g_jy301p_data.acc[0] = 0;
    g_jy301p_data.acc[1] = 0;
    g_jy301p_data.acc[2] = 0;
    g_jy301p_data.gyro[0] = 0;
    g_jy301p_data.gyro[1] = 0;
    g_jy301p_data.gyro[2] = 0;
    g_jy301p_data.angle[0] = 0;
    g_jy301p_data.angle[1] = 0;
    g_jy301p_data.angle[2] = 0;
    g_jy301p_data.mag[0] = 0;
    g_jy301p_data.mag[1] = 0;
    g_jy301p_data.mag[2] = 0;
}

/**
 * @brief  读取并更新陀螺仪数据（使用官方SDK）
 */
void JY301P_Update(void)
{
    int i;
    
    // 使用官方SDK读取12个寄存器（AX到Yaw）
    WitReadReg(AX, 12);
    
    // 从官方SDK的sReg数组中获取数据并转换
    if (s_cDataUpdate & ACC_UPDATE) {
        for (i = 0; i < 3; i++) {
            g_jy301p_data.acc[i] = (float)sReg[AX + i] / 32768.0f * 16.0f;
        }
    }
    
    if (s_cDataUpdate & GYRO_UPDATE) {
        for (i = 0; i < 3; i++) {
            g_jy301p_data.gyro[i] = (float)sReg[GX + i] / 32768.0f * 2000.0f;
        }
    }
    
    if (s_cDataUpdate & MAG_UPDATE) {
        for (i = 0; i < 3; i++) {
            g_jy301p_data.mag[i] = sReg[HX + i];
        }
    }
    
    if (s_cDataUpdate & ANGLE_UPDATE) {
        for (i = 0; i < 3; i++) {
            g_jy301p_data.angle[i] = (float)sReg[Roll + i] / 32768.0f * 180.0f;
        }
    }
}

/**
 * @brief  获取数据更新标志
 */
uint8_t JY301P_GetDataUpdateFlag(void)
{
    return s_cDataUpdate;
}

/**
 * @brief  清除数据更新标志
 */
void JY301P_ClearDataUpdateFlag(uint8_t flag)
{
    s_cDataUpdate &= ~flag;
}

/**
 * @brief  在OLED上显示陀螺仪数据
 */
void JY301P_DisplayOnOLED(void)
{
    // 第1行: 显示角度 Roll, Pitch, Yaw
    OLED_ShowString(1, 1, "R:");
    OLED_ShowSignedNum(1, 3, (int32_t)g_jy301p_data.angle[0], 4);
    OLED_ShowString(1, 8, "P:");
    OLED_ShowSignedNum(1, 10, (int32_t)g_jy301p_data.angle[1], 4);
    
    // 第2行: 显示Yaw角度 和 角速度GZ
    OLED_ShowString(2, 1, "Y:");
    OLED_ShowSignedNum(2, 3, (int32_t)g_jy301p_data.angle[2], 4);
    OLED_ShowString(2, 8, "GZ:");
    OLED_ShowSignedNum(2, 11, (int32_t)g_jy301p_data.gyro[2], 5);
    
    // 第3行: 显示角速度 GX, GY (°/s)
    OLED_ShowString(3, 1, "GX:");
    OLED_ShowSignedNum(3, 4, (int32_t)g_jy301p_data.gyro[0], 5);
    OLED_ShowString(3, 10, "GY:");
    OLED_ShowSignedNum(3, 13, (int32_t)g_jy301p_data.gyro[1], 4);
    
    // 第4行: 显示加速度 AX, AY, AZ (放大100倍显示，单位0.01g)
    OLED_ShowString(4, 1, "A:");
    OLED_ShowSignedNum(4, 3, (int32_t)(g_jy301p_data.acc[0] * 100), 3);
    OLED_ShowString(4, 7, ",");
    OLED_ShowSignedNum(4, 8, (int32_t)(g_jy301p_data.acc[1] * 100), 3);
    OLED_ShowString(4, 12, ",");
    OLED_ShowSignedNum(4, 13, (int32_t)(g_jy301p_data.acc[2] * 100), 3);
}
