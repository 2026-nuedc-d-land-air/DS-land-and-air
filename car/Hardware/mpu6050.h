#ifndef _MPU6050_H_
#define _MPU6050_H_
#include "stm32f10x.h"
#include "MPU6050_I2C.h"

#define	SMPLRT_DIV		0x19	//采样率分频
#define	CONFIG		  	0x1A	//低通滤波配置
#define	GYRO_CONFIG		0x1B	//陀螺仪配置
#define	ACCEL_CONFIG	0x1C	//加速度配置
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B //电源管理1
#define	WHO_AM_I	  	0x75	
#define	SlaveAddress	0x68 //IIC地址

u8   mpu6050_write(u8 addr, u8 reg, u8 len, u8* buf);
u8   mpu6050_read (u8 addr, u8 reg, u8 len, u8 *buf);
void mpu6050_write_reg(u8 reg, u8 dat);
u8   mpu6050_read_reg (u8 reg);
u8   MPU_Init(void); // 修改为 MPU_Init 以匹配 main.c

// 新增高层接口以匹配 main.c
u8 MPU_Get_Accelerometer(short *ax, short *ay, short *az);
u8 MPU_Get_Gyroscope(short *gx, short *gy, short *gz);

#endif
