#ifndef __MPU6050_I2C_H
#define __MPU6050_I2C_H
#include "stm32f10x.h"
#include "Delay.h"

//I2C引脚定义
#define MPU6050_IIC_GPIO                   GPIOB
#define MPU6050_IIC_RCC                    RCC_APB2Periph_GPIOB
#define MPU6050_IIC_SCL_Pin                GPIO_Pin_10	         //PB10
#define MPU6050_IIC_SDA_Pin                GPIO_Pin_11                                                                                                                                                                                                                                                                                                                                                                                                                                                     	         //PB11

//位带操作 (需确保sys.h可用，或直接使用库函数)
//这里为了通用性，使用库函数或直接寄存器操作宏
#define	MPU6050_IIC_SCL_H      GPIO_SetBits(MPU6050_IIC_GPIO, MPU6050_IIC_SCL_Pin)
#define	MPU6050_IIC_SCL_L      GPIO_ResetBits(MPU6050_IIC_GPIO, MPU6050_IIC_SCL_Pin)

#define	MPU6050_IIC_SDA_H      GPIO_SetBits(MPU6050_IIC_GPIO, MPU6050_IIC_SDA_Pin)
#define	MPU6050_IIC_SDA_L      GPIO_ResetBits(MPU6050_IIC_GPIO, MPU6050_IIC_SDA_Pin)

#define	MPU6050_IIC_SDA_READ   GPIO_ReadInputDataBit(MPU6050_IIC_GPIO, MPU6050_IIC_SDA_Pin)

#define MPU6050_IIC_delay_4us()            Delay_us(4)


//MPU6050_IIC函数声明
void MPU6050_IIC_IO_Init(void);               //初始化IO
void MPU6050_IIC_SDA_IO_OUT(void);    
void MPU6050_IIC_SDA_IO_IN(void);
void MPU6050_IIC_Start(void);				  //发送开始信号
void MPU6050_IIC_Stop(void);	  			  //发送停止信号
void MPU6050_IIC_Send_Byte(u8 txd);			  //发送一个字节
u8   MPU6050_IIC_Read_Byte(u8 ack);           //读取一个字节
u8   MPU6050_IIC_Read_Ack(void); 			  //等待ACK
void MPU6050_IIC_Send_Ack(u8 ack);			  //发送ACK

#endif
