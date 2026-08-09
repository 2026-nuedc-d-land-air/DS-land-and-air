#include "mpu6050.h"
#include "Delay.h"

u8 mpu6050_write(u8 addr, u8 reg, u8 len, u8* buf)
{ 
	unsigned char i;
	addr=addr<<1;
	MPU6050_IIC_Start();
	MPU6050_IIC_Send_Byte(addr);
	MPU6050_IIC_Send_Byte(reg);

	for(i=0;i<len;i++)            
		MPU6050_IIC_Send_Byte(*buf++);
	MPU6050_IIC_Stop();

	return 0;
}

u8 mpu6050_read(u8 addr, u8 reg, u8 len, u8 *buf)
{
	unsigned char i;
	addr=addr<<1;
	MPU6050_IIC_Start();
	MPU6050_IIC_Send_Byte(addr);
	MPU6050_IIC_Send_Byte(reg);

	MPU6050_IIC_Start();
	MPU6050_IIC_Send_Byte(addr+1);
	for(i=0;i<len-1;i++)  
		*buf++=MPU6050_IIC_Read_Byte(0); //ACK
	*buf=MPU6050_IIC_Read_Byte(1);     //NACK
	MPU6050_IIC_Stop();

	return 0;
}

void mpu6050_write_reg(u8 reg, u8 dat)
{
   mpu6050_write(SlaveAddress,reg,1,&dat);
}

u8 mpu6050_read_reg (u8 reg)
{
	u8 dat;
	mpu6050_read(SlaveAddress,reg,1,&dat);
	return dat;
}

u8 MPU_Init(void)
{
    MPU6050_IIC_IO_Init(); 
	mpu6050_write_reg(PWR_MGMT_1, 0x00);	//解除休眠状态
	mpu6050_write_reg(SMPLRT_DIV, 0x07);
	mpu6050_write_reg(CONFIG, 0x06);
	mpu6050_write_reg(GYRO_CONFIG, 0x18);
	mpu6050_write_reg(ACCEL_CONFIG, 0x01);
	
	// 检查ID
	if(mpu6050_read_reg(WHO_AM_I) == SlaveAddress)
		return 0;
	else
		return 1;
}

u8 MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
    u8 buf[6];
    u8 res;
    res = mpu6050_read(SlaveAddress, ACCEL_XOUT_H, 6, buf);
    if(res == 0)
    {
        *ax = ((u16)buf[0]<<8) | buf[1];
        *ay = ((u16)buf[2]<<8) | buf[3];
        *az = ((u16)buf[4]<<8) | buf[5];
    }
    return res;
}

u8 MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
    u8 buf[6];
    u8 res;
    res = mpu6050_read(SlaveAddress, GYRO_XOUT_H, 6, buf);
    if(res == 0)
    {
        *gx = ((u16)buf[0]<<8) | buf[1];
        *gy = ((u16)buf[2]<<8) | buf[3];
        *gz = ((u16)buf[4]<<8) | buf[5];
    }
    return res;
}
