#include "MPU6050_I2C.h"

void MPU6050_IIC_IO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(MPU6050_IIC_RCC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = MPU6050_IIC_SCL_Pin | MPU6050_IIC_SDA_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MPU6050_IIC_GPIO, &GPIO_InitStructure);
 
	MPU6050_IIC_SCL_H;
	MPU6050_IIC_SDA_H;
}

void MPU6050_IIC_SDA_IO_OUT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = MPU6050_IIC_SDA_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MPU6050_IIC_GPIO, &GPIO_InitStructure);
}

void MPU6050_IIC_SDA_IO_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = MPU6050_IIC_SDA_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MPU6050_IIC_GPIO, &GPIO_InitStructure);
}

//发送MPU6050_IIC开始信号
void MPU6050_IIC_Start(void)
{
	MPU6050_IIC_SDA_IO_OUT();
	MPU6050_IIC_SDA_H;	  	  
	MPU6050_IIC_SCL_H;
	MPU6050_IIC_delay_4us();
	MPU6050_IIC_SDA_L; 
	MPU6050_IIC_delay_4us();
	MPU6050_IIC_SCL_L; 
}	  
//发送MPU6050_IIC停止信号
void MPU6050_IIC_Stop(void)
{
	MPU6050_IIC_SDA_IO_OUT();
	MPU6050_IIC_SCL_L;
	MPU6050_IIC_SDA_L;   
	MPU6050_IIC_delay_4us();
	MPU6050_IIC_SCL_H; 
	MPU6050_IIC_delay_4us(); 
	MPU6050_IIC_SDA_H;   
	MPU6050_IIC_delay_4us();  					   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 MPU6050_IIC_Read_Ack(void)
{
	u8 ucErrTime=0;
	MPU6050_IIC_SDA_IO_IN();
	MPU6050_IIC_SDA_H;
	MPU6050_IIC_delay_4us();	   
	MPU6050_IIC_SCL_H;
	MPU6050_IIC_delay_4us();	 
	while(MPU6050_IIC_SDA_READ)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			MPU6050_IIC_Stop();
			return 1;
		}
	}
	MPU6050_IIC_SCL_L; 	   
	return 0;  
} 
//发送ACK应答
void MPU6050_IIC_Send_Ack(u8 ack)
{
	MPU6050_IIC_SDA_IO_OUT();
	MPU6050_IIC_SCL_L;
	if(ack) MPU6050_IIC_SDA_H;
	else MPU6050_IIC_SDA_L;
	MPU6050_IIC_delay_4us();
	MPU6050_IIC_SCL_H;
	MPU6050_IIC_delay_4us();
	MPU6050_IIC_SCL_L;
}				 				     
//MPU6050_IIC发送一个字节
void MPU6050_IIC_Send_Byte(u8 txd)
{                        
    u8 t;
	MPU6050_IIC_SDA_IO_OUT();	
    MPU6050_IIC_SCL_L; 
    for(t=0;t<8;t++)
    {              
		if((txd&0x80)>>7)
			MPU6050_IIC_SDA_H;
		else
			MPU6050_IIC_SDA_L;
		txd<<=1; 	  
		MPU6050_IIC_delay_4us();    
		MPU6050_IIC_SCL_H;
		MPU6050_IIC_delay_4us(); 
		MPU6050_IIC_SCL_L;	
		MPU6050_IIC_delay_4us();
    }
    MPU6050_IIC_Read_Ack();		
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 MPU6050_IIC_Read_Byte(u8 ack)
{
	unsigned char i,receive=0;
	MPU6050_IIC_SDA_IO_IN();
	for(i=0;i<8;i++ )
	{
		MPU6050_IIC_SCL_L; 
		MPU6050_IIC_delay_4us();
		MPU6050_IIC_SCL_H;
		receive<<=1;
		if(MPU6050_IIC_SDA_READ)receive++;   
		MPU6050_IIC_delay_4us(); 
	}					 
	MPU6050_IIC_SCL_L;    
	MPU6050_IIC_delay_4us();
	MPU6050_IIC_Send_Ack(ack);
	return receive;
}
