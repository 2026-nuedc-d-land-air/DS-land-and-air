#include "stm32f10x.h"
#include "dht11.h"
#include "Delay.h"

// 数据存储数组: 湿度整数, 湿度小数, 温度整数, 温度小数
unsigned int rec_data[4];

// 配置为推挽输出
void DHT11_GPIO_Init_OUT(void)
{
GPIO_InitTypeDef GPIO_InitStructure;

RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP; 
GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// 配置为浮空输入
void DHT11_GPIO_Init_IN(void)
{
GPIO_InitTypeDef GPIO_InitStructure;

RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING; 
GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// 主机发送开始信号
void DHT11_Start(void)
{
DHT11_GPIO_Init_OUT(); // 输出模式

dht11_high; // 先拉高
Delay_us(30);

dht11_low; // 拉低电平至少18ms
Delay_ms(20);

dht11_high; // 拉高电平20~40us
Delay_us(30);

DHT11_GPIO_Init_IN(); // 输入模式
}

// 获取一个字节
unsigned char DHT11_REC_Byte(void)
{
unsigned char i = 0;
unsigned char data = 0;

for(i=0;i<8;i++) 
{
while( Read_Data == 0); // 等待低电平结束
Delay_us(30); // 延时30us区分0和1

data <<= 1; 

if( Read_Data == 1 ) // 如果还是高电平，则是1
{
data |= 1; 
}

while( Read_Data == 1 ); // 等待高电平结束
}

return data;
}

// 获取温湿度数据
void DHT11_REC_Data(void)
{
unsigned int R_H,R_L,T_H,T_L;
unsigned char RH,RL,TH,TL,CHECK;

DHT11_Start(); // 主机发送信号
dht11_high; 

if( Read_Data == 0 ) // 判断DHT11是否响应
{
while( Read_Data == 0); // 等待响应低电平结束
while( Read_Data == 1); // 等待响应高电平结束

R_H = DHT11_REC_Byte();
R_L = DHT11_REC_Byte();
T_H = DHT11_REC_Byte();
T_L = DHT11_REC_Byte();
CHECK = DHT11_REC_Byte(); 

dht11_low; 
Delay_us(55); 
dht11_high; 

if(R_H + R_L + T_H + T_L == CHECK) // 校验
{
RH = R_H;
RL = R_L;
TH = T_H;
TL = T_L;
}
}
rec_data[0] = RH;
rec_data[1] = RL;
rec_data[2] = TH;
rec_data[3] = TL;
}
