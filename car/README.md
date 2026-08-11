# 小车端比赛源码

本目录来自已烧录的比赛归档提交 `b63f401`（`release(car): archive flashed competition firmware`）。

- 主工程：`CMakeLists.txt`、`Hardware/`、`Src/`、`Inc/`、`Library/`、`Start/`、`System/`。
- 辅助脚本：`scripts/`。

#小车材料清单


## 比赛任务实现要点

- 使用灰度循迹传感器的实测灰度/映射进行线路判断；编码器辅助速度与停车距离控制。
- 雷达距离和陀螺仪 yaw 用作循迹偏差、弯道和回到起始点的限位辅助，降低越过 A 点后不能停车的风险。
- 接收飞机任务阶段中的伴随、降落/停机等协同信息，在满足阶段条件时提速；临近 A 点重新降速并执行停车保护，以缩短任务用时而不牺牲终点约束。

#小车实际使用引脚使用
前左/右 TB6612 PWM
PA2/TIM2_CH3、PA3/TIM2_CH4，20 kHz

前左方向 / 前右方向 / STBY
PE2/PE3、PE4/PE5、PE6
前轮编码器
左 PA0/PA1 → TIM5；右 PA6/PA7 → TIM3
后轮扩展 TB6612 PWM
PE13/TIM1_CH3、PE14/TIM1_CH4，TIM1 全重映射
后轮方向 / STBY
左 PF1/PF2，右 PF3/PF4，PB9
后轮编码器预留
左 PB6/PB7，右 PC6/PC7；当前仅输入初始化，任务镜像未轮询或闭环使用
八路灰度
PC0/PC1/PC2 → AD0/AD1/AD2，PG0 ← OUT；PG1 未使用
JY901B 姿态
USART2 重映射：PD5=MCU TX → 陀螺仪 RX，PD6=MCU RX ← 陀螺仪 TX，9600
树莓派 SL2 位姿
UART4：PC10=TX，PC11=RX，115200
LoRa / AS32
UART5：PC12=TX → LoRa RX，PD2=RX ← LoRa TX，115200
串口冻结日志
USART1：PA9=TX、PA10=RX，115200
任务与维护按键
PG13 任务一、PG9 任务二、PG12 停车态维护复位；均低有效 
任务指示灯
PG15 绿、PG14 备用/红、PE1 黄

##用于原理图绘制的小车引脚规划（ZET6可用引脚分配）
UART串口对 | 复用 已经定死：				
T1 PA9 PA10 （串口usb） 备用：PB6 PB7（映射 循迹串口）
T2 PA2 PA3 （电机PWM）备用：PD5 PD6 （映射 陀螺仪）
T3 PB10 PB11 （OLED） 备用： PD8 PD9（映射 maixcam 可临时日志导出）
T4 PC10 PC11（默认雷达串口）
T5 PC12 PD2 （ 映射 esp32 通信 或者无线模块延展在esp32引脚，临时偶尔作为日志导出串口）


TIM
左轮编码器	A相 / B相	PA0, PA1	TIM5_CH1 / CH2	已确认: TIM5/2专用于左轮编码器                         				
右轮编码器	A相 / B相	PA6, PA7	TIM3_CH1 / CH2	已确认: TIM3专用于右轮编码器
电机驱动(TB6612)	左电机PWM	PA2	TIM2_CH3 (PWM)	已确认: TIM5/2 专用于电机PWM     
                                   右电机PWM      PA3      TIM2_CH4 (PWM)       已确认: TIM5/2 专用于电机PWM
 STBY          PE6  GPIO_Output    已确认: 驱动使能
IN1，IN2，IN3，IN4
PE2 ， PE3  ， PE4 ，PE5

超声波       Trig / Echo      PF0 / PA8   GPIO / TIM1_CH1    已确认: TIM1专用于输入捕获



蜂鸣器  PWM驱动  PB8  TIM4_CH3 (PWM)   已确认: TIM4专用于蜂鸣器，以及预留tim4 ch4用途


定时器剩余： TIM4 ch4 PB9
TIM3 ch3 PB0   TIM3 ch4 PB1   （只能ADC或io备用）
TIM8 ch1 PC6   TIM8 ch2 PC7   （步进电机云台） （电赛右电机）
TIM8 ch3 PC8   TIM8 ch4 PC9  

循迹ADC 剩余可选：PC0 PC1 PC2（方案1灰度循迹）   PC3 PB0 PB1（灰度循迹备用）    PA4 PA5（数模DAC） 
灰度io：PG0  PG1

加上io口
按键5个：
PG9 PG10 PG11PG12 PG13

Led3个：
PG14（红） PG15（绿） PE1（黄）

循迹gpio八个：
x1-x8 PF1-4 PF6-9

一对步进电机/一对舵机需要8个io口，兼容六个42步进电机io口
PE7 PE8 PE9 PE10 PE11 PE12   PE13 PE14

激光笔三极管io开关一个：
PE15

引出排针组：
PC4 PC5 PD12 PD13 PD14 PD15



合并记录：
舵机/步进电机
PE7 PE8 PE9 PE10 PE11 PE12  PC6 PC7 （带定时器通道）

PB9  PC8 PC9

adc备用：
 PC3 PB0 PB1  PA4 PA5
io：
PG1 PE13 PE14（定时器通道1) PC4 PC5 PD12 PD13 PD14 PD15

