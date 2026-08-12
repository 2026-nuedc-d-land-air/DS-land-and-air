# 小车端比赛源码

本目录来自已烧录的比赛归档提交 `b63f401`（`release(car): archive flashed competition firmware`）。

- 主工程：`CMakeLists.txt`、`Hardware/`、`Src/`、`Inc/`、`Library/`、`Start/`、`System/`。
- 辅助脚本：`scripts/`。
- 基于 STM32F103ZET6 的八路灰度循迹智能小车，采用串级 PID 闭环控制（灰度循迹ladrc外环 + 速度 PI 内环）与陀螺仪串级，实现稳定循线、弯道自适应减速、陀螺仪姿态以及雷达信息参考限位判断停车，使用hal库。
- [`引脚图docs/小车pcb原理图展示.pdf`](引脚图docs/小车pcb原理图展示.pdf)
- [`./docs/D题小车端实施基线.md`](./docs/D题小车端实施基线.md)

## 器件材料表
| 器件 | 型号/规格 | 说明 |
|------|-----------|------|
| 主控芯片 | STM32F103ZET6 | Cortex-M3, 72MHz, 512KB Flash, 64K

###传感器模块
| 器件 | 型号/规格 | 说明 |
|------|-----------|------|
| 循迹传感器 | 亚博智能八路灰度传感器（蓝光） | adc PC0 PC1 PC2采样，
| 陀螺仪模块 | witt维特智能 jy901b|串口PD5 PD6|

###雷达系统
| 器件 | 型号/规格 | 说明 |
|------|-----------|------|
| 板卡 | 树莓派4 | ubuntu系统humble核心的slam tool_box建图策略
| 雷达模块 | 思岚s2l|串口PC10 PC11|
|无线模块|AS32-TTL-100-C|串口PC12 PD2|

### 驱动与动力
| 器件 | 型号/规格 | 说明 |
|------|-----------|------|
| 电机 | G513 霍尔编码器电机 | 带 AB 相正交编码器，PPR，减速比 |
| 橡胶轮 |65mm | 配套 513 电机 |
| 电机驱动 | TB6612FNG *2 | 双路 H 桥驱动，支持 PWM 调速 |

### 电源管理

| 器件 | 型号/规格 | 说明 |
|------|-----------|------|
| 锂电池 | 1500mAh | 卧式xt60
| 降压模块 | MP1584en | 固定 5V 稳压输出 |
| 六脚自锁开关 | 5.8*5.8 | 一路管控单片机5v，一路管理无线模块供电|


### 辅料与连接件

| 类别 | 明细 |
|------|------|
| 螺丝紧固 | M3 螺丝、各种高度铜柱、厚螺母；M2 螺丝、螺母 |
| 连接器 | 2.54mm 排针、排母、四层塑高18.5针长5.0加高排母至少29p、长脚排针 |
| 接线 | 杜邦线所有种类 |
| 被动元件 | 10uF 直插电容、1uF 直插电容，100nF 贴片陶瓷电容、4.7K 限流直插电阻、1K 直插电阻，22pf加速 s8550三极管|
| 指示器件 | 直插 LED（红/绿/指示） |
| 开关| 四脚开关高按键、5.8六脚自锁开关|

## 功能特性
###亚博智能八路灰度循迹模块无mcu
-8路灰度循迹模块，adc采样out输出
-非线性权重分布：`{-12, -8, -3, -1, 1, 3, 8, 12}`，边缘传感器权重更大，内侧轮反转提升差速
-传感器值在赛道测试获取，文件数值参考在文件：[`docs/灰度循迹触发测试.md`](docs/灰度循迹触发测试.md)

### 2. ladrc控制内核，串级pid
-**ladrc**：系统参数、速度因子、积分步长、观测器输出、步长系数
- **外环（循迹 PD）**：根据循迹误差计算转向输出
  - 非线性 Kp 调度：误差越大比例增益越强（err=0: 1.0x → err=12+: 4.0x）
  - D 项抑制振荡，防止蛇形走位
  - 基础参数：Kp=，Kd=
- **内环（速度 PI）**：确保左右轮精确跟踪目标速度
  - 位置式 PI 控制，带积分限幅（±200）
  - 参数：正常速600
  - 
### 4. 编码器测速与里程
- MG513 霍尔编码器，PPR x  减速比 x  倍频 =  脉冲/转
- 前轮电机 左 PA0/PA1 → TIM5，右 PA6/PA7 → TIM3，硬件正交解码
- 后轮电机：左PB6/PB7（原串口循迹）->TIM4 ，右 PC6/PC7（原步进）->TIM8
- 实时输出：转速 RPM、线速度 mm/s、累计距离 mm
- 左轮方向反转补偿（ENCODER_L_INVERT=1）

### 7. 按键控制与交互
任务与维护按键
PG13 任务一、PG9 任务二、PG12 停车态维护复位；均低有效 

### 10. 串口调试日志系统
-[`docs/D题_总体要求与车端验收基线.md`](docs/D题_总体要求与车端验收基线.md)

## 项目结构
### 1. 功能-文件映射（当前主流程）主控入口与调度：`User/main.c`
2.循迹功能
3.雷达串口接收
4.陀螺仪控制
5.电机控制
6.ladrc算法speed_ladrc.c
7.四轮控制bsp_four_wheel_direction.h
8.陀螺仪位姿解算bsp_car_pose_link.c
9.小车循迹逻辑app_line_follow_mission
……待补充
表格需要修改到当前对应的文件
| 模块 | 主要文件 | 关键实现 | 参数/依据 |
|------|----------|----------|-----------|
| 循迹采集 | `Hardware/bsp_ir_gpio.c/.h` | `IR_GPIO_Init`、`IR_GPIO_Read` | 
adc三路控制，out一路输出 |
| 循迹控制 | `User/main.c` | 加权误差、丢线恢复、交叉抑制、动态 Kp、PD 转向 | 
`SENSOR_WEIGHTS_V5`、`TRACK_KP/KD` |
| 速度环 | `User/main.c` | `PI_Init`、`PI_Compute`、目标速度合成、PWM 输出 | 
`SPEED_KP/KI`、`INTEGRAL_MAX`、`PWM_MAX/MIN` |
| 电机驱动 | `Hardware/bsp_motor.c/.h` | `Motor_SetSpeedBoth`、方向与PWM映射 | 
TB6612 管脚与 20kHz PWM |
| 编码器测速 | `Hardware/bsp_encoder.c/.h` | `Encoder_Init`、`Encoder_Update`、
`Encoder_GetSpeedMMS` | 11PPR × 20 × 4 倍频、轮径48mm |
| 超声波避障 | `Hardware/HCSR04.c/.h` + `User/main.c` | `HCSR04_Poll`、
`HCSR04_StartMeasure`、状态机接管 | `OBS_WARN_CM/OBS_AVOID_CM/OBS_EXIT_CM` |
| OLED 显示 | `Hardware/OLED.c/.h` + `User/main.c` | `OLED_ShowString` 分帧刷新
双页面显示 | `DISPLAY_PAGES=2`，按状态切页 |
| 按键输入 | `Hardware/bsp_key.c/.h`、`bsp_key2.c/.h` | `Key_Scan`、
`Key2_GetEvent` | C5短/长按与 C4 事件分离 |
| LED 指示 | `Hardware/bsp_led_pwm.c/.h` | `LED_PWM_Init`、`LED_SetBrightness`
`LED_StartFinishEffect` | 待机4档亮度、终点灯效 |
| 蜂鸣器 | `Hardware/bsp_buzzer.c/.h` | `Buzzer_PlayBeep`、`Buzzer_BeepTriple`
`Buzzer_Update` | 启动/激活/急停/终点提示 |
| 串口日志 | `User/main.c` + `Hardware/bsp_usart.c/.h` | `Log_Add`、`Log_Start/
Stop`、`Log_Export`、`UART4_Send_String` | `LOG_ENABLE`、`LOG_PERIOD_MS`、
`LOG_MAX_ENTRIES` |

## 任务执行拆解
-小车执行任务一和任务二。启动后，进行延时20s，等待飞机起飞。初始在a-b段以规定速度慢速行驶，与飞机协同以及完成投掷/降落的联调任务。
-执行之后，无人机切换返航模式，按照通信协议lora发送切换信号，小车接收通信信号之后，开始恢复正常速度加速行驶。
-小车落入编码器范围，以及雷达辅助半径范围内，被限制减速，缓慢靠近a点，在a点附近接受雷达辅助，陀螺仪yaw角归0辅助停车，编码器8800mm后解放，以及主要的灰度循迹低延时触发全黑停车
### 联调启动
-无人机以及小车启动雷达，相机启动代码。将小车和无人机的雷达重合，在地面站进行手动校准，三端信息握手成功才能完成校准。校准后具体实现在飞机伴飞和执行任务，小车每次启动可以点击重置按键执行重置。

### 2.1 重点文件补充说明
-循迹差速是外侧两轮速度相同，内侧两轮速度相同，内外侧做差速。
-雷达只做位置信息参考，雷达建图需要接入imu。雷达py启动以及建图文件为:
-树莓派雷达独立供电，需要ldo 5v拓展坞？dc-dc 5v供电无法供给雷达启动
-灰度循迹的采样信息为：  可以作为参考以及确认初始化
-小车雷达位置放在车头，小车中点距离雷达坐标存在13cm左右误差。协调好处理误差的单位，当前小车存在13cm内部误差处理，我认为这个误差导致验收时无人机返回起点有误差。伴飞小车发送自己雷达坐标，外部无人机接受坐标后同时处理小车和无人机起点存在的误差

## 比赛任务实现要点

- 使用灰度循迹传感器的实测灰度/映射进行线路判断；编码器辅助速度与停车距离控制。
- 雷达距离和陀螺仪 yaw 用作循迹偏差、弯道和回到起始点的限位辅助，降低越过 A 点后不能停车的风险。
- 接收飞机任务阶段中的伴随、降落/停机等协同信息，在满足阶段条件时提速；临近 A 点重新降速并执行停车保护，以缩短任务用时而不牺牲终点约束。

## 小车实际使用引脚使用
-[`docs/贡献与硬件索引.md`](docs/贡献与硬件索引.md)

### 10. 串口调试日志系统
-冻结日志，
## 用于原理图绘制的小车引脚规划（ZET6可用引脚分配）
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

