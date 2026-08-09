/**
  * @file    bsp_encoder.c
  * @brief   编码器驱动模块实现 - MG310电机AB相正交编码器
  * @note    使用定时器编码器模式，4倍频计数
  *          左电机: TIM5 (PA0/PA1)
  *          右电机: TIM3 (PA6/PA7)
  */

#include "bsp_encoder.h"

/*====================================================================================*/
/*                                  全局变量                                           */
/*====================================================================================*/

Encoder_Data_t g_encoderL = {0};    /* 左电机编码器数据 */
Encoder_Data_t g_encoderR = {0};    /* 右电机编码器数据 */

/*====================================================================================*/
/*                                  内部函数                                           */
/*====================================================================================*/

/**
 * @brief  左电机编码器初始化 (TIM5)
  */
static void Encoder_Left_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    
    /* 使能时钟 */
    RCC_APB1PeriphClockCmd(ENCODER_L_TIM_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(ENCODER_L_GPIO_CLK, ENABLE);
    
    /* 配置GPIO - PA0, PA1 浮空输入 */
    GPIO_InitStructure.GPIO_Pin = ENCODER_L_ENCA_PIN | ENCODER_L_ENCB_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(ENCODER_L_GPIO_PORT, &GPIO_InitStructure);
    
    /* 定时器时基配置 */
    TIM_TimeBaseStructure.TIM_Period = 65535;           /* 16位计数器最大值 */
    TIM_TimeBaseStructure.TIM_Prescaler = 0;            /* 不分频 */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(ENCODER_L_TIM, &TIM_TimeBaseStructure);
    
    /* 编码器模式配置 - 模式3 (TI1和TI2都计数，4倍频) */
    TIM_EncoderInterfaceConfig(ENCODER_L_TIM, 
                               TIM_EncoderMode_TI12,    /* 双边沿计数 */
                               TIM_ICPolarity_Rising,   /* TI1极性 */
                               TIM_ICPolarity_Rising);  /* TI2极性 */
    
    /* 输入捕获滤波：A/B 两个通道都配置 */
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICFilter = 6;   /* 滤波器，防抖动 */
    TIM_ICInit(ENCODER_L_TIM, &TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(ENCODER_L_TIM, &TIM_ICInitStructure);
    
    /* 清零计数器 */
    TIM_SetCounter(ENCODER_L_TIM, 0);
    
    /* 使能定时器 */
    TIM_Cmd(ENCODER_L_TIM, ENABLE);
}

/**
  * @brief  右电机编码器初始化 (TIM3)
  */
static void Encoder_Right_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    
    /* 使能时钟 */
    RCC_APB1PeriphClockCmd(ENCODER_R_TIM_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(ENCODER_R_GPIO_CLK, ENABLE);
    
    /* 配置GPIO - PA6, PA7 浮空输入 */
    GPIO_InitStructure.GPIO_Pin = ENCODER_R_ENCA_PIN | ENCODER_R_ENCB_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(ENCODER_R_GPIO_PORT, &GPIO_InitStructure);
    
    /* 定时器时基配置 */
    TIM_TimeBaseStructure.TIM_Period = 65535;           /* 16位计数器最大值 */
    TIM_TimeBaseStructure.TIM_Prescaler = 0;            /* 不分频 */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(ENCODER_R_TIM, &TIM_TimeBaseStructure);
    
    /* 编码器模式配置 - 模式3 (TI1和TI2都计数，4倍频) */
    TIM_EncoderInterfaceConfig(ENCODER_R_TIM, 
                               TIM_EncoderMode_TI12,    /* 双边沿计数 */
                               TIM_ICPolarity_Rising,   /* TI1极性 */
                               TIM_ICPolarity_Rising);  /* TI2极性 */
    
    /* 输入捕获滤波：A/B 两个通道都配置 */
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICFilter = 6;   /* 滤波器，防抖动 */
    TIM_ICInit(ENCODER_R_TIM, &TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(ENCODER_R_TIM, &TIM_ICInitStructure);
    
    /* 清零计数器 */
    TIM_SetCounter(ENCODER_R_TIM, 0);
    
    /* 使能定时器 */
    TIM_Cmd(ENCODER_R_TIM, ENABLE);
}

/*====================================================================================*/
/*                                  接口函数                                           */
/*====================================================================================*/

/**
  * @brief  编码器初始化
  */
void Encoder_Init(void)
{
    Encoder_Left_Init();
    Encoder_Right_Init();
    
    /* 清零数据 */
    g_encoderL.count = 0;
    g_encoderL.totalCount = 0;
    g_encoderL.speed = 0;
    g_encoderL.speedRPM = 0;
    g_encoderL.speedMMS = 0;
    g_encoderL.distance = 0;
    
    g_encoderR.count = 0;
    g_encoderR.totalCount = 0;
    g_encoderR.speed = 0;
    g_encoderR.speedRPM = 0;
    g_encoderR.speedMMS = 0;
    g_encoderR.distance = 0;
}

/**
  * @brief  读取编码器计数值并更新数据
  * @param  samplePeriodMs: 采样周期 (ms)
  */
void Encoder_Update(uint32_t samplePeriodMs)
{
    int16_t countL, countR;
    float sampleFreq;

    if (samplePeriodMs == 0U)
    {
        return;
    }
    sampleFreq = 1000.0f / samplePeriodMs;  /* 采样频率 (Hz) */
    
    /* 读取左电机编码器 */
    countL = (int16_t)TIM_GetCounter(ENCODER_L_TIM);
    TIM_SetCounter(ENCODER_L_TIM, 0);
    
    /* 读取右电机编码器 */
    countR = (int16_t)TIM_GetCounter(ENCODER_R_TIM);
    TIM_SetCounter(ENCODER_R_TIM, 0);
    
    /* 应用方向反转 */
    /* 测试结果：左轮向前是负值(需反转)，右轮向前是正值(不反转) */
#if ENCODER_L_INVERT
    countL = (int16_t)-countL;
#endif
#if ENCODER_R_INVERT
    countR = (int16_t)-countR;
#endif
    
    /* 更新左电机数据 */
    g_encoderL.count = countL;
    g_encoderL.totalCount += countL;
    g_encoderL.speed = (float)countL;
    /* RPM = (脉冲数 / 每转脉冲数) * 采样频率 * 60 */
    g_encoderL.speedRPM = (float)countL / ENCODER_4X_PPR * sampleFreq * 60.0f;
    /* mm/s = RPM * 周长 / 60 */
    g_encoderL.speedMMS = g_encoderL.speedRPM * WHEEL_CIRCUMFERENCE_MM / 60.0f;
    /* 距离 = 脉冲数 / 每转脉冲数 * 周长 */
    g_encoderL.distance += (float)countL / ENCODER_4X_PPR * WHEEL_CIRCUMFERENCE_MM;
    
    /* 更新右电机数据 */
    g_encoderR.count = countR;
    g_encoderR.totalCount += countR;
    g_encoderR.speed = (float)countR;
    g_encoderR.speedRPM = (float)countR / ENCODER_4X_PPR * sampleFreq * 60.0f;
    g_encoderR.speedMMS = g_encoderR.speedRPM * WHEEL_CIRCUMFERENCE_MM / 60.0f;
    g_encoderR.distance += (float)countR / ENCODER_4X_PPR * WHEEL_CIRCUMFERENCE_MM;
}

/**
  * @brief  获取编码器当前计数值
  */
int16_t Encoder_GetCount(Encoder_Index_t encoder)
{
    if (encoder == ENCODER_LEFT)
        return g_encoderL.count;
    else
        return g_encoderR.count;
}

/**
  * @brief  获取编码器速度 (RPM)
  */
float Encoder_GetSpeedRPM(Encoder_Index_t encoder)
{
    if (encoder == ENCODER_LEFT)
        return g_encoderL.speedRPM;
    else
        return g_encoderR.speedRPM;
}

/**
  * @brief  获取编码器速度 (mm/s)
  */
float Encoder_GetSpeedMMS(Encoder_Index_t encoder)
{
    if (encoder == ENCODER_LEFT)
        return g_encoderL.speedMMS;
    else
        return g_encoderR.speedMMS;
}

/**
  * @brief  获取累计距离 (mm)
  */
float Encoder_GetDistance(Encoder_Index_t encoder)
{
    if (encoder == ENCODER_LEFT)
        return g_encoderL.distance;
    else
        return g_encoderR.distance;
}

/**
  * @brief  重置编码器累计值
  */
void Encoder_Reset(void)
{
    TIM_SetCounter(ENCODER_L_TIM, 0);
    TIM_SetCounter(ENCODER_R_TIM, 0);
    
    g_encoderL.count = 0;
    g_encoderL.totalCount = 0;
    g_encoderL.speed = 0;
    g_encoderL.speedRPM = 0;
    g_encoderL.speedMMS = 0;
    g_encoderL.distance = 0;
    
    g_encoderR.count = 0;
    g_encoderR.totalCount = 0;
    g_encoderR.speed = 0;
    g_encoderR.speedRPM = 0;
    g_encoderR.speedMMS = 0;
    g_encoderR.distance = 0;
}
