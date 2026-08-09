/**
  * @file    bsp_encoder.h
  * @brief   编码器驱动模块 - MG310电机AB相正交编码器
  * @note    左电机: TIM5 (PA0/PA1)
  *          右电机: TIM3 (PA6/PA7)
  *          使用定时器编码器模式，4倍频计数
  */

#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H

#include "stm32f10x.h"

/*====================================================================================*/
/*                                  引脚定义                                           */
/*====================================================================================*/

/* 左电机编码器 - TIM5 (PA0/PA1) */
#define ENCODER_L_TIM           TIM5
#define ENCODER_L_TIM_CLK       RCC_APB1Periph_TIM5
#define ENCODER_L_GPIO_PORT     GPIOA
#define ENCODER_L_GPIO_CLK      RCC_APB2Periph_GPIOA
#define ENCODER_L_ENCA_PIN      GPIO_Pin_0      /* PA0 - TIM5_CH1 */
#define ENCODER_L_ENCB_PIN      GPIO_Pin_1      /* PA1 - TIM5_CH2 */

/* 右电机编码器 - TIM3 (PA6/PA7) */
#define ENCODER_R_TIM           TIM3
#define ENCODER_R_TIM_CLK       RCC_APB1Periph_TIM3
#define ENCODER_R_GPIO_PORT     GPIOA
#define ENCODER_R_GPIO_CLK      RCC_APB2Periph_GPIOA
#define ENCODER_R_ENCA_PIN      GPIO_Pin_6      /* PA6 - TIM3_CH1 */
#define ENCODER_R_ENCB_PIN      GPIO_Pin_7      /* PA7 - TIM3_CH2 */

/*====================================================================================*/
/*                                  编码器参数                                         */
/*====================================================================================*/

/* MG310电机编码器参数 (霍尔编码器版) */
#define ENCODER_PPR             11      /* 编码器每转脉冲数 (Pulses Per Revolution) */
#define ENCODER_GEAR_RATIO      20      /* 减速比 1:20 (根据规格书) */
#define ENCODER_4X_PPR          (ENCODER_PPR * 4 * ENCODER_GEAR_RATIO)  /* 4倍频后每转脉冲 = 11*4*20 = 880 */

/* 编码器方向反转配置 (1=反转, 0=不反转) */
/* 测试结果：左轮向前是负值(需反转)，右轮向前是正值(不反转) */
#define ENCODER_L_INVERT        1       /* 左编码器：向前是负值，需要反转 */
#define ENCODER_R_INVERT        0       /* 右编码器：向前是正值，不需要反转 */

/* 轮子参数 */
#define WHEEL_DIAMETER_MM       48      /* 轮子直径 48mm */
#define WHEEL_CIRCUMFERENCE_MM  (WHEEL_DIAMETER_MM * 314 / 100)  /* 轮子周长 (mm) */

/*====================================================================================*/
/*                                  类型定义                                           */
/*====================================================================================*/

typedef enum {
    ENCODER_LEFT = 0,
    ENCODER_RIGHT
} Encoder_Index_t;

typedef struct {
    int16_t count;          /* 当前计数值 (有符号，正=正转，负=反转) */
    int32_t totalCount;     /* 累计脉冲数 */
    float speed;            /* 当前速度 (脉冲/采样周期) */
    float speedRPM;         /* 当前速度 (RPM) */
    float speedMMS;         /* 当前速度 (mm/s) */
    float distance;         /* 累计距离 (mm) */
} Encoder_Data_t;

/*====================================================================================*/
/*                                  全局变量                                           */
/*====================================================================================*/

extern Encoder_Data_t g_encoderL;   /* 左电机编码器数据 */
extern Encoder_Data_t g_encoderR;   /* 右电机编码器数据 */

/*====================================================================================*/
/*                                  函数声明                                           */
/*====================================================================================*/

/**
  * @brief  编码器初始化
  */
void Encoder_Init(void);

/**
  * @brief  读取编码器计数值并更新数据
  * @param  samplePeriodMs: 采样周期 (ms)
  * @note   需要定期调用此函数 (建议10-50ms)
  */
void Encoder_Update(uint32_t samplePeriodMs);

/**
  * @brief  获取编码器当前计数值
  * @param  encoder: ENCODER_LEFT 或 ENCODER_RIGHT
  * @return 当前计数值 (有符号)
  */
int16_t Encoder_GetCount(Encoder_Index_t encoder);

/**
  * @brief  获取编码器速度 (RPM)
  * @param  encoder: ENCODER_LEFT 或 ENCODER_RIGHT
  * @return 速度 (转/分)
  */
float Encoder_GetSpeedRPM(Encoder_Index_t encoder);

/**
  * @brief  获取编码器速度 (mm/s)
  * @param  encoder: ENCODER_LEFT 或 ENCODER_RIGHT
  * @return 速度 (毫米/秒)
  */
float Encoder_GetSpeedMMS(Encoder_Index_t encoder);

/**
  * @brief  获取累计距离 (mm)
  * @param  encoder: ENCODER_LEFT 或 ENCODER_RIGHT
  * @return 距离 (毫米)
  */
float Encoder_GetDistance(Encoder_Index_t encoder);

/**
  * @brief  重置编码器累计值
  */
void Encoder_Reset(void);

#endif /* __BSP_ENCODER_H */
