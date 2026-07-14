/**
  * @file    bsp_pwm.h
  * @brief   PWM管理模块
  * @note    使用TIM4产生PWM信号，用于蜂鸣器等外设
  *          PB8 - TIM4_CH3
  */

#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#include "stm32f10x.h"

/*====================================================================================*/
/*                                  引脚定义                                           */
/*====================================================================================*/

/* 蜂鸣器PWM - PB8 (TIM4_CH3) */
#define PWM_BUZZER_GPIO_PORT    GPIOB
#define PWM_BUZZER_GPIO_PIN     GPIO_Pin_8
#define PWM_BUZZER_GPIO_CLK     RCC_APB2Periph_GPIOB

/* TIM4配置 */
#define PWM_BUZZER_TIM          TIM4
#define PWM_BUZZER_TIM_CLK      RCC_APB1Periph_TIM4
#define PWM_BUZZER_TIM_CHANNEL  TIM_Channel_3

/*====================================================================================*/
/*                                  参数定义                                           */
/*====================================================================================*/

/* PWM频率 (Hz) - 蜂鸣器常用频率 */
#define PWM_FREQ_2K             2000    /* 2kHz */
#define PWM_FREQ_2K5            2500    /* 2.5kHz */
#define PWM_FREQ_3K             3000    /* 3kHz */
#define PWM_FREQ_4K             4000    /* 4kHz - 常用 */

/* 默认占空比 (0-100) */
#define PWM_DUTY_OFF            0       /* 关闭 */
#define PWM_DUTY_LOW            10      /* 低音量 */
#define PWM_DUTY_MED            30      /* 中音量 */
#define PWM_DUTY_HIGH           50      /* 高音量 */

/*====================================================================================*/
/*                                  函数声明                                           */
/*====================================================================================*/

/**
  * @brief  PWM初始化 (TIM4_CH3 - PB8)
  * @param  freq: PWM频率 (Hz)
  */
void PWM_Init(uint16_t freq);

/**
  * @brief  设置PWM占空比
  * @param  duty: 占空比 (0-100)
  */
void PWM_SetDuty(uint8_t duty);

/**
  * @brief  设置PWM频率
  * @param  freq: 频率 (Hz)
  */
void PWM_SetFreq(uint16_t freq);

/**
  * @brief  启动PWM输出
  */
void PWM_Start(void);

/**
  * @brief  停止PWM输出
  */
void PWM_Stop(void);

/**
  * @brief  获取当前占空比
  * @retval 占空比 (0-100)
  */
uint8_t PWM_GetDuty(void);

#endif /* __BSP_PWM_H */
