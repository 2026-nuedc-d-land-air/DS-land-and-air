/**
  * @file    bsp_motor.h
  * @brief   电机驱动模块 - TB6612FNG驱动 + MG310电机
  * @note    驱动板引脚:
  *          PWMA (左电机) - PA2 (TIM2_CH3)
  *          PWMB (右电机) - PA3 (TIM2_CH4)
  *          AIN1 - PE2, AIN2 - PE3 (左电机方向)
  *          BIN1 - PE4, BIN2 - PE5 (右电机方向)
  *          STBY - PE6 (待机控制)
  */

#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "stm32f10x.h"

/*====================================================================================*/
/*                                  引脚定义                                           */
/*====================================================================================*/

/* PWM引脚 - TIM2 */
#define MOTOR_TIM               TIM2
#define MOTOR_TIM_CLK           RCC_APB1Periph_TIM2
#define MOTOR_PWM_GPIO_PORT     GPIOA
#define MOTOR_PWM_GPIO_CLK      RCC_APB2Periph_GPIOA
#define MOTOR_PWMA_PIN          GPIO_Pin_2      /* PA2 - TIM2_CH3 左电机 */
#define MOTOR_PWMB_PIN          GPIO_Pin_3      /* PA3 - TIM2_CH4 右电机 */

/* 方向控制引脚 */
#define MOTOR_DIR_GPIO_PORT     GPIOE
#define MOTOR_DIR_GPIO_CLK      RCC_APB2Periph_GPIOE
#define MOTOR_AIN1_PIN          GPIO_Pin_2      /* PE2 - 左电机方向1 */
#define MOTOR_AIN2_PIN          GPIO_Pin_3      /* PE3 - 左电机方向2 */
#define MOTOR_BIN1_PIN          GPIO_Pin_4      /* PE4 - 右电机方向1 */
#define MOTOR_BIN2_PIN          GPIO_Pin_5      /* PE5 - 右电机方向2 */
#define MOTOR_STBY_PIN          GPIO_Pin_6      /* PE6 - 待机控制 */

/*====================================================================================*/
/*                                  PWM参数                                           */
/*====================================================================================*/

#define MOTOR_PWM_FREQ          20000   /* PWM频率 20kHz */
#define MOTOR_PWM_PERIOD        3600    /* 72MHz / 20kHz = 3600 */
#define MOTOR_PWM_MAX           3600    /* 最大占空比值 */

/*====================================================================================*/
/*                                  类型定义                                           */
/*====================================================================================*/

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT
} Motor_Index_t;

typedef enum {
    MOTOR_DIR_FORWARD = 0,  /* 正转 */
    MOTOR_DIR_BACKWARD,     /* 反转 */
    MOTOR_DIR_BRAKE         /* 刹车 */
} Motor_Dir_t;

/*====================================================================================*/
/*                                  函数声明                                           */
/*====================================================================================*/

/**
  * @brief  电机驱动初始化
  */
void Motor_Init(void);

/**
  * @brief  设置电机速度
  * @param  motor: MOTOR_LEFT 或 MOTOR_RIGHT
  * @param  speed: 速度值 (-100 ~ +100)，正=正转，负=反转
  */
void Motor_SetSpeed(Motor_Index_t motor, int16_t speed);

/**
  * @brief  设置双电机速度
  * @param  speedL: 左电机速度 (-100 ~ +100)
  * @param  speedR: 右电机速度 (-100 ~ +100)
  */
void Motor_SetSpeedBoth(int16_t speedL, int16_t speedR);

/**
  * @brief  电机刹车
  * @param  motor: MOTOR_LEFT 或 MOTOR_RIGHT
  */
void Motor_Brake(Motor_Index_t motor);

/**
  * @brief  双电机刹车
  */
void Motor_BrakeBoth(void);

/**
  * @brief  电机滑行 (自由停止)
  * @param  motor: MOTOR_LEFT 或 MOTOR_RIGHT
  */
void Motor_Coast(Motor_Index_t motor);

/**
  * @brief  使能/禁用电机驱动
  * @param  enable: 1=使能, 0=禁用(待机)
  */
void Motor_Enable(uint8_t enable);

/**
  * @brief  获取电机是否使能
  * @return 1=使能, 0=禁用
  */
uint8_t Motor_IsEnabled(void);

/* 100% DC on PA2/PA3 as GPIO; bypasses TIM2 for bring-up. */
void Motor_ForceGpioFull(Motor_Index_t motor, int16_t rawSign);
void Motor_RestorePwmAf(void);
uint8_t Motor_GetPwmGpioLevel(Motor_Index_t motor);
uint8_t Motor_GetStbyLevel(void);
uint8_t Motor_GetDirBits(Motor_Index_t motor);

#endif /* __BSP_MOTOR_H */
