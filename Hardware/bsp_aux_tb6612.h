/**
 ******************************************************************************
 * @file    bsp_aux_tb6612.h
 * @brief   扩展两路 G513 的 TB6612 驱动（原始方向，开环）。
 *
 * 方向/待机：
 *   AIN1=PF1, AIN2=PF2
 *   BIN1=PF3, BIN2=PF4
 *   STBY=PB9
 *
 * 速度 PWM（TIM1 完全重映射）：PWMA=PE13、PWMB=PE14。
 * 编码器相位：左轮 A/B=PB6/PB7，右轮 A/B=PC6/PC7。
 ******************************************************************************
 */

#ifndef __BSP_AUX_TB6612_H
#define __BSP_AUX_TB6612_H

#include "stm32f10x.h"

/* Motor A direction */
#define AUX_TB6612_AIN_GPIO_PORT       GPIOF
#define AUX_TB6612_AIN_GPIO_CLK        RCC_APB2Periph_GPIOF
#define AUX_TB6612_AIN1_PIN            GPIO_Pin_1
#define AUX_TB6612_AIN2_PIN            GPIO_Pin_2

/* Motor B direction */
#define AUX_TB6612_BIN_GPIO_PORT       GPIOF
#define AUX_TB6612_BIN_GPIO_CLK        RCC_APB2Periph_GPIOF
#define AUX_TB6612_BIN1_PIN            GPIO_Pin_3
#define AUX_TB6612_BIN2_PIN            GPIO_Pin_4

/* Standby */
#define AUX_TB6612_STBY_GPIO_PORT      GPIOB
#define AUX_TB6612_STBY_GPIO_CLK       RCC_APB2Periph_GPIOB
#define AUX_TB6612_STBY_PIN            GPIO_Pin_9

/* TIM1 full remap: CH3=PE13 (PWMA), CH4=PE14 (PWMB). */
#define AUX_TB6612_PWM_GPIO_PORT       GPIOE
#define AUX_TB6612_PWM_GPIO_CLK        RCC_APB2Periph_GPIOE
#define AUX_TB6612_PWMA_PIN            GPIO_Pin_13
#define AUX_TB6612_PWMB_PIN            GPIO_Pin_14
#define AUX_TB6612_TIM                 TIM1
#define AUX_TB6612_TIM_CLK             RCC_APB2Periph_TIM1
#define AUX_TB6612_PWM_PERIOD          3600U

/* Rear wheel encoder phases; input only, never driven by this BSP. */
#define AUX_TB6612_LEFT_ENC_GPIO_PORT  GPIOB
#define AUX_TB6612_LEFT_ENC_GPIO_CLK   RCC_APB2Periph_GPIOB
#define AUX_TB6612_LEFT_ENC_A_PIN      GPIO_Pin_6  /* TIM4_CH1 */
#define AUX_TB6612_LEFT_ENC_B_PIN      GPIO_Pin_7  /* TIM4_CH2 */
#define AUX_TB6612_RIGHT_ENC_GPIO_PORT GPIOC
#define AUX_TB6612_RIGHT_ENC_GPIO_CLK  RCC_APB2Periph_GPIOC
#define AUX_TB6612_RIGHT_ENC_A_PIN     GPIO_Pin_6  /* TIM8_CH1 */
#define AUX_TB6612_RIGHT_ENC_B_PIN     GPIO_Pin_7  /* TIM8_CH2 */

typedef enum
{
    AUX_TB6612_MOTOR_A = 0, /* left */
    AUX_TB6612_MOTOR_B      /* right */
} AuxTb6612Motor_t;

void AuxTb6612_Init(void);
void AuxTb6612_Enable(uint8_t enable);
uint8_t AuxTb6612_IsEnabled(void);
void AuxTb6612_SetRawSpeed(AuxTb6612Motor_t motor, int16_t rawPercent);
/* 100% DC on PWMx as GPIO high; bypasses TIM3 to prove pin wiring. */
void AuxTb6612_ForceGpioFull(AuxTb6612Motor_t motor, int16_t rawSign);
void AuxTb6612_RestorePwmAf(void);
void AuxTb6612_StopAll(void);
void AuxTb6612_EncoderPoll(void);
uint16_t AuxTb6612_GetEncoderTransitions(AuxTb6612Motor_t motor);
uint16_t AuxTb6612_GetPwmCompare(AuxTb6612Motor_t motor);
uint8_t AuxTb6612_GetDirBits(AuxTb6612Motor_t motor);
uint8_t AuxTb6612_GetPwmGpioLevel(AuxTb6612Motor_t motor);
uint8_t AuxTb6612_GetStbyLevel(void);

#endif /* __BSP_AUX_TB6612_H */
