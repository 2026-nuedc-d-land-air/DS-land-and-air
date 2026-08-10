/**
  * @file    bsp_led_pwm.h
  * @brief   LED PWM亮度控制模块
  * @note    使用TIM3软件PWM实现LED亮度调节
  *          LED1 (绿色) - PB9
  *          LED2 (红色) - PE0
  *          LED3 (指示) - PE1
  *          低电平点亮
  */

#ifndef __BSP_LED_PWM_H
#define __BSP_LED_PWM_H

#include "stm32f10x.h"

/*====================================================================================*/
/*                                  引脚定义                                           */
/*====================================================================================*/

/* LED1 绿色 - PB9 */
#define LED1_GPIO_PORT      GPIOB
#define LED1_GPIO_PIN       GPIO_Pin_9
#define LED1_GPIO_CLK       RCC_APB2Periph_GPIOB

/* LED2 红色 - PE0 */
#define LED2_GPIO_PORT      GPIOE
#define LED2_GPIO_PIN       GPIO_Pin_0
#define LED2_GPIO_CLK       RCC_APB2Periph_GPIOE

/* LED3 指示灯 - PE1 */
#define LED3_GPIO_PORT      GPIOE
#define LED3_GPIO_PIN       GPIO_Pin_1
#define LED3_GPIO_CLK       RCC_APB2Periph_GPIOE

/*====================================================================================*/
/*                                  类型定义                                           */
/*====================================================================================*/

typedef enum {
    LED_GREEN = 0,      /* PB9 绿色 */
    LED_RED,            /* PE0 红色 */
    LED_INDICATOR,      /* PE1 指示灯 */
    LED_COUNT           /* LED总数 */
} LED_Index_t;

typedef enum {
    BRIGHTNESS_MODE_NORMAL = 0,     /* 正常模式 */
    BRIGHTNESS_MODE_ADJUST          /* 亮度调节模式 */
} BrightnessMode_t;

/*====================================================================================*/
/*                                  常量定义                                           */
/*====================================================================================*/

#define LED_PWM_PERIOD      50     /* PWM周期 */

/*====================================================================================*/
/*                                  函数声明                                           */
/*====================================================================================*/

/**
  * @brief  LED PWM模块初始化
  */
void LED_PWM_Init(void);

/**
  * @brief  设置LED亮度
  * @param  led: LED索引
  * @param  brightness: 亮度 (0-100)
  */
void LED_SetBrightness(LED_Index_t led, uint8_t brightness);

/**
  * @brief  获取LED亮度
  * @param  led: LED索引
  * @retval 亮度值 (0-100)
  */
uint8_t LED_GetBrightness(LED_Index_t led);

/**
  * @brief  LED开关控制
  * @param  led: LED索引
  * @param  state: 0=关, 1=开
  */
void LED_Switch(LED_Index_t led, uint8_t state);

/**
  * @brief  获取LED开关状态
  * @param  led: LED索引
  * @retval 0=关, 1=开
  */
uint8_t LED_GetState(LED_Index_t led);

/**
  * @brief  指示灯闪烁两次 (页面切换时调用)
  */
void LED_IndicatorBlink2(void);

/**
  * @brief  启动避障提示灯效 (两两交替)
  */
void LED_StartObstacleEffect(void);

/**
  * @brief  检查避障提示灯效是否正在播放
  * @retval 1=正在播放, 0=已结束
  */
uint8_t LED_IsObstacleEffectPlaying(void);

/**
  * @brief  避障灯效结束后请求UI刷新，使页面重新接管LED显示
  */
void LED_BindUIRefresh(void (*refreshFn)(void));

/**
  * @brief  进入亮度调节模式
  */
void LED_EnterAdjustMode(void);

/**
  * @brief  退出亮度调节模式
  */
void LED_ExitAdjustMode(void);

/**
  * @brief  切换亮度等级 (短按PC4调用)
  */
void LED_ChangeBrightnessStep(void);

/**
  * @brief  亮度调节模式更新 (主循环调用)
  * @param  deltaMs: 时间间隔(ms)
  */
void LED_AdjustModeUpdate(uint32_t deltaMs);

/**
  * @brief  获取当前调节模式
  * @retval BRIGHTNESS_MODE_NORMAL 或 BRIGHTNESS_MODE_ADJUST
  */
BrightnessMode_t LED_GetAdjustMode(void);

/**
  * @brief  获取保存的亮度值 (用于传递给页面)
  * @param  led: LED_GREEN 或 LED_RED
  * @retval 亮度值 (0-100)
  */
uint8_t LED_GetSavedBrightness(LED_Index_t led);

/**
  * @brief  启动终点流水灯效果
  * @note   配合蜂鸣器三段升调，每段每个LED快速闪烁两次
  */
void LED_StartFinishEffect(void);

/**
  * @brief  检查终点效果是否正在播放
  * @retval 1=正在播放, 0=已结束
  */
uint8_t LED_IsFinishEffectPlaying(void);

#endif /* __BSP_LED_PWM_H */
