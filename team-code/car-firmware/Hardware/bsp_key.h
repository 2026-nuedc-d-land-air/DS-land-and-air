#ifndef __BSP_KEY_H
#define __BSP_KEY_H

#include "stm32f10x.h"

/*====================================================================================*/
/*                                  按键事件定义                                       */
/*====================================================================================*/

/* 按键事件类型 */
typedef enum {
    KEY_EVENT_NONE = 0,     // 无事件
    KEY_EVENT_SHORT,        // 短按事件
    KEY_EVENT_LONG          // 长按事件 (>1s)
} KeyEvent_t;

/* 按键状态 */
typedef enum {
    KEY_STATE_IDLE = 0,     // 空闲状态
    KEY_STATE_DEBOUNCE,     // 消抖中
    KEY_STATE_PRESSED,      // 按下确认
    KEY_STATE_LONG_WAIT     // 等待长按
} KeyState_t;

/*====================================================================================*/
/*                                  配置定义                                           */
/*====================================================================================*/

/* 按键引脚配置 - PC5 */
#define KEY_GPIO_PORT       GPIOC
#define KEY_GPIO_PIN        GPIO_Pin_5
#define KEY_GPIO_CLK        RCC_APB2Periph_GPIOC

/* 时间参数 (单位: ms) */
#define KEY_DEBOUNCE_TIME   20      // 消抖时间
#define KEY_LONG_TIME       800    // 长按阈值

/*====================================================================================*/
/*                                  函数声明                                           */
/*====================================================================================*/

/**
  * @brief  按键初始化
  * @note   配置PC5为上拉输入
  */
void Key_Init(void);

/**
  * @brief  按键扫描（非阻塞）
  * @note   需要在主循环中周期性调用，建议10-50ms调用一次
  * @retval 按键事件: KEY_EVENT_NONE/KEY_EVENT_SHORT/KEY_EVENT_LONG
  */
KeyEvent_t Key_Scan(void);

/**
  * @brief  带时间参数的按键扫描
  * @param  deltaMs: 距离上次调用的时间间隔(ms)
  * @retval 按键事件
  */
KeyEvent_t Key_ScanWithTime(uint32_t deltaMs);

/**
  * @brief  获取按键当前电平状态
  * @retval 0:按下(低电平), 1:释放(高电平)
  */
uint8_t Key_GetLevel(void);

/**
  * @brief  按键状态机复位
  * @note   在需要清除按键状态时调用
  */
void Key_Reset(void);

/**
  * @brief  获取当前状态机状态（调试用）
  * @retval 状态值: 0=IDLE, 1=PRESSED, 2=LONG_WAIT
  */
uint8_t Key_GetState(void);

/**
  * @brief  获取按键按下持续时间
  * @retval 按下时间(ms)，未按下返回0
  */
uint32_t Key_GetPressTime(void);

/**
  * @brief  获取长按计时等级 (1-9)
  * @retval 0=未长按, 1-9=长按等级
  */
uint8_t Key_GetLongPressLevel(void);

#endif /* __BSP_KEY_H */
