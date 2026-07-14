/**
  * @file    bsp_systick.h
  * @brief   系统滴答定时器 - 精准计时模块
  * @note    使用 TIM2 实现 1ms 中断，提供精准的时间基准
  */

#ifndef __BSP_SYSTICK_H
#define __BSP_SYSTICK_H

#include "stm32f10x.h"

/*====================================================================================*/
/*                                  时间结构体                                         */
/*====================================================================================*/

typedef struct {
    volatile uint32_t milliseconds;   /* 毫秒计数 (0-999) */
    volatile uint32_t seconds;        /* 秒计数 (0-59) */
    volatile uint32_t minutes;        /* 分钟计数 (0-59) */
    volatile uint32_t hours;          /* 小时计数 */
    volatile uint32_t totalMs;        /* 总毫秒数 (用于计算时间差) */
} SysTick_Time_t;

/*====================================================================================*/
/*                                  外部变量                                           */
/*====================================================================================*/

extern volatile SysTick_Time_t g_sysTime;

/*====================================================================================*/
/*                                  函数声明                                           */
/*====================================================================================*/

/**
  * @brief  初始化系统滴答定时器 (TIM2, 1ms中断)
  */
void SysTick_Init(void);

/**
  * @brief  获取当前毫秒数
  * @return 从启动到现在的总毫秒数
  */
uint32_t SysTick_GetMs(void);

/**
  * @brief  获取格式化的时间字符串
  * @param  buf: 输出缓冲区 (至少6字节: "MM:SS\0")
  */
void SysTick_GetTimeString(char *buf);

/**
  * @brief  重置计时器
  */
void SysTick_Reset(void);

#endif /* __BSP_SYSTICK_H */
