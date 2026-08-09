/**
 * @file bsp_timer.h
 * @brief TIM4 定时器驱动 (已废弃 - 2026/01/16)
 * @note 此文件已不再使用，精确计时功能已迁移到 bsp_systick.h (使用TIM2)
 */

#ifndef __BSP_TIMER_H__
#define __BSP_TIMER_H__

#include "stm32f10x.h"
#include "AllHeader.h"

#if 0  // ========== 以下声明已废弃 ==========

void TIM4_Init(void);
void my_delay_10ms(u16 time);

#endif // ========== 废弃声明结束 ==========

#endif
