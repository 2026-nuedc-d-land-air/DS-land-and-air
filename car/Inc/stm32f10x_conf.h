#ifndef __STM32F10X_CONF_H
#define __STM32F10X_CONF_H

/* 本工程当前使用 STM32F10x 标准外设库。 */
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line);
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
#else
#define assert_param(expr) ((void)0U)
#endif

#endif /* __STM32F10X_CONF_H */
