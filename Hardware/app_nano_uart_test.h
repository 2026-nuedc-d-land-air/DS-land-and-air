#ifndef __APP_NANO_UART_TEST_H
#define __APP_NANO_UART_TEST_H

#include "stm32f10x.h"

void NanoUartTest_Init(uint32_t nowMs);
void NanoUartTest_HandleByte(uint8_t byte, uint32_t nowMs);
void NanoUartTest_HandleUartErrors(uint16_t flags, uint32_t nowMs);
void NanoUartTest_Update(uint32_t nowMs);

#endif /* __APP_NANO_UART_TEST_H */
