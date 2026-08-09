#ifndef __BSP_DIAG_UART_H
#define __BSP_DIAG_UART_H

#include "stm32f10x.h"

/* USART1 remap: PB6=MCU TX -> adapter RX, PB7=MCU RX <- adapter TX. */
void DiagUart_Init(uint32_t baudrate);
void DiagUart_WriteChar(char ch);
void DiagUart_WriteString(const char *text);
void DiagUart_WriteInt32(int32_t value);
void DiagUart_WriteUInt32(uint32_t value);
uint8_t DiagUart_TryReadChar(char *ch);

#endif /* __BSP_DIAG_UART_H */
