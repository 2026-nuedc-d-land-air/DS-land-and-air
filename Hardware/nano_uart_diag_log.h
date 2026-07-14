#ifndef __NANO_UART_DIAG_LOG_H
#define __NANO_UART_DIAG_LOG_H

#include "stm32f10x.h"

typedef enum
{
    NANO_UART_DIAG_EVENT_BOOT = 0,
    NANO_UART_DIAG_EVENT_RX_LINE,
    NANO_UART_DIAG_EVENT_PING,
    NANO_UART_DIAG_EVENT_ECHO,
    NANO_UART_DIAG_EVENT_VISION,
    NANO_UART_DIAG_EVENT_STATUS,
    NANO_UART_DIAG_EVENT_HEARTBEAT,
    NANO_UART_DIAG_EVENT_PARSE_ERROR,
    NANO_UART_DIAG_EVENT_LINE_OVERFLOW,
    NANO_UART_DIAG_EVENT_NON_ASCII,
    NANO_UART_DIAG_EVENT_UART_ERROR,
    NANO_UART_DIAG_EVENT_COMMAND
} NanoUartDiagEvent_t;

void NanoUartDiagLog_Init(uint32_t nowMs);
void NanoUartDiagLog_Record(uint32_t nowMs,
                            NanoUartDiagEvent_t event,
                            uint32_t value,
                            uint16_t detail);
void NanoUartDiagLog_Freeze(void);
void NanoUartDiagLog_Export(void);
void NanoUartDiagLog_PollCommand(uint32_t nowMs);

#endif /* __NANO_UART_DIAG_LOG_H */
