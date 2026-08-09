#include "app_nano_uart_test.h"

#include <string.h>

#include "bsp_diag_uart.h"
#include "bsp_robot_uart.h"
#include "nano_uart_diag_log.h"

#define NANO_UART_LINE_CAPACITY 128U
#define NANO_UART_STATS_PERIOD_MS 1000U

#ifndef NANO_UART_TRACE_BYTES
#define NANO_UART_TRACE_BYTES 0
#endif

static char s_line[NANO_UART_LINE_CAPACITY];
static uint16_t s_lineLength;
static uint8_t s_dropUntilNewline;
static uint32_t s_rxBytes;
static uint32_t s_rxLines;
static uint32_t s_txLines;
static uint32_t s_parseErrors;
static uint32_t s_overflowErrors;
static uint32_t s_emptyLines;
static uint32_t s_okLines;
static uint32_t s_uartParityErrors;
static uint32_t s_uartFramingErrors;
static uint32_t s_uartNoiseErrors;
static uint32_t s_uartOverrunErrors;
static uint32_t s_lastRxMs;
static uint32_t s_lastStatsMs;

static uint8_t NanoUartTest_ParseUInt32(const char *text,
                                        uint16_t length,
                                        uint32_t *value)
{
    uint32_t result = 0U;
    uint16_t i;

    if ((text == 0) || (value == 0) || (length == 0U))
    {
        return 0U;
    }

    for (i = 0U; i < length; ++i)
    {
        uint32_t digit;
        if ((text[i] < '0') || (text[i] > '9'))
        {
            return 0U;
        }
        digit = (uint32_t)(text[i] - '0');
        if (result > ((0xFFFFFFFFU - digit) / 10U))
        {
            return 0U;
        }
        result = (result * 10U) + digit;
    }

    *value = result;
    return 1U;
}

static uint8_t NanoUartTest_ValidateVisionFields(const char *text)
{
    uint8_t separators = 0U;
    uint8_t fieldLength = 0U;

    while (*text != '\0')
    {
        if (*text == ',')
        {
            if (fieldLength == 0U)
            {
                return 0U;
            }
            ++separators;
            fieldLength = 0U;
        }
        else
        {
            ++fieldLength;
        }
        ++text;
    }

    return ((separators == 6U) && (fieldLength != 0U)) ? 1U : 0U;
}

static void NanoUartTest_WriteUInt32Both(uint32_t value)
{
    char buffer[10];
    uint8_t length = 0U;

    do
    {
        buffer[length++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);

    while (length != 0U)
    {
        char ch = buffer[--length];
        RobotUart_NanoWriteByte((uint8_t)ch);
        DiagUart_WriteChar(ch);
    }
}

static void NanoUartTest_TxBegin(uint32_t nowMs)
{
    DiagUart_WriteString("NANO_TX,");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteChar(',');
}

static void NanoUartTest_TxText(const char *text)
{
    RobotUart_NanoWriteString(text);
    DiagUart_WriteString(text);
}

static void NanoUartTest_TxEnd(void)
{
    RobotUart_NanoWriteString("\r\n");
    DiagUart_WriteString("\r\n");
    ++s_txLines;
}

static void NanoUartTest_SendError(uint32_t nowMs, const char *reason)
{
    ++s_parseErrors;
    NanoUartTest_TxBegin(nowMs);
    NanoUartTest_TxText("ERR,");
    NanoUartTest_TxText(reason);
    NanoUartTest_TxEnd();
}

static void NanoUartTest_LogRxLine(uint32_t nowMs, const char *line)
{
    DiagUart_WriteString("NANO_RX_LINE,");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteChar(',');
    DiagUart_WriteString(line);
    DiagUart_WriteString("\r\n");
}

#if NANO_UART_TRACE_BYTES
static void NanoUartTest_LogRxByte(uint32_t nowMs, uint8_t byte)
{
    static const char hex[] = "0123456789ABCDEF";

    DiagUart_WriteString("NANO_RX_BYTE,");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",0x");
    DiagUart_WriteChar(hex[(byte >> 4) & 0x0FU]);
    DiagUart_WriteChar(hex[byte & 0x0FU]);
    DiagUart_WriteString("\r\n");
}
#endif

static void NanoUartTest_HandleLine(uint32_t nowMs)
{
    const char *argument;
    const char *separator;
    uint32_t sequence;

    s_line[s_lineLength] = '\0';
    ++s_rxLines;
    NanoUartTest_LogRxLine(nowMs, s_line);
    NanoUartDiagLog_Record(nowMs, NANO_UART_DIAG_EVENT_RX_LINE,
                           s_lineLength, 0U);

    if (strncmp(s_line, "PING,", 5U) == 0)
    {
        argument = &s_line[5];
        if (NanoUartTest_ParseUInt32(argument,
                                     (uint16_t)strlen(argument),
                                     &sequence) == 0U)
        {
            NanoUartDiagLog_Record(nowMs,
                                   NANO_UART_DIAG_EVENT_PARSE_ERROR,
                                   0U, 1U);
            NanoUartTest_SendError(nowMs, "PING_SEQ_INVALID");
            return;
        }

        ++s_okLines;
        NanoUartDiagLog_Record(nowMs, NANO_UART_DIAG_EVENT_PING,
                               sequence, s_lineLength);
        NanoUartTest_TxBegin(nowMs);
        NanoUartTest_TxText("PONG,");
        NanoUartTest_WriteUInt32Both(sequence);
        NanoUartTest_TxText(",");
        NanoUartTest_WriteUInt32Both(nowMs);
        NanoUartTest_TxEnd();
        return;
    }

    if (strncmp(s_line, "ECHO,", 5U) == 0)
    {
        ++s_okLines;
        NanoUartDiagLog_Record(nowMs, NANO_UART_DIAG_EVENT_ECHO,
                               (uint32_t)strlen(&s_line[5]), s_lineLength);
        NanoUartTest_TxBegin(nowMs);
        NanoUartTest_TxText("ECHO_ACK,");
        NanoUartTest_TxText(&s_line[5]);
        NanoUartTest_TxEnd();
        return;
    }

    if (strncmp(s_line, "VISION,", 7U) == 0)
    {
        argument = &s_line[7];
        separator = strchr(argument, ',');
        if ((separator == 0) ||
            (NanoUartTest_ValidateVisionFields(argument) == 0U) ||
            (NanoUartTest_ParseUInt32(argument,
                                      (uint16_t)(separator - argument),
                                      &sequence) == 0U))
        {
            NanoUartDiagLog_Record(nowMs,
                                   NANO_UART_DIAG_EVENT_PARSE_ERROR,
                                   0U, 2U);
            NanoUartTest_SendError(nowMs, "VISION_FORMAT");
            return;
        }

        ++s_okLines;
        NanoUartDiagLog_Record(nowMs, NANO_UART_DIAG_EVENT_VISION,
                               sequence, s_lineLength);
        NanoUartTest_TxBegin(nowMs);
        NanoUartTest_TxText("VISION_ACK,");
        NanoUartTest_WriteUInt32Both(sequence);
        NanoUartTest_TxEnd();
        return;
    }

    if (strcmp(s_line, "STATUS") == 0)
    {
        ++s_okLines;
        NanoUartDiagLog_Record(nowMs, NANO_UART_DIAG_EVENT_STATUS,
                               s_rxBytes, s_lineLength);
        NanoUartTest_TxBegin(nowMs);
        NanoUartTest_TxText("STATUS,");
        NanoUartTest_WriteUInt32Both(nowMs);
        NanoUartTest_TxText(",rx_bytes=");
        NanoUartTest_WriteUInt32Both(s_rxBytes);
        NanoUartTest_TxText(",rx_lines=");
        NanoUartTest_WriteUInt32Both(s_rxLines);
        NanoUartTest_TxText(",parse_err=");
        NanoUartTest_WriteUInt32Both(s_parseErrors);
        NanoUartTest_TxText(",motors_safe=1");
        NanoUartTest_TxEnd();
        return;
    }

    NanoUartDiagLog_Record(nowMs, NANO_UART_DIAG_EVENT_PARSE_ERROR,
                           0U, 3U);
    NanoUartTest_SendError(nowMs, "UNKNOWN_COMMAND");
}

void NanoUartTest_Init(uint32_t nowMs)
{
    s_lineLength = 0U;
    s_dropUntilNewline = 0U;
    s_rxBytes = 0U;
    s_rxLines = 0U;
    s_txLines = 0U;
    s_parseErrors = 0U;
    s_overflowErrors = 0U;
    s_emptyLines = 0U;
    s_okLines = 0U;
    s_uartParityErrors = 0U;
    s_uartFramingErrors = 0U;
    s_uartNoiseErrors = 0U;
    s_uartOverrunErrors = 0U;
    s_lastRxMs = nowMs;
    s_lastStatsMs = nowMs;

    RobotUart_NanoWriteString("STM32_READY,nano_uart_test,115200,motors_safe=1\r\n");
    DiagUart_WriteString("NANO_TX,");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",STM32_READY,nano_uart_test,115200,motors_safe=1\r\n");
    ++s_txLines;
}

void NanoUartTest_HandleByte(uint8_t byte, uint32_t nowMs)
{
    ++s_rxBytes;
    s_lastRxMs = nowMs;

#if NANO_UART_TRACE_BYTES
    NanoUartTest_LogRxByte(nowMs, byte);
#endif

    if (byte == (uint8_t)'\r')
    {
        return;
    }

    if (byte == (uint8_t)'\n')
    {
        if (s_dropUntilNewline != 0U)
        {
            s_dropUntilNewline = 0U;
            s_lineLength = 0U;
            ++s_overflowErrors;
            NanoUartDiagLog_Record(nowMs,
                                   NANO_UART_DIAG_EVENT_LINE_OVERFLOW,
                                   s_rxBytes, 0U);
            DiagUart_WriteString("FAULT,");
            DiagUart_WriteUInt32(nowMs);
            DiagUart_WriteString(",NANO_LINE_TOO_LONG\r\n");
            NanoUartTest_SendError(nowMs, "LINE_TOO_LONG");
            return;
        }

        if (s_lineLength != 0U)
        {
            NanoUartTest_HandleLine(nowMs);
            s_lineLength = 0U;
        }
        else
        {
            ++s_emptyLines;
        }
        return;
    }

    if (s_dropUntilNewline != 0U)
    {
        return;
    }

    if ((byte < 0x20U) || (byte > 0x7EU))
    {
        ++s_parseErrors;
        NanoUartDiagLog_Record(nowMs, NANO_UART_DIAG_EVENT_NON_ASCII,
                               byte, s_lineLength);
        DiagUart_WriteString("FAULT,");
        DiagUart_WriteUInt32(nowMs);
        DiagUart_WriteString(",NANO_NON_ASCII\r\n");
        return;
    }

    if (s_lineLength >= (NANO_UART_LINE_CAPACITY - 1U))
    {
        s_dropUntilNewline = 1U;
        return;
    }

    s_line[s_lineLength++] = (char)byte;
}

void NanoUartTest_HandleUartErrors(uint16_t flags, uint32_t nowMs)
{
    if ((flags & ROBOT_UART_ERROR_PARITY) != 0U)
    {
        ++s_uartParityErrors;
    }
    if ((flags & ROBOT_UART_ERROR_FRAMING) != 0U)
    {
        ++s_uartFramingErrors;
    }
    if ((flags & ROBOT_UART_ERROR_NOISE) != 0U)
    {
        ++s_uartNoiseErrors;
    }
    if ((flags & ROBOT_UART_ERROR_OVERRUN) != 0U)
    {
        ++s_uartOverrunErrors;
    }

    NanoUartDiagLog_Record(nowMs, NANO_UART_DIAG_EVENT_UART_ERROR,
                           flags, 0U);
    DiagUart_WriteString("FAULT,");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",NANO_UART,flags=");
    DiagUart_WriteUInt32(flags);
    DiagUart_WriteString("\r\n");
}

void NanoUartTest_Update(uint32_t nowMs)
{
    if ((uint32_t)(nowMs - s_lastStatsMs) < NANO_UART_STATS_PERIOD_MS)
    {
        return;
    }

    s_lastStatsMs = nowMs;

    NanoUartTest_TxBegin(nowMs);
    NanoUartTest_TxText("HEARTBEAT,");
    NanoUartTest_WriteUInt32Both(nowMs);
    NanoUartTest_TxText(",rx_bytes=");
    NanoUartTest_WriteUInt32Both(s_rxBytes);
    NanoUartTest_TxText(",motors_safe=1");
    NanoUartTest_TxEnd();
    NanoUartDiagLog_Record(nowMs, NANO_UART_DIAG_EVENT_HEARTBEAT,
                           s_rxBytes, 0U);

    DiagUart_WriteString("NANO_STAT,");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",rx_bytes=");
    DiagUart_WriteUInt32(s_rxBytes);
    DiagUart_WriteString(",rx_lines=");
    DiagUart_WriteUInt32(s_rxLines);
    DiagUart_WriteString(",ok_lines=");
    DiagUart_WriteUInt32(s_okLines);
    DiagUart_WriteString(",tx_lines=");
    DiagUart_WriteUInt32(s_txLines);
    DiagUart_WriteString(",parse_err=");
    DiagUart_WriteUInt32(s_parseErrors);
    DiagUart_WriteString(",overflow=");
    DiagUart_WriteUInt32(s_overflowErrors);
    DiagUart_WriteString(",empty=");
    DiagUart_WriteUInt32(s_emptyLines);
    DiagUart_WriteString(",uart_pe=");
    DiagUart_WriteUInt32(s_uartParityErrors);
    DiagUart_WriteString(",uart_fe=");
    DiagUart_WriteUInt32(s_uartFramingErrors);
    DiagUart_WriteString(",uart_ne=");
    DiagUart_WriteUInt32(s_uartNoiseErrors);
    DiagUart_WriteString(",uart_ore=");
    DiagUart_WriteUInt32(s_uartOverrunErrors);
    DiagUart_WriteString(",last_rx_age_ms=");
    DiagUart_WriteUInt32((uint32_t)(nowMs - s_lastRxMs));
    DiagUart_WriteString(",motors_safe=1\r\n");
}
