#include "nano_uart_diag_log.h"

#include "bsp_diag_uart.h"

#define NANO_UART_DIAG_LOG_CAPACITY 128U

typedef struct
{
    uint32_t sequence;
    uint32_t timeMs;
    uint32_t value;
    uint16_t detail;
    uint8_t event;
    uint8_t reserved;
} NanoUartDiagEntry_t;

static NanoUartDiagEntry_t s_entries[NANO_UART_DIAG_LOG_CAPACITY];
static uint32_t s_sequence;
static uint32_t s_dropped;
static uint16_t s_writeIndex;
static uint16_t s_count;
static uint8_t s_frozen;

static const char *NanoUartDiagLog_EventName(uint8_t event)
{
    switch ((NanoUartDiagEvent_t)event)
    {
        case NANO_UART_DIAG_EVENT_BOOT: return "BOOT";
        case NANO_UART_DIAG_EVENT_RX_LINE: return "RX_LINE";
        case NANO_UART_DIAG_EVENT_PING: return "PING";
        case NANO_UART_DIAG_EVENT_ECHO: return "ECHO";
        case NANO_UART_DIAG_EVENT_VISION: return "VISION";
        case NANO_UART_DIAG_EVENT_STATUS: return "STATUS";
        case NANO_UART_DIAG_EVENT_HEARTBEAT: return "HEARTBEAT";
        case NANO_UART_DIAG_EVENT_PARSE_ERROR: return "PARSE_ERROR";
        case NANO_UART_DIAG_EVENT_LINE_OVERFLOW: return "LINE_OVERFLOW";
        case NANO_UART_DIAG_EVENT_NON_ASCII: return "NON_ASCII";
        case NANO_UART_DIAG_EVENT_UART_ERROR: return "UART_ERROR";
        case NANO_UART_DIAG_EVENT_COMMAND: return "COMMAND";
        default: return "UNKNOWN";
    }
}

void NanoUartDiagLog_Init(uint32_t nowMs)
{
    s_sequence = 0U;
    s_dropped = 0U;
    s_writeIndex = 0U;
    s_count = 0U;
    s_frozen = 0U;
    NanoUartDiagLog_Record(nowMs, NANO_UART_DIAG_EVENT_BOOT, 0U, 0U);
}

void NanoUartDiagLog_Record(uint32_t nowMs,
                            NanoUartDiagEvent_t event,
                            uint32_t value,
                            uint16_t detail)
{
    NanoUartDiagEntry_t *entry;

    if (s_frozen != 0U)
    {
        return;
    }

    entry = &s_entries[s_writeIndex];
    entry->sequence = s_sequence++;
    entry->timeMs = nowMs;
    entry->value = value;
    entry->detail = detail;
    entry->event = (uint8_t)event;
    entry->reserved = 0U;

    s_writeIndex = (uint16_t)((s_writeIndex + 1U) %
                              NANO_UART_DIAG_LOG_CAPACITY);
    if (s_count < NANO_UART_DIAG_LOG_CAPACITY)
    {
        ++s_count;
    }
    else
    {
        ++s_dropped;
    }
}

void NanoUartDiagLog_Freeze(void)
{
    s_frozen = 1U;
}

void NanoUartDiagLog_Export(void)
{
    uint16_t i;
    uint16_t start;

    NanoUartDiagLog_Freeze();
    DiagUart_WriteString("nano_uart.export begin tag=nano_uart_w0 count=");
    DiagUart_WriteUInt32(s_count);
    DiagUart_WriteString(" dropped=");
    DiagUart_WriteUInt32(s_dropped);
    DiagUart_WriteString(" capacity=");
    DiagUart_WriteUInt32(NANO_UART_DIAG_LOG_CAPACITY);
    DiagUart_WriteString(" frozen=1\r\n");
    DiagUart_WriteString(
        "nano_uart.header,seq,time_ms,event,value,detail\r\n");

    start = (s_count == NANO_UART_DIAG_LOG_CAPACITY) ? s_writeIndex : 0U;
    for (i = 0U; i < s_count; ++i)
    {
        uint16_t index = (uint16_t)((start + i) %
                                    NANO_UART_DIAG_LOG_CAPACITY);
        const NanoUartDiagEntry_t *entry = &s_entries[index];

        DiagUart_WriteString("nano_uart.log,");
        DiagUart_WriteUInt32(entry->sequence);
        DiagUart_WriteChar(',');
        DiagUart_WriteUInt32(entry->timeMs);
        DiagUart_WriteChar(',');
        DiagUart_WriteString(NanoUartDiagLog_EventName(entry->event));
        DiagUart_WriteChar(',');
        DiagUart_WriteUInt32(entry->value);
        DiagUart_WriteChar(',');
        DiagUart_WriteUInt32(entry->detail);
        DiagUart_WriteString("\r\n");
    }

    DiagUart_WriteString("nano_uart.export end\r\n");
}

void NanoUartDiagLog_PollCommand(uint32_t nowMs)
{
    char command;

    if (DiagUart_TryReadChar(&command) == 0U)
    {
        return;
    }

    if ((command == 'E') || (command == 'e') ||
        (command == 'D') || (command == 'd'))
    {
        NanoUartDiagLog_Record(nowMs, NANO_UART_DIAG_EVENT_COMMAND,
                               (uint32_t)(uint8_t)command, 0U);
        NanoUartDiagLog_Export();
    }
    else if ((command == 'F') || (command == 'f'))
    {
        NanoUartDiagLog_Record(nowMs, NANO_UART_DIAG_EVENT_COMMAND,
                               (uint32_t)(uint8_t)command, 0U);
        NanoUartDiagLog_Freeze();
        DiagUart_WriteString("nano_uart.freeze ok\r\n");
    }
    else if ((command == 'C') || (command == 'c'))
    {
        NanoUartDiagLog_Init(nowMs);
        NanoUartDiagLog_Record(nowMs, NANO_UART_DIAG_EVENT_COMMAND,
                               (uint32_t)(uint8_t)command, 0U);
        DiagUart_WriteString("nano_uart.clear ok\r\n");
    }
    else if ((command == 'H') || (command == 'h') || (command == '?'))
    {
        DiagUart_WriteString(
            "nano_uart.cmd E/D=freeze_export F=freeze C=clear H=help\r\n");
    }
}
