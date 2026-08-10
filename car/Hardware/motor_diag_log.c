#include "motor_diag_log.h"

#include "bsp_diag_uart.h"

#define MOTOR_DIAG_LOG_CAPACITY 64U
#define MOTOR_DIAG_WHEEL_NONE   ((L298N_Wheel_t)255U)

typedef struct
{
    uint32_t sequence;
    uint32_t timeMs;
    int16_t command;
    uint16_t gpioEOdr;
    uint16_t gpioCOdr;
    uint16_t gpioAOdr;
    uint16_t gpioEIdr;
    uint16_t gpioCIdr;
    uint16_t gpioAIdr;
    uint16_t gpioGOdr;
    uint16_t gpioGIdr;
    uint16_t tim5Ccr1;
    uint16_t tim5Ccr2;
    uint16_t tim3Ccr1;
    uint16_t tim3Ccr2;
    uint16_t tim2Ccr3;
    uint16_t tim2Ccr4;
    uint8_t event;
    uint8_t wheel;
} MotorDiagLogEntry_t;

static MotorDiagLogEntry_t s_entries[MOTOR_DIAG_LOG_CAPACITY];
static uint32_t s_sequence = 0U;
static uint16_t s_writeIndex = 0U;
static uint16_t s_count = 0U;
static uint16_t s_dropped = 0U;
static uint8_t s_frozen = 0U;

static void MotorDiagLog_WriteHexNibble(uint8_t value)
{
    char ch;

    value &= 0x0FU;
    ch = (value < 10U) ? (char)('0' + value) :
                         (char)('A' + (value - 10U));
    DiagUart_WriteChar(ch);
}

static void MotorDiagLog_WriteHex16(uint16_t value)
{
    MotorDiagLog_WriteHexNibble((uint8_t)(value >> 12));
    MotorDiagLog_WriteHexNibble((uint8_t)(value >> 8));
    MotorDiagLog_WriteHexNibble((uint8_t)(value >> 4));
    MotorDiagLog_WriteHexNibble((uint8_t)value);
}

static void MotorDiagLog_PrintEntry(const MotorDiagLogEntry_t *entry)
{
    DiagUart_WriteString("motor_gpio.log,");
    DiagUart_WriteUInt32(entry->sequence);
    DiagUart_WriteChar(',');
    DiagUart_WriteUInt32(entry->timeMs);
    DiagUart_WriteChar(',');
    DiagUart_WriteUInt32(entry->event);
    DiagUart_WriteChar(',');
    DiagUart_WriteUInt32(entry->wheel);
    DiagUart_WriteChar(',');
    DiagUart_WriteInt32(entry->command);
    DiagUart_WriteString(",0x");
    MotorDiagLog_WriteHex16(entry->gpioEOdr);
    DiagUart_WriteString(",0x");
    MotorDiagLog_WriteHex16(entry->gpioCOdr);
    DiagUart_WriteString(",0x");
    MotorDiagLog_WriteHex16(entry->gpioAOdr);
    DiagUart_WriteString(",0x");
    MotorDiagLog_WriteHex16(entry->gpioEIdr);
    DiagUart_WriteString(",0x");
    MotorDiagLog_WriteHex16(entry->gpioCIdr);
    DiagUart_WriteString(",0x");
    MotorDiagLog_WriteHex16(entry->gpioAIdr);
    DiagUart_WriteString(",0x");
    MotorDiagLog_WriteHex16(entry->gpioGOdr);
    DiagUart_WriteString(",0x");
    MotorDiagLog_WriteHex16(entry->gpioGIdr);
    DiagUart_WriteChar(',');
    DiagUart_WriteUInt32(entry->tim5Ccr1);
    DiagUart_WriteChar(',');
    DiagUart_WriteUInt32(entry->tim5Ccr2);
    DiagUart_WriteChar(',');
    DiagUart_WriteUInt32(entry->tim3Ccr1);
    DiagUart_WriteChar(',');
    DiagUart_WriteUInt32(entry->tim3Ccr2);
    DiagUart_WriteChar(',');
    DiagUart_WriteUInt32(entry->tim2Ccr3);
    DiagUart_WriteChar(',');
    DiagUart_WriteUInt32(entry->tim2Ccr4);
    DiagUart_WriteString("\r\n");
}

void MotorDiagLog_Init(void)
{
    s_sequence = 0U;
    s_writeIndex = 0U;
    s_count = 0U;
    s_dropped = 0U;
    s_frozen = 0U;
}

void MotorDiagLog_Record(uint32_t timeMs,
                         MotorDiagEvent_t event,
                         L298N_Wheel_t wheel,
                         int16_t command)
{
    MotorDiagLogEntry_t *entry;

    if (s_frozen != 0U)
    {
        return;
    }

    entry = &s_entries[s_writeIndex];
    entry->sequence = s_sequence++;
    entry->timeMs = timeMs;
    entry->command = command;
    entry->gpioEOdr = (uint16_t)GPIOE->ODR;
    entry->gpioCOdr = (uint16_t)GPIOC->ODR;
    entry->gpioAOdr = (uint16_t)GPIOA->ODR;
    entry->gpioEIdr = (uint16_t)GPIOE->IDR;
    entry->gpioCIdr = (uint16_t)GPIOC->IDR;
    entry->gpioAIdr = (uint16_t)GPIOA->IDR;
    entry->gpioGOdr = (uint16_t)GPIOG->ODR;
    entry->gpioGIdr = (uint16_t)GPIOG->IDR;
    entry->tim5Ccr1 = (uint16_t)TIM5->CCR1;
    entry->tim5Ccr2 = (uint16_t)TIM5->CCR2;
    entry->tim3Ccr1 = (uint16_t)TIM3->CCR1;
    entry->tim3Ccr2 = (uint16_t)TIM3->CCR2;
    entry->tim2Ccr3 = (uint16_t)TIM2->CCR3;
    entry->tim2Ccr4 = (uint16_t)TIM2->CCR4;
    entry->event = (uint8_t)event;
    entry->wheel = (uint8_t)wheel;

    s_writeIndex = (uint16_t)((s_writeIndex + 1U) %
                              MOTOR_DIAG_LOG_CAPACITY);
    if (s_count < MOTOR_DIAG_LOG_CAPACITY)
    {
        ++s_count;
    }
    else
    {
        ++s_dropped;
    }
}

void MotorDiagLog_Freeze(void)
{
    s_frozen = 1U;
}

void MotorDiagLog_Export(void)
{
    uint16_t i;
    uint16_t start;

    MotorDiagLog_Freeze();
    DiagUart_WriteString("motor_gpio.export begin count=");
    DiagUart_WriteUInt32(s_count);
    DiagUart_WriteString(" dropped=");
    DiagUart_WriteUInt32(s_dropped);
    DiagUart_WriteString(" capacity=");
    DiagUart_WriteUInt32(MOTOR_DIAG_LOG_CAPACITY);
    DiagUart_WriteString("\r\n");
    DiagUart_WriteString("motor_gpio.header,seq,time_ms,event,wheel,cmd,pe_odr,pc_odr,pa_odr,pe_idr,pc_idr,pa_idr,pg_odr,pg_idr,tim5_ccr1,tim5_ccr2,tim3_ccr1,tim3_ccr2,tim2_ccr3,tim2_ccr4\r\n");

    start = (s_count == MOTOR_DIAG_LOG_CAPACITY) ? s_writeIndex : 0U;
    for (i = 0U; i < s_count; ++i)
    {
        uint16_t index = (uint16_t)((start + i) %
                                    MOTOR_DIAG_LOG_CAPACITY);
        MotorDiagLog_PrintEntry(&s_entries[index]);
    }
    DiagUart_WriteString("motor_gpio.export end\r\n");
}

uint8_t MotorDiagLog_PollCommand(void)
{
    char command;

    if (DiagUart_TryReadChar(&command) == 0U)
    {
        return 0U;
    }

    if ((command == 'E') || (command == 'e') ||
        (command == 'D') || (command == 'd'))
    {
        MotorDiagLog_Export();
    }
    else if ((command == 'C') || (command == 'c'))
    {
        MotorDiagLog_Init();
        MotorDiagLog_Record(0U, MOTOR_DIAG_EVENT_COMMAND,
                            MOTOR_DIAG_WHEEL_NONE, 0);
        DiagUart_WriteString("motor_gpio.clear ok\r\n");
    }
    else if ((command == 'H') || (command == 'h') ||
             (command == '?'))
    {
        DiagUart_WriteString("motor_gpio.cmd R=tb6612_left E/D=export C=clear H=help\r\n");
    }
    else if ((command == 'R') || (command == 'r'))
    {
        MotorDiagLog_Init();
        MotorDiagLog_Record(0U, MOTOR_DIAG_EVENT_COMMAND,
                            MOTOR_DIAG_WHEEL_NONE, 0);
        return 1U;
    }

    return 0U;
}
