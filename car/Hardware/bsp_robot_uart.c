#include "bsp_robot_uart.h"

#include "misc.h"

#define ROBOT_UART_RADAR_RX_RING_SIZE 256U
#define ROBOT_UART_NANO_RX_RING_SIZE  256U

static volatile uint16_t s_nanoErrorFlags;
static volatile uint8_t s_nanoRxRing[ROBOT_UART_NANO_RX_RING_SIZE];
static volatile uint16_t s_nanoRxHead;
static volatile uint16_t s_nanoRxTail;
static volatile uint32_t s_nanoRxOverflowCount;
static volatile uint16_t s_radarErrorFlags;
static volatile uint8_t s_radarRxRing[ROBOT_UART_RADAR_RX_RING_SIZE];
static volatile uint16_t s_radarRxHead;
static volatile uint16_t s_radarRxTail;
static volatile uint32_t s_radarRxOverflowCount;

static uint16_t RobotUart_RadarRingNext(uint16_t index)
{
    ++index;
    return (index >= ROBOT_UART_RADAR_RX_RING_SIZE) ? 0U : index;
}

static uint16_t RobotUart_NanoRingNext(uint16_t index)
{
    ++index;
    return (index >= ROBOT_UART_NANO_RX_RING_SIZE) ? 0U : index;
}

static void RobotUart_NanoRecordErrors(uint16_t status)
{
    if ((status & USART_FLAG_PE) != 0U)
    {
        s_nanoErrorFlags |= ROBOT_UART_ERROR_PARITY;
    }
    if ((status & USART_FLAG_FE) != 0U)
    {
        s_nanoErrorFlags |= ROBOT_UART_ERROR_FRAMING;
    }
    if ((status & USART_FLAG_NE) != 0U)
    {
        s_nanoErrorFlags |= ROBOT_UART_ERROR_NOISE;
    }
    if ((status & USART_FLAG_ORE) != 0U)
    {
        s_nanoErrorFlags |= ROBOT_UART_ERROR_OVERRUN;
    }
}

static void RobotUart_NanoQueueByte(uint8_t byte)
{
    uint16_t next = RobotUart_NanoRingNext(s_nanoRxHead);

    if (next == s_nanoRxTail)
    {
        ++s_nanoRxOverflowCount;
        s_nanoErrorFlags |= ROBOT_UART_ERROR_RING_OVERFLOW;
        return;
    }

    s_nanoRxRing[s_nanoRxHead] = byte;
    s_nanoRxHead = next;
}

static void RobotUart_RadarRecordErrors(uint16_t status)
{
    if ((status & USART_FLAG_PE) != 0U)
    {
        s_radarErrorFlags |= ROBOT_UART_ERROR_PARITY;
    }
    if ((status & USART_FLAG_FE) != 0U)
    {
        s_radarErrorFlags |= ROBOT_UART_ERROR_FRAMING;
    }
    if ((status & USART_FLAG_NE) != 0U)
    {
        s_radarErrorFlags |= ROBOT_UART_ERROR_NOISE;
    }
    if ((status & USART_FLAG_ORE) != 0U)
    {
        s_radarErrorFlags |= ROBOT_UART_ERROR_OVERRUN;
    }
}

static void RobotUart_RadarQueueByte(uint8_t byte)
{
    uint16_t next = RobotUart_RadarRingNext(s_radarRxHead);

    if (next == s_radarRxTail)
    {
        ++s_radarRxOverflowCount;
        s_radarErrorFlags |= ROBOT_UART_ERROR_RING_OVERFLOW;
        return;
    }

    s_radarRxRing[s_radarRxHead] = byte;
    s_radarRxHead = next;
}

static void RobotUart_InitPeripheral(USART_TypeDef *uart, uint32_t baudrate)
{
    USART_InitTypeDef usart;

    USART_DeInit(uart);
    USART_StructInit(&usart);
    usart.USART_BaudRate = baudrate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(uart, &usart);
    USART_Cmd(uart, ENABLE);
}

static void RobotUart_WriteByte(USART_TypeDef *uart, uint8_t byte)
{
    while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET)
    {
    }
    USART_SendData(uart, byte);
}

static void RobotUart_WriteBuffer(USART_TypeDef *uart,
                                  const uint8_t *data,
                                  uint16_t length)
{
    if (data == 0)
    {
        return;
    }

    while (length-- != 0U)
    {
        RobotUart_WriteByte(uart, *data++);
    }

    while (USART_GetFlagStatus(uart, USART_FLAG_TC) == RESET)
    {
    }
}

static void RobotUart_WriteString(USART_TypeDef *uart, const char *text)
{
    if (text == 0)
    {
        return;
    }

    while (*text != '\0')
    {
        RobotUart_WriteByte(uart, (uint8_t)*text++);
    }

    while (USART_GetFlagStatus(uart, USART_FLAG_TC) == RESET)
    {
    }
}

static uint8_t RobotUart_TryReadByte(USART_TypeDef *uart, uint8_t *byte)
{
    if ((byte == 0) ||
        (USART_GetFlagStatus(uart, USART_FLAG_RXNE) == RESET))
    {
        return 0U;
    }

    *byte = (uint8_t)USART_ReceiveData(uart);
    return 1U;
}

void RobotUart_ServoInit(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |
                           RCC_APB2Periph_AFIO, ENABLE);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &gpio);

    RobotUart_InitPeripheral(UART4, baudrate);
}

void RobotUart_ServoWriteByte(uint8_t byte)
{
    RobotUart_WriteByte(UART4, byte);
}

void RobotUart_ServoWriteBuffer(const uint8_t *data, uint16_t length)
{
    RobotUart_WriteBuffer(UART4, data, length);
}

void RobotUart_ServoWriteString(const char *text)
{
    RobotUart_WriteString(UART4, text);
}

uint8_t RobotUart_ServoTryReadByte(uint8_t *byte)
{
    return RobotUart_TryReadByte(UART4, byte);
}

void RobotUart_RadarInit(uint32_t baudrate)
{
    NVIC_InitTypeDef nvic;

    RobotUart_ServoInit(baudrate);
    USART_ITConfig(UART4, USART_IT_RXNE, DISABLE);
    s_radarErrorFlags = 0U;
    s_radarRxHead = 0U;
    s_radarRxTail = 0U;
    s_radarRxOverflowCount = 0U;
    (void)UART4->SR;
    (void)UART4->DR;

    /* UART5 LoRa TX and diagnostic TX can block foreground code longer than
     * a UART4 byte time. Buffer Pi pose bytes in RXNE IRQ so an entire
     * 14-byte pose burst survives unrelated output. */
    nvic.NVIC_IRQChannel = UART4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1U;
    nvic.NVIC_IRQChannelSubPriority = 0U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
}

void RobotUart_RadarWriteByte(uint8_t byte)
{
    RobotUart_WriteByte(UART4, byte);
}

void RobotUart_RadarWriteBuffer(const uint8_t *data, uint16_t length)
{
    RobotUart_WriteBuffer(UART4, data, length);
}

uint8_t RobotUart_RadarTryReadByte(uint8_t *byte)
{
    if (byte == 0)
    {
        return 0U;
    }

    if (s_radarRxTail == s_radarRxHead)
    {
        return 0U;
    }

    *byte = s_radarRxRing[s_radarRxTail];
    s_radarRxTail = RobotUart_RadarRingNext(s_radarRxTail);
    return 1U;
}

uint16_t RobotUart_RadarConsumeErrorFlags(void)
{
    uint16_t flags = s_radarErrorFlags;
    s_radarErrorFlags = 0U;
    return flags;
}

uint32_t RobotUart_RadarConsumeOverflowCount(void)
{
    uint32_t count = s_radarRxOverflowCount;
    s_radarRxOverflowCount = 0U;
    return count;
}

void UART4_IRQHandler(void)
{
    uint16_t status = UART4->SR;

    RobotUart_RadarRecordErrors(status);
    if ((status & USART_FLAG_RXNE) != 0U)
    {
        RobotUart_RadarQueueByte((uint8_t)UART4->DR);
    }
    else if ((status & (USART_FLAG_PE | USART_FLAG_FE |
                        USART_FLAG_NE | USART_FLAG_ORE)) != 0U)
    {
        (void)UART4->DR;
    }
}

void RobotUart_BluetoothInit(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD |
                           RCC_APB2Periph_AFIO, ENABLE);

    GPIO_PinRemapConfig(GPIO_FullRemap_USART3, ENABLE);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_8;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &gpio);

    RobotUart_InitPeripheral(USART3, baudrate);
}

void RobotUart_BluetoothWriteByte(uint8_t byte)
{
    RobotUart_WriteByte(USART3, byte);
}

void RobotUart_BluetoothWriteString(const char *text)
{
    RobotUart_WriteString(USART3, text);
}

uint8_t RobotUart_BluetoothTryReadByte(uint8_t *byte)
{
    return RobotUart_TryReadByte(USART3, byte);
}

void RobotUart_NanoInit(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio;
    NVIC_InitTypeDef nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |
                           RCC_APB2Periph_GPIOD |
                           RCC_APB2Periph_AFIO, ENABLE);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_12;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &gpio);

    RobotUart_InitPeripheral(UART5, baudrate);
    USART_ITConfig(UART5, USART_IT_RXNE, DISABLE);
    s_nanoErrorFlags = 0U;
    s_nanoRxHead = 0U;
    s_nanoRxTail = 0U;
    s_nanoRxOverflowCount = 0U;
    (void)UART5->SR;
    (void)UART5->DR;

    /* Air ACK frames may arrive while foreground code is sending a verbose
     * diagnostic line. Buffer UART5 in RXNE IRQ so that output cannot cover
     * the 30-55 ms air response window. */
    nvic.NVIC_IRQChannel = UART5_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1U;
    nvic.NVIC_IRQChannelSubPriority = 1U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
}

void RobotUart_NanoWriteByte(uint8_t byte)
{
    RobotUart_WriteByte(UART5, byte);
}

void RobotUart_NanoWriteBuffer(const uint8_t *data, uint16_t length)
{
    RobotUart_WriteBuffer(UART5, data, length);
}

void RobotUart_NanoWriteString(const char *text)
{
    RobotUart_WriteString(UART5, text);
}

uint8_t RobotUart_NanoTryReadByte(uint8_t *byte)
{
    if (byte == 0)
    {
        return 0U;
    }

    if (s_nanoRxTail == s_nanoRxHead)
    {
        return 0U;
    }

    *byte = s_nanoRxRing[s_nanoRxTail];
    s_nanoRxTail = RobotUart_NanoRingNext(s_nanoRxTail);
    return 1U;
}

void UART5_IRQHandler(void)
{
    uint16_t status = UART5->SR;

    RobotUart_NanoRecordErrors(status);
    if ((status & USART_FLAG_RXNE) != 0U)
    {
        RobotUart_NanoQueueByte((uint8_t)UART5->DR);
    }
    else if ((status & (USART_FLAG_PE | USART_FLAG_FE |
                        USART_FLAG_NE | USART_FLAG_ORE)) != 0U)
    {
        (void)UART5->DR;
    }
}

uint16_t RobotUart_NanoConsumeErrorFlags(void)
{
    uint16_t flags = s_nanoErrorFlags;
    s_nanoErrorFlags = 0U;
    return flags;
}
