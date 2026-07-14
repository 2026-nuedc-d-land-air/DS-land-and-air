#include "bsp_robot_uart.h"

static uint16_t s_nanoErrorFlags;

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
    s_nanoErrorFlags = 0U;
}

void RobotUart_NanoWriteByte(uint8_t byte)
{
    RobotUart_WriteByte(UART5, byte);
}

void RobotUart_NanoWriteString(const char *text)
{
    RobotUart_WriteString(UART5, text);
}

uint8_t RobotUart_NanoTryReadByte(uint8_t *byte)
{
    uint16_t status;
    uint16_t errors;

    if (byte == 0)
    {
        return 0U;
    }

    status = UART5->SR;
    errors = (uint16_t)(status & (USART_FLAG_PE |
                                  USART_FLAG_FE |
                                  USART_FLAG_NE |
                                  USART_FLAG_ORE));
    if ((errors & USART_FLAG_PE) != 0U)
    {
        s_nanoErrorFlags |= ROBOT_UART_ERROR_PARITY;
    }
    if ((errors & USART_FLAG_FE) != 0U)
    {
        s_nanoErrorFlags |= ROBOT_UART_ERROR_FRAMING;
    }
    if ((errors & USART_FLAG_NE) != 0U)
    {
        s_nanoErrorFlags |= ROBOT_UART_ERROR_NOISE;
    }
    if ((errors & USART_FLAG_ORE) != 0U)
    {
        s_nanoErrorFlags |= ROBOT_UART_ERROR_OVERRUN;
    }

    if ((status & USART_FLAG_RXNE) == 0U)
    {
        if (errors != 0U)
        {
            (void)UART5->DR;
        }
        return 0U;
    }

    *byte = (uint8_t)UART5->DR;
    return 1U;
}

uint16_t RobotUart_NanoConsumeErrorFlags(void)
{
    uint16_t flags = s_nanoErrorFlags;
    s_nanoErrorFlags = 0U;
    return flags;
}
