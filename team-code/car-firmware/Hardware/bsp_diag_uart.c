#include "bsp_diag_uart.h"

void DiagUart_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;

    /* Default USART1 on PA9/PA10 for J-Link CDC / onboard VCOM.
     * Do not enable USART1 remap (PB6/PB7) here. */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_AFIO |
                           RCC_APB2Periph_USART1, ENABLE);

    GPIO_PinRemapConfig(GPIO_Remap_USART1, DISABLE);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_9; /* PA9 = USART1_TX */
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_10; /* PA10 = USART1_RX */
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    USART_StructInit(&usart);
    usart.USART_BaudRate = baudrate;
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &usart);
    USART_Cmd(USART1, ENABLE);
}

void DiagUart_WriteChar(char ch)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
    USART_SendData(USART1, (uint16_t)(uint8_t)ch);
}

void DiagUart_WriteString(const char *text)
{
    if (text == 0)
    {
        return;
    }

    while (*text != '\0')
    {
        DiagUart_WriteChar(*text++);
    }
}

void DiagUart_WriteUInt32(uint32_t value)
{
    char buffer[10];
    uint8_t length = 0;

    do
    {
        buffer[length++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);

    while (length != 0U)
    {
        DiagUart_WriteChar(buffer[--length]);
    }
}

void DiagUart_WriteInt32(int32_t value)
{
    uint32_t magnitude;

    if (value < 0)
    {
        DiagUart_WriteChar('-');
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    }
    else
    {
        magnitude = (uint32_t)value;
    }

    DiagUart_WriteUInt32(magnitude);
}

uint8_t DiagUart_TryReadChar(char *ch)
{
    if ((ch == 0) ||
        (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET))
    {
        return 0U;
    }

    *ch = (char)(uint8_t)USART_ReceiveData(USART1);
    return 1U;
}
