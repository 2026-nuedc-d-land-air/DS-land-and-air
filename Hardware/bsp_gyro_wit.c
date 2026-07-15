#include "bsp_gyro_wit.h"

#define GYRO_WIT_FRAME_SIZE  11U
#define GYRO_WIT_FRAME_HEAD  0x55U
#define GYRO_WIT_FRAME_ANGLE 0x53U

static GyroWitState_t s_state;
static uint8_t s_frame[GYRO_WIT_FRAME_SIZE];
static uint8_t s_frameLength = 0U;

static int16_t GyroWit_ReadLe16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[1] << 8) | data[0]);
}

static int16_t GyroWit_RawAngleToTenthsDeg(int16_t raw)
{
    return (int16_t)(((int32_t)raw * 1800) / 32768);
}

static uint8_t GyroWit_ConsumeByte(uint8_t byte)
{
    uint8_t index;
    uint8_t checksum = 0U;

    if ((s_frameLength == 0U) && (byte != GYRO_WIT_FRAME_HEAD))
    {
        return 0U;
    }

    s_frame[s_frameLength++] = byte;
    if (s_frameLength < GYRO_WIT_FRAME_SIZE)
    {
        return 0U;
    }

    for (index = 0U; index < (GYRO_WIT_FRAME_SIZE - 1U); ++index)
    {
        checksum = (uint8_t)(checksum + s_frame[index]);
    }

    if (checksum != s_frame[GYRO_WIT_FRAME_SIZE - 1U])
    {
        ++s_state.checksumErrorCount;
        s_frameLength = 0U;
        return 0U;
    }

    ++s_state.validFrameCount;
    s_frameLength = 0U;

    if (s_frame[1] != GYRO_WIT_FRAME_ANGLE)
    {
        return 0U;
    }

    s_state.rollTenthsDeg = GyroWit_RawAngleToTenthsDeg(
        GyroWit_ReadLe16(&s_frame[2]));
    s_state.pitchTenthsDeg = GyroWit_RawAngleToTenthsDeg(
        GyroWit_ReadLe16(&s_frame[4]));
    s_state.yawTenthsDeg = GyroWit_RawAngleToTenthsDeg(
        GyroWit_ReadLe16(&s_frame[6]));
    ++s_state.angleFrameCount;
    return 1U;
}

void GyroWit_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD |
                           RCC_APB2Periph_AFIO, ENABLE);

    /* USART2 remap: PD5=TX and PD6=RX. */
    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_5;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &gpio);

    USART_StructInit(&usart);
    usart.USART_BaudRate = baudrate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &usart);
    USART_Cmd(USART2, ENABLE);

    s_state.rollTenthsDeg = 0;
    s_state.pitchTenthsDeg = 0;
    s_state.yawTenthsDeg = 0;
    s_state.validFrameCount = 0U;
    s_state.angleFrameCount = 0U;
    s_state.checksumErrorCount = 0U;
    s_frameLength = 0U;
}

uint8_t GyroWit_Poll(void)
{
    uint8_t updated = 0U;

    while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET)
    {
        if (GyroWit_ConsumeByte((uint8_t)USART_ReceiveData(USART2)) != 0U)
        {
            updated = 1U;
        }
    }

    return updated;
}

const GyroWitState_t *GyroWit_GetState(void)
{
    return &s_state;
}
