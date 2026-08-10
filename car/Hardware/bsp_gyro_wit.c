#include "bsp_gyro_wit.h"
#include "misc.h"

#define GYRO_WIT_FRAME_SIZE  11U
#define GYRO_WIT_FRAME_HEAD  0x55U
#define GYRO_WIT_FRAME_RATE  0x52U
#define GYRO_WIT_FRAME_ANGLE 0x53U

static volatile GyroWitState_t s_state;
static volatile uint8_t s_frame[GYRO_WIT_FRAME_SIZE];
static volatile uint8_t s_frameLength = 0U;

static int16_t GyroWit_ReadLe16(const volatile uint8_t *data)
{
    return (int16_t)(((uint16_t)data[1] << 8) | data[0]);
}

static int16_t GyroWit_RawAngleToTenthsDeg(int16_t raw)
{
    return (int16_t)(((int32_t)raw * 1800) / 32768);
}

static int16_t GyroWit_RawRateToTenthsDegPerSec(int16_t raw)
{
    /* WIT normal-protocol angular velocity full scale is +/-2000 deg/s. */
    return (int16_t)(((int32_t)raw * 20000) / 32768);
}

static uint8_t GyroWit_ConsumeByte(uint8_t byte)
{
    uint8_t index;
    uint8_t checksum = 0U;

    if ((s_frameLength == 0U) && (byte != GYRO_WIT_FRAME_HEAD))
    {
        ++s_state.discardedByteCount;
        return 0U;
    }

    if ((s_frameLength == 0U) && (byte == GYRO_WIT_FRAME_HEAD))
    {
        ++s_state.frameHeadCount;
    }
    else if ((s_frameLength == 1U) && (byte == GYRO_WIT_FRAME_HEAD))
    {
        /* Match the proven MSPM0 parser: a repeated header restarts sync. */
        s_frame[0] = byte;
        ++s_state.frameHeadCount;
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

    if (s_frame[1] == GYRO_WIT_FRAME_RATE)
    {
        s_state.rollRateTenthsDegPerSec = GyroWit_RawRateToTenthsDegPerSec(
            GyroWit_ReadLe16(&s_frame[2]));
        s_state.pitchRateTenthsDegPerSec = GyroWit_RawRateToTenthsDegPerSec(
            GyroWit_ReadLe16(&s_frame[4]));
        s_state.yawRateTenthsDegPerSec = GyroWit_RawRateToTenthsDegPerSec(
            GyroWit_ReadLe16(&s_frame[6]));
        ++s_state.angularVelocityFrameCount;
        return 1U;
    }

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
    NVIC_InitTypeDef nvic;

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

    s_state.rollTenthsDeg = 0;
    s_state.pitchTenthsDeg = 0;
    s_state.yawTenthsDeg = 0;
    s_state.rollRateTenthsDegPerSec = 0;
    s_state.pitchRateTenthsDegPerSec = 0;
    s_state.yawRateTenthsDegPerSec = 0;
    s_state.validFrameCount = 0U;
    s_state.angleFrameCount = 0U;
    s_state.angularVelocityFrameCount = 0U;
    s_state.rawByteCount = 0U;
    s_state.frameHeadCount = 0U;
    s_state.discardedByteCount = 0U;
    s_state.checksumErrorCount = 0U;
    s_frameLength = 0U;

    /*
     * JY901 frames at 9600 baud must be consumed per byte. The line mission
     * runs its control observer at 20 ms, so polling only there loses most of
     * the stream. This RXNE path mirrors the known-good MSPM0 UART ISR.
     */
    (void)USART2->SR;
    (void)USART2->DR;
    nvic.NVIC_IRQChannel = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1U;
    nvic.NVIC_IRQChannelSubPriority = 1U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);
}

uint8_t GyroWit_ReceiveByte(uint8_t byte)
{
    ++s_state.rawByteCount;
    return GyroWit_ConsumeByte(byte);
}

uint8_t GyroWit_Poll(void)
{
    uint8_t updated = 0U;

    while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET)
    {
        if (GyroWit_ReceiveByte((uint8_t)USART_ReceiveData(USART2)) != 0U)
        {
            updated = 1U;
        }
    }

    return updated;
}

const volatile GyroWitState_t *GyroWit_GetState(void)
{
    return &s_state;
}

void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        (void)GyroWit_ReceiveByte((uint8_t)USART_ReceiveData(USART2));
    }
}
