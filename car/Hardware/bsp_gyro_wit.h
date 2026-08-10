#ifndef __BSP_GYRO_WIT_H
#define __BSP_GYRO_WIT_H

#include "stm32f10x.h"

/* WIT/JY serial normal-protocol telemetry from USART2 remap (PD5/PD6). */
typedef struct
{
    int16_t rollTenthsDeg;
    int16_t pitchTenthsDeg;
    int16_t yawTenthsDeg;
    int16_t rollRateTenthsDegPerSec;
    int16_t pitchRateTenthsDegPerSec;
    int16_t yawRateTenthsDegPerSec;
    uint32_t validFrameCount;
    uint32_t angleFrameCount;
    uint32_t angularVelocityFrameCount;
    uint32_t rawByteCount;
    uint32_t frameHeadCount;
    uint32_t discardedByteCount;
    uint32_t checksumErrorCount;
} GyroWitState_t;

void GyroWit_Init(uint32_t baudrate);
/* Called from USART2 RXNE ISR; valid for the remapped PD6 RX stream. */
uint8_t GyroWit_ReceiveByte(uint8_t byte);
uint8_t GyroWit_Poll(void);
const volatile GyroWitState_t *GyroWit_GetState(void);

#endif /* __BSP_GYRO_WIT_H */
