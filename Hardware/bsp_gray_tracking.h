#ifndef __BSP_GRAY_TRACKING_H
#define __BSP_GRAY_TRACKING_H

#include "stm32f10x.h"

typedef struct
{
    /* bit0..bit7 = channel 1..channel 8 digital values. */
    uint8_t rawMask;
} GrayTrackingSample_t;

void GrayTracking_Init(void);
void GrayTracking_Read(GrayTrackingSample_t *sample);

#endif /* __BSP_GRAY_TRACKING_H */
