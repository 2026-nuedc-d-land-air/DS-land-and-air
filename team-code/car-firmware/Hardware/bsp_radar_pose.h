/**
 ******************************************************************************
 * @file    bsp_radar_pose.h
 * @brief   Pi radar/SLAM pose bridge receiver on UART4.
 *
 * The currently deployed Pi bridge writes only fresh, accepted poses as:
 *   FA | sx xxx | sy yyy | syaw yyy | AB
 * where each s is 00=positive / 01=negative, X/Y are integer centimetres,
 * and yaw is integer 0.1 degree.  The 14-byte frame has no CRC; framing,
 * sign fields, yaw range, UART error flags and freshness are therefore gated
 * locally before the line-follow controller consumes it.
 ******************************************************************************
 */

#ifndef __BSP_RADAR_POSE_H
#define __BSP_RADAR_POSE_H

#include "stm32f10x.h"

typedef struct
{
    uint8_t valid;
    int32_t xCm;
    int32_t yCm;
    int16_t yawTenthsDeg;
    uint32_t lastFrameMs;
    uint32_t rawByteCount;
    uint32_t frameHeadCount;
    uint32_t validFrameCount;
    uint32_t invalidFrameCount;
    uint16_t uartErrorFlags;
} RadarPoseState_t;

void RadarPose_Init(uint32_t baudrate);
void RadarPose_Poll(uint32_t nowMs);
const RadarPoseState_t *RadarPose_GetState(void);
uint8_t RadarPose_IsFresh(uint32_t nowMs, uint32_t maxAgeMs);

#endif /* __BSP_RADAR_POSE_H */
