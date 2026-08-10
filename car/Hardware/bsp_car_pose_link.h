/**
 ******************************************************************************
 * @file    bsp_car_pose_link.h
 * @brief   V2.3 Pi-to-MCU CAR_POSE ingress on UART4.
 *
 * Preferred input is a CRC-valid V2.3 CAR_POSE at 10 Hz with src=0x31 (car
 * Pi), dst=0x32 (car MCU).  During migration it also accepts the physically
 * deployed 14-byte FA...AB Pi bridge frame as an *uncalibrated* local pose;
 * that compatibility path can support ground-station display but never marks
 * a sample CALIBRATED or substitutes for the Pi V2.2 task source timestamp.
 ******************************************************************************
 */

#ifndef __BSP_CAR_POSE_LINK_H
#define __BSP_CAR_POSE_LINK_H

#include "stm32f10x.h"

typedef struct
{
    uint8_t valid;
    uint8_t sourceFormat;
    uint8_t sequence;
    uint8_t lastAckRequestType;
    uint8_t lastAckRequestSeq;
    uint8_t lastAckResult;
    uint8_t lastAckDetail;
    uint8_t coordinateFrame;
    uint8_t poseFlags;
    uint8_t calibratedConsecutiveFrameCount;
    uint16_t calibrationId;
    int32_t xCm;
    int32_t yCm;
    int16_t yawTenthsDeg;
    int16_t vxCmPerSec;
    int16_t vyCmPerSec;
    uint32_t sourceTimeMs;
    uint32_t lastFrameMs;
    uint32_t rawByteCount;
    uint32_t validFrameCount;
    uint32_t v22FrameCount;
    uint32_t legacyFrameCount;
    uint32_t ackFrameCount;
    uint32_t ackInvalidFrameCount;
    uint32_t invalidFrameCount;
    uint32_t legacyInvalidFrameCount;
    uint32_t versionErrorCount;
    uint32_t lengthErrorCount;
    uint32_t crcErrorCount;
    uint32_t timeoutCount;
    uint32_t outOfOrderFrameCount;
    uint32_t sourceTimeRollbackCount;
    uint32_t invalidVelocityCount;
    uint16_t uartErrorFlags;
    uint32_t uartRingOverflowCount;
    /* Latest complete legacy candidate, retained only for diagnostic capture.
     * It is not a CRC-protected pose and must never be used as a second data
     * source by motion control. */
    uint8_t lastLegacyFrame[14];
    uint8_t lastLegacyFrameAvailable;
} CarPoseLinkState_t;

typedef struct
{
    uint8_t requestType;
    uint8_t requestSeq;
    uint8_t result;
    uint8_t detail;
} CarPoseLinkAck_t;

void CarPoseLink_Init(uint32_t baudrate);
void CarPoseLink_Poll(uint32_t nowMs);
const CarPoseLinkState_t *CarPoseLink_GetState(void);
uint8_t CarPoseLink_IsFresh(uint32_t nowMs, uint32_t maxAgeMs);
/* Returns the next validated Pi ACK in receive order. */
uint8_t CarPoseLink_TakeAck(CarPoseLinkAck_t *ack);
/* A confirmed Pi maintenance reset invalidates cached calibration immediately,
 * before the next uncalibrated CAR_POSE is due on the 10 Hz stream. */
void CarPoseLink_InvalidateCalibration(void);
/* A flight task may use only native V2.3, calibrated platform pose data.
 * Legacy FA...AB data remains display-only even while it is fresh. */
uint8_t CarPoseLink_IsTaskReady(uint32_t nowMs, uint32_t maxAgeMs);

#endif /* __BSP_CAR_POSE_LINK_H */
