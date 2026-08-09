#ifndef __APP_CONTROL_H_
#define __APP_CONTROL_H_

#include "stm32f10x.h"

typedef enum {
    CONTROL_STATE_IDLE = 0,
    CONTROL_STATE_LINE_FOLLOW,
    CONTROL_STATE_FINISH,
    CONTROL_STATE_LOST_LINE,
    CONTROL_STATE_STUCK,
    CONTROL_STATE_OBSTACLE,
    CONTROL_STATE_LIFTED,
    CONTROL_STATE_STOP,
    CONTROL_STATE_CROSS,
    CONTROL_STATE_GAP,
    CONTROL_STATE_TURN
} ControlState_t;

typedef struct {
    int16_t vx;                /* forward command 0-1000 */
    int16_t vz;                /* turn command -1000..1000 */
    int8_t trackError;         /* line error */
    uint8_t trackCount;        /* active sensor count */
    uint8_t trackLost;         /* 1 if lost line */
    uint8_t isStraight;        /* 1 if straight window */
    uint8_t isSlope;           /* 1 if slope detected */
    uint8_t isRepeat;          /* 1 if path repeat detected */
    uint8_t isLifted;          /* 1 if lifted off ground */
    ControlState_t state;
} ControlOutput_t;

void Control_Init(void);
void Control_Reset(void);
void Control_SetObstacleMode(uint8_t enable);
uint8_t Control_GetObstacleMode(void);
uint8_t Control_IsObstacleActive(void);
ControlOutput_t Control_Update(uint8_t trackData, float gyroZ, float anglePitch, float distanceCm, float speedLeftMms, float speedRightMms, uint8_t gyroValid, uint32_t nowMs);

#endif
