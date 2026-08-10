#ifndef __MOTOR_DIAG_LOG_H
#define __MOTOR_DIAG_LOG_H

#include "bsp_l298n.h"

typedef enum
{
    MOTOR_DIAG_EVENT_BOOT = 0,
    MOTOR_DIAG_EVENT_STOP,
    MOTOR_DIAG_EVENT_COMMAND,
    MOTOR_DIAG_EVENT_STAGE_BEGIN,
    MOTOR_DIAG_EVENT_STAGE_END
} MotorDiagEvent_t;

void MotorDiagLog_Init(void);
void MotorDiagLog_Record(uint32_t timeMs,
                         MotorDiagEvent_t event,
                         L298N_Wheel_t wheel,
                         int16_t command);
void MotorDiagLog_Freeze(void);
void MotorDiagLog_Export(void);
uint8_t MotorDiagLog_PollCommand(void);

#endif /* __MOTOR_DIAG_LOG_H */
