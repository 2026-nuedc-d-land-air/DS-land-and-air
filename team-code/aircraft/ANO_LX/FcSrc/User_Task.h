#ifndef __USER_TASK_H
#define __USER_TASK_H

#ifdef UNIT_TEST
#include <stdint.h>
#else
#include "SysConfig.h"
#include "stm32f4xx.h"
#endif

typedef enum
{
    MISSION_STAGE_IDLE = 0,
    MISSION_STAGE_PRECHECK = 1,
    MISSION_STAGE_TAKEOFF = 2,
    MISSION_STAGE_INTERCEPT = 3,
    MISSION_STAGE_FOLLOW = 4,
    MISSION_STAGE_DROP_ALIGN = 5,
    MISSION_STAGE_DROP_ACTION = 6,
    MISSION_STAGE_LAND_ALIGN = 7,
    MISSION_STAGE_DESCEND = 8,
    MISSION_STAGE_ON_PLATFORM_5S = 9,
    MISSION_STAGE_PLATFORM_TAKEOFF = 10,
    MISSION_STAGE_RETURN_HOME = 11,
    MISSION_STAGE_HOME_LAND = 12,
    MISSION_STAGE_ABORT = 13
} mission_stage_t;

void UserTask_OneKeyCmd(void);

#endif
