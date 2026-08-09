#ifndef __APP_LINE_FOLLOW_MISSION_H
#define __APP_LINE_FOLLOW_MISSION_H

#include "stm32f10x.h"

/* Task keys: PG13 starts task 1 and PG9 starts task 2. Each task holds the
 * car stopped for 20 s before release; a second press stops an active run.
 * PG10 is intentionally not a control key. */
void LineFollowMission_Init(uint32_t nowMs);
void LineFollowMission_Update(uint32_t nowMs);
void LineFollowMission_HandleCommand(char command, uint32_t nowMs);

#endif /* __APP_LINE_FOLLOW_MISSION_H */
