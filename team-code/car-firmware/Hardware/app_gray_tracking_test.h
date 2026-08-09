#ifndef __APP_GRAY_TRACKING_TEST_H
#define __APP_GRAY_TRACKING_TEST_H

#include "stm32f10x.h"

void GrayTrackingTest_Init(uint32_t nowMs);
void GrayTrackingTest_Update(uint32_t nowMs);
void GrayTrackingTest_HandleCommand(char command, uint32_t nowMs);

#endif /* __APP_GRAY_TRACKING_TEST_H */
