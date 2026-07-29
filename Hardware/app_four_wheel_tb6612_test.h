#ifndef __APP_FOUR_WHEEL_TB6612_TEST_H
#define __APP_FOUR_WHEEL_TB6612_TEST_H

#include "stm32f10x.h"

void FourWheelTb6612Test_Init(uint32_t nowMs);
void FourWheelTb6612Test_Update(uint32_t nowMs);
void FourWheelTb6612Test_HandleCommand(char command, uint32_t nowMs);

#endif /* __APP_FOUR_WHEEL_TB6612_TEST_H */
