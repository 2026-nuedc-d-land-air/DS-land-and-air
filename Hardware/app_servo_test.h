#ifndef __APP_SERVO_TEST_H
#define __APP_SERVO_TEST_H

#include "stm32f10x.h"

void ServoTest_Init(void);
void ServoTest_StopAll(void);
void ServoTest_HandleCommand(char source, uint8_t command);
void ServoTest_PollServoResponse(void);
void ServoTest_LogNanoByte(uint8_t byte);

#endif /* __APP_SERVO_TEST_H */
