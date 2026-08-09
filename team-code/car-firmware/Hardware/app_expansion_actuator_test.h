/**
 ******************************************************************************
 * @file    app_expansion_actuator_test.h
 * @brief   扩展 TB6612 与双 A4988 的手动触发测试应用。
 ******************************************************************************
 */

#ifndef __APP_EXPANSION_ACTUATOR_TEST_H
#define __APP_EXPANSION_ACTUATOR_TEST_H

#include "stm32f10x.h"

void ExpansionActuatorTest_Init(uint32_t nowMs);
void ExpansionActuatorTest_Update(uint32_t nowMs);
void ExpansionActuatorTest_HandleCommand(char command, uint32_t nowMs);

#endif /* __APP_EXPANSION_ACTUATOR_TEST_H */
