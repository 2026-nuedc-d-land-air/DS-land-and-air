#ifndef __APP_LORA_PROTOCOL_TEST_H
#define __APP_LORA_PROTOCOL_TEST_H

#include "stm32f10x.h"

/*
 * UART5 LoRa V2.2 bench diagnostic.
 * This test parses/logs frames only: it never changes motion, calibration,
 * task, or acknowledgement state.
 */
void LoraProtocolTest_Init(uint32_t nowMs);
void LoraProtocolTest_HandleRadioByte(uint8_t byte, uint32_t nowMs);
void LoraProtocolTest_HandleUartErrors(uint16_t flags, uint32_t nowMs);
void LoraProtocolTest_HandleCommand(char command, uint32_t nowMs);
void LoraProtocolTest_Update(uint32_t nowMs);

#endif /* __APP_LORA_PROTOCOL_TEST_H */
