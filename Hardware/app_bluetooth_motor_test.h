#ifndef __APP_BLUETOOTH_MOTOR_TEST_H
#define __APP_BLUETOOTH_MOTOR_TEST_H

#include "stm32f10x.h"

#define BLUETOOTH_RX_HISTORY_CAPACITY 16U

extern volatile uint32_t g_bluetoothRxCount;
extern volatile uint32_t g_bluetoothRxLastMs;
extern volatile uint8_t g_bluetoothRxLastByte;
extern volatile uint8_t g_bluetoothRxHistory[BLUETOOTH_RX_HISTORY_CAPACITY];
extern volatile uint8_t g_bluetoothRxHistoryCount;
extern volatile uint8_t g_bluetoothRxHistoryWriteIndex;

void BluetoothMotorTest_Init(uint32_t nowMs);
void BluetoothMotorTest_HandleByte(char source, uint8_t byte, uint32_t nowMs);
void BluetoothMotorTest_Update(uint32_t nowMs);

#endif /* __APP_BLUETOOTH_MOTOR_TEST_H */
