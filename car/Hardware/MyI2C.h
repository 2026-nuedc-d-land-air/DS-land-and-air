#ifndef __MYI2C_H
#define __MYI2C_H

void MyI2C_Init(void);
void MyI2C_Start(void);
void MyI2C_Stop(void);
void MyI2C_SendByte(uint8_t Byte);
uint8_t MyI2C_ReceiveByte(void);
void MyI2C_SendAck(uint8_t AckBit);
uint8_t MyI2C_ReceiveAck(void);

void MyI2C_ReleaseBus(void);  // 释放I2C总线（切换前调用）
void MyI2C_ReInit(void);      // 重新初始化MyI2C（切换回来时调用）

#endif
