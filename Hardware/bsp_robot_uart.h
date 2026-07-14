#ifndef __BSP_ROBOT_UART_H
#define __BSP_ROBOT_UART_H

#include "stm32f10x.h"

/* UART4: PC10=TX, PC11=RX, ZL bus-servo controller. */
void RobotUart_ServoInit(uint32_t baudrate);
void RobotUart_ServoWriteByte(uint8_t byte);
void RobotUart_ServoWriteBuffer(const uint8_t *data, uint16_t length);
void RobotUart_ServoWriteString(const char *text);
uint8_t RobotUart_ServoTryReadByte(uint8_t *byte);

/* USART3 full remap: PD8=TX, PD9=RX, Bluetooth module. */
void RobotUart_BluetoothInit(uint32_t baudrate);
void RobotUart_BluetoothWriteByte(uint8_t byte);
void RobotUart_BluetoothWriteString(const char *text);
uint8_t RobotUart_BluetoothTryReadByte(uint8_t *byte);

/* UART5: PC12=TX, PD2=RX, Jetson Nano. */
#define ROBOT_UART_ERROR_PARITY  (1U << 0)
#define ROBOT_UART_ERROR_FRAMING (1U << 1)
#define ROBOT_UART_ERROR_NOISE   (1U << 2)
#define ROBOT_UART_ERROR_OVERRUN (1U << 3)

void RobotUart_NanoInit(uint32_t baudrate);
void RobotUart_NanoWriteByte(uint8_t byte);
void RobotUart_NanoWriteString(const char *text);
uint8_t RobotUart_NanoTryReadByte(uint8_t *byte);
uint16_t RobotUart_NanoConsumeErrorFlags(void);

#endif /* __BSP_ROBOT_UART_H */
