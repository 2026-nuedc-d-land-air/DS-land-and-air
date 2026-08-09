#ifndef UART_LORA_H
#define UART_LORA_H

#include <Arduino.h>
#include <stddef.h>

#include "protocol_v2.h"

#define Lora Serial1
#define Lora_TX 17
#define Lora_RX 18

void data_receive_Lora(void);
void lora_protocol_service(void);
void lora_on_mission_response(
    const gpio_lora_v2::MissionResponsePayload &response);
bool lora_queue_mission_status(const uint8_t *payload, size_t payload_length);

#endif
