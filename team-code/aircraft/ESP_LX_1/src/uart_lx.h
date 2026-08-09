#ifndef UART_LX_H
#define UART_LX_H

#include "main.h"
#include <Arduino.h>
#include <stddef.h>

#define LX     Serial2
#define LX_TX  2
#define LX_RX  1

enum LxExtendedSubtype : uint8_t {
    LX_CAR_POSE = 0x01,
    LX_MISSION_REQUEST = 0x02,
    LX_MISSION_RESPONSE = 0x03,
    LX_MISSION_STATUS = 0x04,
    LX_MAINTENANCE_RESET = 0x05,
    // V2.3 requires wireless MISSION_ABORT but omits its private-LX adapter.
    // This zero-payload event closes that ESP32 -> flight-controller gap.
    LX_MISSION_ABORT = 0x06,
    LX_MISSION_TRACE = 0x07
};

typedef struct {
    bool hasPosition;
    bool taskRunning;
    uint8_t modeCode;
    int32_t xCm;
    int32_t yCm;
    int32_t zCm;
    int16_t yawDeciDegrees;
    uint32_t sourceTimeMs;
} FlightStatusSnapshot;

void data_receive_LX(void);
void data_anl_LX(uint8_t *data, uint8_t data_len);
bool data_send_LX(uint8_t data_func, uint8_t data_message);
bool data_send_LX_extended(uint8_t subtype, const uint8_t *payload,
                           uint8_t payload_length);
bool data_send_LX_car_pose(const uint8_t *payload, size_t payload_length);
bool data_send_LX_mission_request(const uint8_t *payload,
                                  size_t payload_length);
bool data_send_LX_mission_abort(void);
bool data_send_LX_maintenance_reset(const uint8_t *payload,
                                    size_t payload_length);
void get_flight_status_snapshot(FlightStatusSnapshot *snapshot);
bool flight_position_is_fresh(uint32_t now_ms, uint32_t max_age_ms = 500);

#endif
