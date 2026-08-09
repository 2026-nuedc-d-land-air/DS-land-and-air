#ifndef MISSION_COMMAND_H
#define MISSION_COMMAND_H

#include <stdint.h>

#define MISSION_CH6_ENABLE_MIN 1700u
#define MISSION_CH6_ENABLE_MAX 2000u

typedef struct
{
    uint8_t ch6_initialized;
    uint8_t ch6_start_enabled;
    uint8_t stop_pending;
} mission_command_state_t;

typedef struct
{
    uint8_t start;
    uint8_t stop;
} mission_command_output_t;

void mission_command_init(mission_command_state_t *state);
uint8_t mission_command_ch6_allows_start(uint8_t rc_valid, uint16_t ch6_value);
void mission_command_update(mission_command_state_t *state,
                            uint8_t rc_valid,
                            uint16_t ch6_value,
                            uint8_t uart_start_request,
                            uint8_t uart_stop_request,
                            mission_command_output_t *output);
void mission_command_stop_accepted(mission_command_state_t *state);

#endif
