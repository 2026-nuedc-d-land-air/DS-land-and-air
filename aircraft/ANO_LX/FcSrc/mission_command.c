#include "mission_command.h"

uint8_t mission_command_ch6_allows_start(uint8_t rc_valid, uint16_t ch6_value)
{
    return (rc_valid != 0 &&
            ch6_value >= MISSION_CH6_ENABLE_MIN &&
            ch6_value <= MISSION_CH6_ENABLE_MAX)
               ? 1u
               : 0u;
}

void mission_command_init(mission_command_state_t *state)
{
    state->ch6_initialized = 0;
    state->ch6_start_enabled = 0;
    state->stop_pending = 0;
}

void mission_command_update(mission_command_state_t *state,
                            uint8_t rc_valid,
                            uint16_t ch6_value,
                            uint8_t uart_start_request,
                            uint8_t uart_stop_request,
                            mission_command_output_t *output)
{
    uint8_t ch6_start_enabled;

    output->start = 0;
    output->stop = 0;

    /*
     * CH6 是硬件启动保险，不是启动命令：只有 1700~2000 允许 UART START。
     * 离开允许区或遥控失联时锁存 STOP；回到允许区不会自动启动。
     * 首个样本只建立保险状态，避免上电时无任务却发送多余降落命令。
     */
    ch6_start_enabled = mission_command_ch6_allows_start(rc_valid, ch6_value);
    if (state->ch6_initialized == 0)
    {
        state->ch6_initialized = 1;
        state->ch6_start_enabled = ch6_start_enabled;
    }
    else if (state->ch6_start_enabled != 0 && ch6_start_enabled == 0)
    {
        state->ch6_start_enabled = 0;
        state->stop_pending = 1;
    }
    else
    {
        state->ch6_start_enabled = ch6_start_enabled;
    }

    /* UART STOP 始终优先，并保持到 OneKey_Land() 成功进入命令队列。 */
    if (uart_stop_request != 0)
    {
        state->stop_pending = 1;
    }

    if (state->stop_pending != 0)
    {
        output->stop = 1;
        return;
    }

    /* START 只看当前保险状态；在中位/失联时直接丢弃，不锁存。 */
    if (uart_start_request != 0 && state->ch6_start_enabled != 0)
    {
        output->start = 1;
    }
}

void mission_command_stop_accepted(mission_command_state_t *state)
{
    state->stop_pending = 0;
}
