#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "mission_command.h"

typedef struct
{
    mission_command_state_t command_state;
    uint8_t running;
    uint8_t land_busy_cycles;
    unsigned int start_count;
    unsigned int stop_call_count;
    unsigned int land_accept_count;
} virtual_fc_t;

static void fail(const char *scenario, const char *message)
{
    fprintf(stderr, "[FAIL] %s: %s\n", scenario, message);
    exit(1);
}

static void expect(const char *scenario, uint8_t condition, const char *message)
{
    if (condition == 0)
    {
        fail(scenario, message);
    }
}

static void virtual_fc_init(virtual_fc_t *fc)
{
    mission_command_init(&fc->command_state);
    fc->running = 0;
    fc->land_busy_cycles = 0;
    fc->start_count = 0;
    fc->stop_call_count = 0;
    fc->land_accept_count = 0;
}

/* 模拟 UserTask_OneKeyCmd() 的一次 20 ms 调度和 OneKey_Land() 驱动响应。 */
static void virtual_fc_tick(virtual_fc_t *fc,
                            uint8_t rc_valid,
                            uint16_t ch6_value,
                            uint8_t uart_start,
                            uint8_t uart_stop)
{
    mission_command_output_t output;

    mission_command_update(&fc->command_state,
                           rc_valid,
                           ch6_value,
                           uart_start,
                           uart_stop,
                           &output);

    if (output.stop != 0)
    {
        fc->stop_call_count++;
        fc->running = 0;

        if (fc->land_busy_cycles != 0)
        {
            fc->land_busy_cycles--;
        }
        else
        {
            fc->land_accept_count++;
            mission_command_stop_accepted(&fc->command_state);
        }
        return;
    }

    if (output.start != 0)
    {
        fc->start_count++;
        fc->running = 1;
    }
}

static void test_required_start_sequence(void)
{
    const char *name = "required_start_sequence";
    virtual_fc_t fc;

    virtual_fc_init(&fc);
    virtual_fc_tick(&fc, 1, 1500, 0, 0); /* CH6 中位：保险关闭 */
    virtual_fc_tick(&fc, 1, 1500, 1, 0); /* 中位点击地面站 START */
    expect(name, fc.running == 0 && fc.start_count == 0, "START bypassed CH6 interlock");

    virtual_fc_tick(&fc, 1, 1900, 0, 0); /* CH6 最低位置只打开保险 */
    expect(name, fc.running == 0, "CH6 enabled mission without UART START");

    virtual_fc_tick(&fc, 1, 1900, 1, 0); /* 保险打开后点击 START */
    expect(name, fc.running == 1 && fc.start_count == 1, "enabled UART START did not run mission");
}

static void test_middle_stops_and_inhibits(void)
{
    const char *name = "middle_stops_and_inhibits";
    virtual_fc_t fc;

    virtual_fc_init(&fc);
    virtual_fc_tick(&fc, 1, 1900, 0, 0);
    virtual_fc_tick(&fc, 1, 1900, 1, 0);
    virtual_fc_tick(&fc, 1, 1500, 0, 0); /* 离开允许区立即停止 */
    expect(name, fc.running == 0 && fc.land_accept_count == 1, "CH6 middle did not stop mission");

    virtual_fc_tick(&fc, 1, 1500, 1, 0);
    virtual_fc_tick(&fc, 1, 1500, 1, 0);
    expect(name, fc.running == 0 && fc.start_count == 1, "stable middle accepted UART START");
}

static void test_reenable_still_requires_uart_start(void)
{
    const char *name = "reenable_requires_uart_start";
    virtual_fc_t fc;

    virtual_fc_init(&fc);
    virtual_fc_tick(&fc, 1, 1900, 0, 0);
    virtual_fc_tick(&fc, 1, 1900, 1, 0);
    virtual_fc_tick(&fc, 1, 1500, 0, 0);
    virtual_fc_tick(&fc, 1, 1900, 0, 0);
    expect(name, fc.running == 0 && fc.start_count == 1, "CH6 re-enable auto-started mission");

    virtual_fc_tick(&fc, 1, 1900, 1, 0);
    expect(name, fc.running == 1 && fc.start_count == 2, "new UART START did not run after re-enable");
}

static void test_uart_stop_then_explicit_restart(void)
{
    const char *name = "uart_stop_then_restart";
    virtual_fc_t fc;

    virtual_fc_init(&fc);
    virtual_fc_tick(&fc, 1, 1900, 0, 0);
    virtual_fc_tick(&fc, 1, 1900, 1, 0);
    virtual_fc_tick(&fc, 1, 1900, 0, 1);
    expect(name, fc.running == 0 && fc.land_accept_count == 1, "UART STOP did not stop mission");

    virtual_fc_tick(&fc, 1, 1900, 0, 0);
    expect(name, fc.running == 0, "mission restarted without a new UART START");

    virtual_fc_tick(&fc, 1, 1900, 1, 0);
    expect(name, fc.running == 1 && fc.start_count == 2, "explicit restart was rejected while interlock enabled");
}

static void test_stop_retry_and_priority(void)
{
    const char *name = "stop_retry_and_priority";
    virtual_fc_t fc;
    unsigned int i;

    virtual_fc_init(&fc);
    virtual_fc_tick(&fc, 1, 1900, 0, 0);
    virtual_fc_tick(&fc, 1, 1900, 1, 0);
    fc.land_busy_cycles = 13; /* 约 260 ms，覆盖底层 wait_ck 的最坏释放时间 */

    virtual_fc_tick(&fc, 1, 1900, 0, 1);
    virtual_fc_tick(&fc, 1, 1900, 1, 0); /* STOP 未入队时 START 必须丢弃 */
    for (i = 0; i < 12; i++)
    {
        virtual_fc_tick(&fc, 1, 1900, 0, 0);
    }

    expect(name, fc.stop_call_count == 14, "STOP was not retried until accepted");
    expect(name, fc.land_accept_count == 1, "STOP was never accepted");
    expect(name, fc.running == 0 && fc.start_count == 1, "START bypassed a pending STOP");

    virtual_fc_tick(&fc, 1, 1900, 1, 0);
    expect(name, fc.running == 1 && fc.start_count == 2, "new START after accepted STOP did not run");
}

static void test_rc_loss_closes_interlock(void)
{
    const char *name = "rc_loss_closes_interlock";
    virtual_fc_t fc;

    virtual_fc_init(&fc);
    virtual_fc_tick(&fc, 1, 1900, 0, 0);
    virtual_fc_tick(&fc, 1, 1900, 1, 0);
    virtual_fc_tick(&fc, 0, 1900, 0, 0);
    expect(name, fc.running == 0 && fc.land_accept_count == 1, "RC loss did not stop mission");

    virtual_fc_tick(&fc, 0, 1900, 1, 0);
    expect(name, fc.running == 0 && fc.start_count == 1, "START ran while RC signal was invalid");

    virtual_fc_tick(&fc, 1, 1900, 0, 0);
    expect(name, fc.running == 0, "RC recovery auto-started mission");
    virtual_fc_tick(&fc, 1, 1900, 1, 0);
    expect(name, fc.running == 1 && fc.start_count == 2, "START failed after RC recovery");
}

int main(void)
{
    test_required_start_sequence();
    test_middle_stops_and_inhibits();
    test_reenable_still_requires_uart_start();
    test_uart_stop_then_explicit_restart();
    test_stop_retry_and_priority();
    test_rc_loss_closes_interlock();

    puts("[PASS] mission command interlock simulation: 6 scenarios");
    return 0;
}
