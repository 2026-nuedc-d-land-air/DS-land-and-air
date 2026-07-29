/**
 ******************************************************************************
 * @file    app_four_wheel_tb6612_test.c
 * @brief   Four-wheel PWM bring-up with a bounded, simultaneous drive test.
 *
 * Auto test after 1 s: all four wheels run for 4 s, then coast and disable.
 * G repeats the forward test, B runs the reverse test, and S stops immediately.
 ******************************************************************************
 */

#include "app_four_wheel_tb6612_test.h"

#include "bsp_aux_tb6612.h"
#include "bsp_diag_uart.h"
#include "bsp_motor.h"

#define DRIVE_MS           4000U
#define DRIVE_PERCENT      65
#define LOG_PERIOD_MS       250U
#define IDLE_LOG_MS        1000U
#define AUTO_START_MS      1000U

typedef enum
{
    ST_IDLE = 0,
    ST_ALL_DRIVE,
    ST_ALL_STOP
} Stage_t;

static Stage_t s_stage;
static uint32_t s_stageStartMs;
static uint32_t s_lastLogMs;
static uint32_t s_bootMs;
static uint8_t s_running;
static uint8_t s_autoStarted;
static int16_t s_driveSign;

static uint8_t TimeReached(uint32_t nowMs, uint32_t deadlineMs)
{
    return ((int32_t)(nowMs - deadlineMs) >= 0) ? 1U : 0U;
}

static const char *StageName(Stage_t stage)
{
    switch (stage)
    {
        case ST_ALL_DRIVE: return (s_driveSign >= 0) ? "ALL_FORWARD_PWM" : "ALL_REVERSE_PWM";
        case ST_ALL_STOP: return "ALL_STOP";
        default: return "IDLE";
    }
}

static void WriteEvent(const char *event, uint32_t nowMs)
{
    DiagUart_WriteString("FW,event,");
    DiagUart_WriteString(event);
    DiagUart_WriteString(",t_ms,");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",stage,");
    DiagUart_WriteString(StageName(s_stage));
    DiagUart_WriteString(",front_stby_sw,");
    DiagUart_WriteUInt32(Motor_IsEnabled());
    DiagUart_WriteString(",pe6,");
    DiagUart_WriteUInt32(Motor_GetStbyLevel());
    DiagUart_WriteString(",pa2,");
    DiagUart_WriteUInt32(Motor_GetPwmGpioLevel(MOTOR_LEFT));
    DiagUart_WriteString(",pa3,");
    DiagUart_WriteUInt32(Motor_GetPwmGpioLevel(MOTOR_RIGHT));
    DiagUart_WriteString(",front_l_dir,");
    DiagUart_WriteUInt32(Motor_GetDirBits(MOTOR_LEFT));
    DiagUart_WriteString(",front_r_dir,");
    DiagUart_WriteUInt32(Motor_GetDirBits(MOTOR_RIGHT));
    DiagUart_WriteString(",rear_stby_sw,");
    DiagUart_WriteUInt32(AuxTb6612_IsEnabled());
    DiagUart_WriteString(",rear_pwm_a,");
    DiagUart_WriteUInt32(AuxTb6612_GetPwmCompare(AUX_TB6612_MOTOR_A));
    DiagUart_WriteString(",rear_pwm_b,");
    DiagUart_WriteUInt32(AuxTb6612_GetPwmCompare(AUX_TB6612_MOTOR_B));
    DiagUart_WriteString(",ain,");
    DiagUart_WriteUInt32(AuxTb6612_GetDirBits(AUX_TB6612_MOTOR_A));
    DiagUart_WriteString(",bin,");
    DiagUart_WriteUInt32(AuxTb6612_GetDirBits(AUX_TB6612_MOTOR_B));
    DiagUart_WriteString(",rear_enc_a,");
    DiagUart_WriteUInt32(AuxTb6612_GetEncoderTransitions(AUX_TB6612_MOTOR_A));
    DiagUart_WriteString(",rear_enc_b,");
    DiagUart_WriteUInt32(AuxTb6612_GetEncoderTransitions(AUX_TB6612_MOTOR_B));
    DiagUart_WriteString("\r\n");
}

static void StopAll(void)
{
    AuxTb6612_StopAll();
    Motor_Coast(MOTOR_LEFT);
    Motor_Coast(MOTOR_RIGHT);
    Motor_Enable(0U);
    Motor_RestorePwmAf();
    s_running = 0U;
    s_stage = ST_IDLE;
}

static void EnterStage(Stage_t stage, uint32_t nowMs)
{
    s_stage = stage;
    s_stageStartMs = nowMs;

    switch (stage)
    {
        case ST_ALL_DRIVE:
            Motor_RestorePwmAf();
            AuxTb6612_RestorePwmAf();
            Motor_SetSpeedBoth((int16_t)(s_driveSign * DRIVE_PERCENT),
                                (int16_t)(s_driveSign * DRIVE_PERCENT));
            AuxTb6612_SetRawSpeed(AUX_TB6612_MOTOR_A,
                                   (int16_t)(s_driveSign * DRIVE_PERCENT));
            AuxTb6612_SetRawSpeed(AUX_TB6612_MOTOR_B,
                                   (int16_t)(s_driveSign * DRIVE_PERCENT));
            Motor_Enable(1U);
            AuxTb6612_Enable(1U);
            break;
        case ST_ALL_STOP:
        default:
            AuxTb6612_StopAll();
            Motor_Coast(MOTOR_LEFT);
            Motor_Coast(MOTOR_RIGHT);
            Motor_Enable(0U);
            Motor_RestorePwmAf();
            s_running = 0U;
            break;
    }

    WriteEvent("stage_begin", nowMs);
}

static void StartDrive(int16_t sign, uint32_t nowMs)
{
    StopAll();
    s_driveSign = (sign >= 0) ? 1 : -1;
    s_running = 1U;
    EnterStage(ST_ALL_DRIVE, nowMs);
}

void FourWheelTb6612Test_Init(uint32_t nowMs)
{
    StopAll();
    s_stageStartMs = nowMs;
    s_lastLogMs = nowMs;
    s_bootMs = nowMs;
    s_autoStarted = 0U;

    s_driveSign = 1;
    DiagUart_WriteString("FW,boot,fw=all_wheel_pwm,safe_idle=1\r\n");
    DiagUart_WriteString("FW,config,front=pa2_pa3_pe2_pe6,rear=pe13_pe14_pf1_pf4_pc2\r\n");
    DiagUart_WriteString("FW,note,auto=all_forward_65pct_4s;G=forward,B=reverse,S=stop\r\n");
}

void FourWheelTb6612Test_Update(uint32_t nowMs)
{
    uint32_t elapsed = nowMs - s_stageStartMs;

    AuxTb6612_EncoderPoll();

    if ((s_autoStarted == 0U) &&
        (s_running == 0U) &&
        (TimeReached(nowMs, s_bootMs + AUTO_START_MS) != 0U))
    {
        s_autoStarted = 1U;
        DiagUart_WriteString("FW,event,auto_start\r\n");
        StartDrive(+1, nowMs);
        elapsed = 0U;
    }

    if (((s_running != 0U) &&
         (TimeReached(nowMs, s_lastLogMs + LOG_PERIOD_MS) != 0U)) ||
        ((s_running == 0U) &&
         (TimeReached(nowMs, s_lastLogMs + IDLE_LOG_MS) != 0U)))
    {
        s_lastLogMs = nowMs;
        WriteEvent((s_running != 0U) ? "sample" : "idle", nowMs);
    }

    switch (s_stage)
    {
        case ST_ALL_DRIVE:
            if (elapsed >= DRIVE_MS) EnterStage(ST_ALL_STOP, nowMs);
            break;
        default:
            break;
    }
}

void FourWheelTb6612Test_HandleCommand(char command, uint32_t nowMs)
{
    switch (command)
    {
        case 'G':
        case 'g':
            StartDrive(+1, nowMs);
            break;
        case 'B':
        case 'b':
            StartDrive(-1, nowMs);
            break;
        case 'S':
        case 's':
            StopAll();
            WriteEvent("stopped", nowMs);
            break;
        case 'H':
        case 'h':
        case '?':
            DiagUart_WriteString("FW,commands,G=forward,B=reverse,S=stop,H=help\r\n");
            break;
        default:
            break;
    }
}
