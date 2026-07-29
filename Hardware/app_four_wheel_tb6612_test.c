/**
 ******************************************************************************
 * @file    app_four_wheel_tb6612_test.c
 * @brief   Long GPIO force bring-up: FRONT then REAR, physical pin readback.
 *
 * Auto loop after 1 s:
 *   FRONT_L 100% GPIO 4s -> stop -> FRONT_R 100% GPIO 4s -> stop
 *   REAR_A  100% GPIO 4s -> stop -> REAR_B  100% GPIO 4s -> stop -> repeat
 *
 * Logs pe6/pa2/pa3 and rear pc8/pc9/stby so wiring can be checked without scope.
 ******************************************************************************
 */

#include "app_four_wheel_tb6612_test.h"

#include "bsp_aux_tb6612.h"
#include "bsp_diag_uart.h"
#include "bsp_motor.h"

#define DRIVE_MS           4000U
#define STOP_MS            1000U
#define LOG_PERIOD_MS       250U
#define IDLE_LOG_MS        1000U
#define AUTO_START_MS      1000U

typedef enum
{
    ST_IDLE = 0,
    ST_FRONT_L,
    ST_FRONT_L_STOP,
    ST_FRONT_R,
    ST_FRONT_R_STOP,
    ST_REAR_A,
    ST_REAR_A_STOP,
    ST_REAR_B,
    ST_REAR_B_STOP
} Stage_t;

static Stage_t s_stage;
static uint32_t s_stageStartMs;
static uint32_t s_lastLogMs;
static uint32_t s_bootMs;
static uint8_t s_running;
static uint8_t s_autoStarted;

static uint8_t TimeReached(uint32_t nowMs, uint32_t deadlineMs)
{
    return ((int32_t)(nowMs - deadlineMs) >= 0) ? 1U : 0U;
}

static const char *StageName(Stage_t stage)
{
    switch (stage)
    {
        case ST_FRONT_L: return "FRONT_L_GPIO100";
        case ST_FRONT_L_STOP: return "FRONT_L_STOP";
        case ST_FRONT_R: return "FRONT_R_GPIO100";
        case ST_FRONT_R_STOP: return "FRONT_R_STOP";
        case ST_REAR_A: return "REAR_A_GPIO100";
        case ST_REAR_A_STOP: return "REAR_A_STOP";
        case ST_REAR_B: return "REAR_B_GPIO100";
        case ST_REAR_B_STOP: return "REAR_B_STOP";
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
    DiagUart_WriteString(",pc8,");
    DiagUart_WriteUInt32(AuxTb6612_GetPwmGpioLevel(AUX_TB6612_MOTOR_A));
    DiagUart_WriteString(",pc9,");
    DiagUart_WriteUInt32(AuxTb6612_GetPwmGpioLevel(AUX_TB6612_MOTOR_B));
    DiagUart_WriteString(",ain,");
    DiagUart_WriteUInt32(AuxTb6612_GetDirBits(AUX_TB6612_MOTOR_A));
    DiagUart_WriteString(",bin,");
    DiagUart_WriteUInt32(AuxTb6612_GetDirBits(AUX_TB6612_MOTOR_B));
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
        case ST_FRONT_L:
            AuxTb6612_StopAll();
            Motor_Coast(MOTOR_RIGHT);
            Motor_ForceGpioFull(MOTOR_LEFT, +1);
            break;
        case ST_FRONT_R:
            AuxTb6612_StopAll();
            Motor_Coast(MOTOR_LEFT);
            Motor_ForceGpioFull(MOTOR_RIGHT, +1);
            break;
        case ST_REAR_A:
            Motor_Coast(MOTOR_LEFT);
            Motor_Coast(MOTOR_RIGHT);
            Motor_Enable(0U);
            Motor_RestorePwmAf();
            AuxTb6612_ForceGpioFull(AUX_TB6612_MOTOR_A, +1);
            break;
        case ST_REAR_B:
            Motor_Coast(MOTOR_LEFT);
            Motor_Coast(MOTOR_RIGHT);
            Motor_Enable(0U);
            Motor_RestorePwmAf();
            AuxTb6612_ForceGpioFull(AUX_TB6612_MOTOR_B, +1);
            break;
        case ST_FRONT_L_STOP:
        case ST_FRONT_R_STOP:
        case ST_REAR_A_STOP:
        case ST_REAR_B_STOP:
        default:
            AuxTb6612_StopAll();
            Motor_Coast(MOTOR_LEFT);
            Motor_Coast(MOTOR_RIGHT);
            Motor_Enable(0U);
            Motor_RestorePwmAf();
            break;
    }

    WriteEvent("stage_begin", nowMs);
}

static void StartLoop(uint32_t nowMs)
{
    StopAll();
    s_running = 1U;
    EnterStage(ST_FRONT_L, nowMs);
}

void FourWheelTb6612Test_Init(uint32_t nowMs)
{
    StopAll();
    s_stageStartMs = nowMs;
    s_lastLogMs = nowMs;
    s_bootMs = nowMs;
    s_autoStarted = 0U;

    DiagUart_WriteString("FW,boot,fw=gpio_force_front_rear,safe_idle=1\r\n");
    DiagUart_WriteString("FW,config,front=pa2_pa3_pe2_pe6,rear=pc8_pc9_pa5_pa4_pd8_pd9_pb8\r\n");
    DiagUart_WriteString("FW,note,auto=FL,FR,RA,RB_each_4s_gpio100;S=stop,G=restart\r\n");
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
        StartLoop(nowMs);
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
        case ST_FRONT_L:
            if (elapsed >= DRIVE_MS) EnterStage(ST_FRONT_L_STOP, nowMs);
            break;
        case ST_FRONT_L_STOP:
            if (elapsed >= STOP_MS) EnterStage(ST_FRONT_R, nowMs);
            break;
        case ST_FRONT_R:
            if (elapsed >= DRIVE_MS) EnterStage(ST_FRONT_R_STOP, nowMs);
            break;
        case ST_FRONT_R_STOP:
            if (elapsed >= STOP_MS) EnterStage(ST_REAR_A, nowMs);
            break;
        case ST_REAR_A:
            if (elapsed >= DRIVE_MS) EnterStage(ST_REAR_A_STOP, nowMs);
            break;
        case ST_REAR_A_STOP:
            if (elapsed >= STOP_MS) EnterStage(ST_REAR_B, nowMs);
            break;
        case ST_REAR_B:
            if (elapsed >= DRIVE_MS) EnterStage(ST_REAR_B_STOP, nowMs);
            break;
        case ST_REAR_B_STOP:
            if (elapsed >= STOP_MS)
            {
                WriteEvent("loop_restart", nowMs);
                EnterStage(ST_FRONT_L, nowMs);
            }
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
            StartLoop(nowMs);
            break;
        case 'S':
        case 's':
            StopAll();
            WriteEvent("stopped", nowMs);
            break;
        case 'H':
        case 'h':
        case '?':
            DiagUart_WriteString("FW,commands,G=restart,S=stop,H=help\r\n");
            break;
        default:
            break;
    }
}
