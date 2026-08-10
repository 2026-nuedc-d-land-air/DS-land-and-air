/**
 ******************************************************************************
 * @file    app_expansion_actuator_test.c
 * @brief   扩展执行器安全测试流程。
 *
 * G: 扩展 TB6612 两台 G513 的原始正反转测试（左 A=PF1/2+PA4/5，右 B=PF3/4+PD8/9，STBY=PB9）。
 * P: 两块 A4988 的单步、16 步、整圈和双机整圈测试。
 * E: 两块 A4988 仅使能线圈 5 s，不发 STEP；用于确认保持力。
 * A/B: A4988-1 / A4988-2 低速单独 200 步整圈尝试（仅全步进时为一圈）。
 * C/D: 两块 A4988 同时低速同向正/反 200 步整圈尝试。
 * 1/2/3/4: A4988-1+/A4988-1-/A4988-2+/A4988-2- 单步。
 * 5/6/7/8: 对应四个方向的低速 16 步观察动作。
 * S: 立即禁用所有扩展驱动。
 *
 * A4988 无编码器，issued_steps 是命令台账而非位置反馈。
 ******************************************************************************
 */

#include "app_expansion_actuator_test.h"

#include "bsp_a4988.h"
#include "bsp_aux_tb6612.h"
#include "bsp_diag_uart.h"

#define EXP_LOG_PERIOD_MS              100U
#define EXP_AUX_PWM_PERCENT             25
#define EXP_AUX_DRIVE_MS              1000U
#define EXP_AUX_STOP_MS                500U
#define EXP_STEP_RATE_SPS              100U
#define EXP_MANUAL_STEP_RATE_SPS        20U
#define EXP_SHORT_STEPS                 16U
#define EXP_FULL_STEPS_PER_REV         200U
#define EXP_MICROSTEP_DIVIDER            1U
#define EXP_TURN_STEPS  (EXP_FULL_STEPS_PER_REV * EXP_MICROSTEP_DIVIDER)
#define EXP_STEP_SETTLE_MS             300U
#define EXP_HOLD_TEST_MS              5000U

typedef enum
{
    EXP_STAGE_IDLE = 0,
    EXP_STAGE_AUX_A_POS,
    EXP_STAGE_AUX_A_STOP_1,
    EXP_STAGE_AUX_A_NEG,
    EXP_STAGE_AUX_A_STOP_2,
    EXP_STAGE_AUX_B_POS,
    EXP_STAGE_AUX_B_STOP_1,
    EXP_STAGE_AUX_B_NEG,
    EXP_STAGE_AUX_B_STOP_2,
    EXP_STAGE_AUX_FINISHED,
    EXP_STAGE_STP_M1_POS_16,
    EXP_STAGE_STP_M1_NEG_16,
    EXP_STAGE_STP_M2_POS_16,
    EXP_STAGE_STP_M2_NEG_16,
    EXP_STAGE_STP_M1_POS_TURN,
    EXP_STAGE_STP_M1_NEG_TURN,
    EXP_STAGE_STP_M2_POS_TURN,
    EXP_STAGE_STP_M2_NEG_TURN,
    EXP_STAGE_STP_BOTH_POS_TURN,
    EXP_STAGE_STP_BOTH_NEG_TURN,
    EXP_STAGE_STP_FINISHED,
    EXP_STAGE_STP_HOLD_BOTH,
    EXP_STAGE_MANUAL_STEP,
    EXP_STAGE_FAULT
} ExpansionStage_t;

static ExpansionStage_t s_stage = EXP_STAGE_IDLE;
static uint32_t s_stageStartMs = 0U;
static uint32_t s_stepFinishedMs = 0U;
static uint32_t s_lastLogMs = 0U;
static uint8_t s_running = 0U;

static uint8_t ExpansionActuator_TimeReached(uint32_t nowMs, uint32_t deadlineMs)
{
    return ((int32_t)(nowMs - deadlineMs) >= 0) ? 1U : 0U;
}

static const char *ExpansionActuator_StageName(ExpansionStage_t stage)
{
    switch (stage)
    {
        case EXP_STAGE_AUX_A_POS:       return "AUX_A_RAW_POS";
        case EXP_STAGE_AUX_A_STOP_1:    return "AUX_A_STOP_1";
        case EXP_STAGE_AUX_A_NEG:       return "AUX_A_RAW_NEG";
        case EXP_STAGE_AUX_A_STOP_2:    return "AUX_A_STOP_2";
        case EXP_STAGE_AUX_B_POS:       return "AUX_B_RAW_POS";
        case EXP_STAGE_AUX_B_STOP_1:    return "AUX_B_STOP_1";
        case EXP_STAGE_AUX_B_NEG:       return "AUX_B_RAW_NEG";
        case EXP_STAGE_AUX_B_STOP_2:    return "AUX_B_STOP_2";
        case EXP_STAGE_AUX_FINISHED:    return "AUX_FINISHED";
        case EXP_STAGE_STP_M1_POS_16:   return "STP_M1_POS_16";
        case EXP_STAGE_STP_M1_NEG_16:   return "STP_M1_NEG_16";
        case EXP_STAGE_STP_M2_POS_16:   return "STP_M2_POS_16";
        case EXP_STAGE_STP_M2_NEG_16:   return "STP_M2_NEG_16";
        case EXP_STAGE_STP_M1_POS_TURN: return "STP_M1_POS_TURN";
        case EXP_STAGE_STP_M1_NEG_TURN: return "STP_M1_NEG_TURN";
        case EXP_STAGE_STP_M2_POS_TURN: return "STP_M2_POS_TURN";
        case EXP_STAGE_STP_M2_NEG_TURN: return "STP_M2_NEG_TURN";
        case EXP_STAGE_STP_BOTH_POS_TURN:return "STP_BOTH_POS_TURN";
        case EXP_STAGE_STP_BOTH_NEG_TURN:return "STP_BOTH_NEG_TURN";
        case EXP_STAGE_STP_FINISHED:    return "STP_FINISHED";
        case EXP_STAGE_STP_HOLD_BOTH:   return "STP_HOLD_BOTH";
        case EXP_STAGE_MANUAL_STEP:     return "MANUAL_STEP";
        case EXP_STAGE_FAULT:           return "FAULT";
        case EXP_STAGE_IDLE:
        default:                        return "IDLE";
    }
}

static uint8_t ExpansionActuator_IsStepperStage(ExpansionStage_t stage)
{
    return ((stage >= EXP_STAGE_STP_M1_POS_16) &&
            (stage <= EXP_STAGE_STP_FINISHED)) ||
           (stage == EXP_STAGE_MANUAL_STEP);
}

static void ExpansionActuator_WriteEvent(const char *event, uint32_t nowMs)
{
    DiagUart_WriteString("ACT,event,");
    DiagUart_WriteString(event);
    DiagUart_WriteString(",t_ms,");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",stage,");
    DiagUart_WriteString(ExpansionActuator_StageName(s_stage));
    DiagUart_WriteString(",aux_stby,");
    DiagUart_WriteUInt32(AuxTb6612_IsEnabled());
    DiagUart_WriteString(",m1_issued,");
    DiagUart_WriteUInt32(A4988_GetIssuedSteps(A4988_MOTOR_1));
    DiagUart_WriteString(",m2_issued,");
    DiagUart_WriteUInt32(A4988_GetIssuedSteps(A4988_MOTOR_2));
    DiagUart_WriteString("\r\n");
}

static void ExpansionActuator_WriteStatus(uint32_t nowMs)
{
    DiagUart_WriteString("ACT,sample,");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteChar(',');
    DiagUart_WriteString(ExpansionActuator_StageName(s_stage));
    DiagUart_WriteString(",aux_stby,");
    DiagUart_WriteUInt32(AuxTb6612_IsEnabled());
    DiagUart_WriteString(",m1_en,");
    DiagUart_WriteUInt32(A4988_IsEnabled(A4988_MOTOR_1));
    DiagUart_WriteString(",m1_dir,");
    DiagUart_WriteInt32(A4988_GetDirection(A4988_MOTOR_1));
    DiagUart_WriteString(",m1_req,");
    DiagUart_WriteUInt32(A4988_GetRequestedSteps(A4988_MOTOR_1));
    DiagUart_WriteString(",m1_issued,");
    DiagUart_WriteUInt32(A4988_GetIssuedSteps(A4988_MOTOR_1));
    DiagUart_WriteString(",m1_remaining,");
    DiagUart_WriteUInt32(A4988_GetRemainingSteps(A4988_MOTOR_1));
    DiagUart_WriteString(",m2_en,");
    DiagUart_WriteUInt32(A4988_IsEnabled(A4988_MOTOR_2));
    DiagUart_WriteString(",m2_dir,");
    DiagUart_WriteInt32(A4988_GetDirection(A4988_MOTOR_2));
    DiagUart_WriteString(",m2_req,");
    DiagUart_WriteUInt32(A4988_GetRequestedSteps(A4988_MOTOR_2));
    DiagUart_WriteString(",m2_issued,");
    DiagUart_WriteUInt32(A4988_GetIssuedSteps(A4988_MOTOR_2));
    DiagUart_WriteString(",m2_remaining,");
    DiagUart_WriteUInt32(A4988_GetRemainingSteps(A4988_MOTOR_2));
    DiagUart_WriteString(",feedback,open_loop_only\r\n");
}

static void ExpansionActuator_StopAll(void)
{
    AuxTb6612_StopAll();
    A4988_StopAll();
    s_running = 0U;
    s_stage = EXP_STAGE_IDLE;
    s_stepFinishedMs = 0U;
}

static uint8_t ExpansionActuator_StartStepperMove(ExpansionStage_t stage,
                                                    uint32_t nowMs)
{
    uint8_t success = 1U;

    A4988_StopAll();
    switch (stage)
    {
        case EXP_STAGE_STP_M1_POS_16:
            success = A4988_StartMove(A4988_MOTOR_1, 1, EXP_SHORT_STEPS,
                                       EXP_STEP_RATE_SPS, nowMs);
            break;
        case EXP_STAGE_STP_M1_NEG_16:
            success = A4988_StartMove(A4988_MOTOR_1, -1, EXP_SHORT_STEPS,
                                       EXP_STEP_RATE_SPS, nowMs);
            break;
        case EXP_STAGE_STP_M2_POS_16:
            success = A4988_StartMove(A4988_MOTOR_2, 1, EXP_SHORT_STEPS,
                                       EXP_STEP_RATE_SPS, nowMs);
            break;
        case EXP_STAGE_STP_M2_NEG_16:
            success = A4988_StartMove(A4988_MOTOR_2, -1, EXP_SHORT_STEPS,
                                       EXP_STEP_RATE_SPS, nowMs);
            break;
        case EXP_STAGE_STP_M1_POS_TURN:
            success = A4988_StartMove(A4988_MOTOR_1, 1, EXP_TURN_STEPS,
                                       EXP_STEP_RATE_SPS, nowMs);
            break;
        case EXP_STAGE_STP_M1_NEG_TURN:
            success = A4988_StartMove(A4988_MOTOR_1, -1, EXP_TURN_STEPS,
                                       EXP_STEP_RATE_SPS, nowMs);
            break;
        case EXP_STAGE_STP_M2_POS_TURN:
            success = A4988_StartMove(A4988_MOTOR_2, 1, EXP_TURN_STEPS,
                                       EXP_STEP_RATE_SPS, nowMs);
            break;
        case EXP_STAGE_STP_M2_NEG_TURN:
            success = A4988_StartMove(A4988_MOTOR_2, -1, EXP_TURN_STEPS,
                                       EXP_STEP_RATE_SPS, nowMs);
            break;
        case EXP_STAGE_STP_BOTH_POS_TURN:
            success = A4988_StartMove(A4988_MOTOR_1, 1, EXP_TURN_STEPS,
                                       EXP_STEP_RATE_SPS, nowMs);
            if (success != 0U)
            {
                success = A4988_StartMove(A4988_MOTOR_2, 1, EXP_TURN_STEPS,
                                           EXP_STEP_RATE_SPS, nowMs);
            }
            break;
        case EXP_STAGE_STP_BOTH_NEG_TURN:
            success = A4988_StartMove(A4988_MOTOR_1, -1, EXP_TURN_STEPS,
                                       EXP_STEP_RATE_SPS, nowMs);
            if (success != 0U)
            {
                success = A4988_StartMove(A4988_MOTOR_2, -1, EXP_TURN_STEPS,
                                           EXP_STEP_RATE_SPS, nowMs);
            }
            break;
        default:
            success = 0U;
            break;
    }

    return success;
}

static void ExpansionActuator_EnterStage(ExpansionStage_t stage, uint32_t nowMs)
{
    s_stage = stage;
    s_stageStartMs = nowMs;
    s_stepFinishedMs = 0U;

    switch (stage)
    {
        case EXP_STAGE_AUX_A_POS:
            A4988_StopAll();
            AuxTb6612_Enable(1U);
            AuxTb6612_SetRawSpeed(AUX_TB6612_MOTOR_A, EXP_AUX_PWM_PERCENT);
            break;
        case EXP_STAGE_AUX_A_NEG:
            AuxTb6612_Enable(1U);
            AuxTb6612_SetRawSpeed(AUX_TB6612_MOTOR_A, -EXP_AUX_PWM_PERCENT);
            break;
        case EXP_STAGE_AUX_B_POS:
            AuxTb6612_Enable(1U);
            AuxTb6612_SetRawSpeed(AUX_TB6612_MOTOR_B, EXP_AUX_PWM_PERCENT);
            break;
        case EXP_STAGE_AUX_B_NEG:
            AuxTb6612_Enable(1U);
            AuxTb6612_SetRawSpeed(AUX_TB6612_MOTOR_B, -EXP_AUX_PWM_PERCENT);
            break;
        case EXP_STAGE_AUX_A_STOP_1:
        case EXP_STAGE_AUX_A_STOP_2:
        case EXP_STAGE_AUX_B_STOP_1:
        case EXP_STAGE_AUX_B_STOP_2:
        case EXP_STAGE_AUX_FINISHED:
            AuxTb6612_StopAll();
            break;
        case EXP_STAGE_STP_FINISHED:
            A4988_StopAll();
            break;
        case EXP_STAGE_STP_HOLD_BOTH:
            AuxTb6612_StopAll();
            A4988_StopAll();
            A4988_SetEnabled(A4988_MOTOR_1, 1U);
            A4988_SetEnabled(A4988_MOTOR_2, 1U);
            break;
        case EXP_STAGE_IDLE:
        case EXP_STAGE_FAULT:
            AuxTb6612_StopAll();
            A4988_StopAll();
            break;
        default:
            AuxTb6612_StopAll();
            if (ExpansionActuator_StartStepperMove(stage, nowMs) == 0U)
            {
                s_stage = EXP_STAGE_FAULT;
                s_running = 0U;
            }
            break;
    }

    ExpansionActuator_WriteEvent("stage_begin", nowMs);
}

static void ExpansionActuator_AdvanceStepperStage(uint32_t nowMs)
{
    switch (s_stage)
    {
        case EXP_STAGE_STP_M1_POS_16:
            ExpansionActuator_EnterStage(EXP_STAGE_STP_M1_NEG_16, nowMs);
            break;
        case EXP_STAGE_STP_M1_NEG_16:
            ExpansionActuator_EnterStage(EXP_STAGE_STP_M2_POS_16, nowMs);
            break;
        case EXP_STAGE_STP_M2_POS_16:
            ExpansionActuator_EnterStage(EXP_STAGE_STP_M2_NEG_16, nowMs);
            break;
        case EXP_STAGE_STP_M2_NEG_16:
            ExpansionActuator_EnterStage(EXP_STAGE_STP_M1_POS_TURN, nowMs);
            break;
        case EXP_STAGE_STP_M1_POS_TURN:
            ExpansionActuator_EnterStage(EXP_STAGE_STP_M1_NEG_TURN, nowMs);
            break;
        case EXP_STAGE_STP_M1_NEG_TURN:
            ExpansionActuator_EnterStage(EXP_STAGE_STP_M2_POS_TURN, nowMs);
            break;
        case EXP_STAGE_STP_M2_POS_TURN:
            ExpansionActuator_EnterStage(EXP_STAGE_STP_M2_NEG_TURN, nowMs);
            break;
        case EXP_STAGE_STP_M2_NEG_TURN:
            ExpansionActuator_EnterStage(EXP_STAGE_STP_BOTH_POS_TURN, nowMs);
            break;
        case EXP_STAGE_STP_BOTH_POS_TURN:
            ExpansionActuator_EnterStage(EXP_STAGE_STP_BOTH_NEG_TURN, nowMs);
            break;
        case EXP_STAGE_STP_BOTH_NEG_TURN:
            ExpansionActuator_EnterStage(EXP_STAGE_STP_FINISHED, nowMs);
            s_running = 0U;
            break;
        case EXP_STAGE_MANUAL_STEP:
            ExpansionActuator_StopAll();
            ExpansionActuator_WriteEvent("manual_finished", nowMs);
            break;
        default:
            ExpansionActuator_StopAll();
            break;
    }
}

static void ExpansionActuator_StartManualMove(A4988Motor_t motor,
                                               int8_t direction,
                                               uint32_t requestedSteps,
                                               uint32_t nowMs)
{
    ExpansionActuator_StopAll();
    s_running = 1U;
    s_stage = EXP_STAGE_MANUAL_STEP;
    s_stageStartMs = nowMs;
    s_stepFinishedMs = 0U;
    if (A4988_StartMove(motor, direction, requestedSteps,
                        EXP_MANUAL_STEP_RATE_SPS, nowMs) == 0U)
    {
        s_stage = EXP_STAGE_FAULT;
        s_running = 0U;
    }
    ExpansionActuator_WriteEvent("manual_move_begin", nowMs);
}

/* Both StartMove calls receive the same timebase, so their scheduled STEP
 * rising edges are emitted in the same Update() pass.  This is a command
 * synchronization check only; neither A4988 has position feedback. */
static void ExpansionActuator_StartManualBothMove(int8_t direction,
                                                   uint32_t requestedSteps,
                                                   uint32_t nowMs)
{
    uint8_t success;

    ExpansionActuator_StopAll();
    s_running = 1U;
    s_stage = EXP_STAGE_MANUAL_STEP;
    s_stageStartMs = nowMs;
    s_stepFinishedMs = 0U;

    success = A4988_StartMove(A4988_MOTOR_1, direction, requestedSteps,
                              EXP_MANUAL_STEP_RATE_SPS, nowMs);
    if (success != 0U)
    {
        success = A4988_StartMove(A4988_MOTOR_2, direction, requestedSteps,
                                  EXP_MANUAL_STEP_RATE_SPS, nowMs);
    }

    if (success == 0U)
    {
        A4988_StopAll();
        s_stage = EXP_STAGE_FAULT;
        s_running = 0U;
    }

    ExpansionActuator_WriteEvent("manual_both_move_begin", nowMs);
}

void ExpansionActuatorTest_Init(uint32_t nowMs)
{
    ExpansionActuator_StopAll();
    s_stageStartMs = nowMs;
    s_lastLogMs = nowMs;

    DiagUart_WriteString("ACT,boot,fw=expansion_actuator,safe_idle=1,closed_loop=0\r\n");
    DiagUart_WriteString("ACT,config,aux_pwm_left_pf1_pf2,right_pf3_pf4,soft_tim7_1khz,aux_raw_only=1\r\n");
    DiagUart_WriteString("ACT,config,aux_dir_ain_pa4_pa5,bin_pd8_pd9,stby_pb9\r\n");
    DiagUart_WriteString("ACT,config,a4988_m1_step_dir_en=pe14_pe13_pc4,m2=pd14_pd13_pd15,en_active_low=1\r\n");
    DiagUart_WriteString("ACT,config,full_steps_per_rev=200,microstep_divider=1,turn_steps=200,feedback=open_loop_only\r\n");
    DiagUart_WriteString("ACT,commands,G=aux_raw_sequence,P=stepper_sequence,E=hold_5s,A/B=single_turn200,C/D=both_turn200,1/2/3/4=single_step,5/6/7/8=slow_16_steps,S=stop,H=help\r\n");
}

void ExpansionActuatorTest_Update(uint32_t nowMs)
{
    A4988_Update(nowMs);

    if ((s_running != 0U) &&
        (ExpansionActuator_TimeReached(nowMs, s_lastLogMs + EXP_LOG_PERIOD_MS) != 0U))
    {
        s_lastLogMs = nowMs;
        ExpansionActuator_WriteStatus(nowMs);
    }

    switch (s_stage)
    {
        case EXP_STAGE_AUX_A_POS:
            if (ExpansionActuator_TimeReached(nowMs, s_stageStartMs + EXP_AUX_DRIVE_MS) != 0U)
                ExpansionActuator_EnterStage(EXP_STAGE_AUX_A_STOP_1, nowMs);
            break;
        case EXP_STAGE_AUX_A_STOP_1:
            if (ExpansionActuator_TimeReached(nowMs, s_stageStartMs + EXP_AUX_STOP_MS) != 0U)
                ExpansionActuator_EnterStage(EXP_STAGE_AUX_A_NEG, nowMs);
            break;
        case EXP_STAGE_AUX_A_NEG:
            if (ExpansionActuator_TimeReached(nowMs, s_stageStartMs + EXP_AUX_DRIVE_MS) != 0U)
                ExpansionActuator_EnterStage(EXP_STAGE_AUX_A_STOP_2, nowMs);
            break;
        case EXP_STAGE_AUX_A_STOP_2:
            if (ExpansionActuator_TimeReached(nowMs, s_stageStartMs + EXP_AUX_STOP_MS) != 0U)
                ExpansionActuator_EnterStage(EXP_STAGE_AUX_B_POS, nowMs);
            break;
        case EXP_STAGE_AUX_B_POS:
            if (ExpansionActuator_TimeReached(nowMs, s_stageStartMs + EXP_AUX_DRIVE_MS) != 0U)
                ExpansionActuator_EnterStage(EXP_STAGE_AUX_B_STOP_1, nowMs);
            break;
        case EXP_STAGE_AUX_B_STOP_1:
            if (ExpansionActuator_TimeReached(nowMs, s_stageStartMs + EXP_AUX_STOP_MS) != 0U)
                ExpansionActuator_EnterStage(EXP_STAGE_AUX_B_NEG, nowMs);
            break;
        case EXP_STAGE_AUX_B_NEG:
            if (ExpansionActuator_TimeReached(nowMs, s_stageStartMs + EXP_AUX_DRIVE_MS) != 0U)
                ExpansionActuator_EnterStage(EXP_STAGE_AUX_B_STOP_2, nowMs);
            break;
        case EXP_STAGE_AUX_B_STOP_2:
            if (ExpansionActuator_TimeReached(nowMs, s_stageStartMs + EXP_AUX_STOP_MS) != 0U)
            {
                ExpansionActuator_EnterStage(EXP_STAGE_AUX_FINISHED, nowMs);
                AuxTb6612_StopAll();
                s_running = 0U;
                ExpansionActuator_WriteEvent("aux_raw_sequence_done", nowMs);
            }
            break;
        case EXP_STAGE_STP_HOLD_BOTH:
            if (ExpansionActuator_TimeReached(nowMs,
                                               s_stageStartMs + EXP_HOLD_TEST_MS) != 0U)
            {
                AuxTb6612_StopAll();
                A4988_StopAll();
                s_running = 0U;
                s_stage = EXP_STAGE_IDLE;
                ExpansionActuator_WriteEvent("hold_finished", nowMs);
            }
            break;
        default:
            break;
    }

    if ((s_running != 0U) && (ExpansionActuator_IsStepperStage(s_stage) != 0U))
    {
        if ((A4988_IsBusy(A4988_MOTOR_1) == 0U) &&
            (A4988_IsBusy(A4988_MOTOR_2) == 0U))
        {
            if (s_stepFinishedMs == 0U)
            {
                s_stepFinishedMs = nowMs;
                A4988_StopAll();
                ExpansionActuator_WriteEvent("step_move_done", nowMs);
            }
            else if (ExpansionActuator_TimeReached(nowMs,
                                                    s_stepFinishedMs + EXP_STEP_SETTLE_MS) != 0U)
            {
                ExpansionActuator_AdvanceStepperStage(nowMs);
            }
        }
    }
}

void ExpansionActuatorTest_HandleCommand(char command, uint32_t nowMs)
{
    switch (command)
    {
        case 'G':
        case 'g':
            ExpansionActuator_StopAll();
            s_running = 1U;
            ExpansionActuator_EnterStage(EXP_STAGE_AUX_A_POS, nowMs);
            break;
        case 'P':
        case 'p':
            ExpansionActuator_StopAll();
            s_running = 1U;
            ExpansionActuator_EnterStage(EXP_STAGE_STP_M1_POS_16, nowMs);
            break;
        case 'E':
        case 'e':
            ExpansionActuator_StopAll();
            s_running = 1U;
            ExpansionActuator_EnterStage(EXP_STAGE_STP_HOLD_BOTH, nowMs);
            break;
        case 'A':
        case 'a':
            ExpansionActuator_StartManualMove(A4988_MOTOR_1, 1,
                                               EXP_TURN_STEPS, nowMs);
            break;
        case 'B':
        case 'b':
            ExpansionActuator_StartManualMove(A4988_MOTOR_2, 1,
                                               EXP_TURN_STEPS, nowMs);
            break;
        case 'C':
        case 'c':
            ExpansionActuator_StartManualBothMove(1, EXP_TURN_STEPS, nowMs);
            break;
        case 'D':
        case 'd':
            ExpansionActuator_StartManualBothMove(-1, EXP_TURN_STEPS, nowMs);
            break;
        case '1':
            ExpansionActuator_StartManualMove(A4988_MOTOR_1, 1, 1U, nowMs);
            break;
        case '2':
            ExpansionActuator_StartManualMove(A4988_MOTOR_1, -1, 1U, nowMs);
            break;
        case '3':
            ExpansionActuator_StartManualMove(A4988_MOTOR_2, 1, 1U, nowMs);
            break;
        case '4':
            ExpansionActuator_StartManualMove(A4988_MOTOR_2, -1, 1U, nowMs);
            break;
        case '5':
            ExpansionActuator_StartManualMove(A4988_MOTOR_1, 1, EXP_SHORT_STEPS, nowMs);
            break;
        case '6':
            ExpansionActuator_StartManualMove(A4988_MOTOR_1, -1, EXP_SHORT_STEPS, nowMs);
            break;
        case '7':
            ExpansionActuator_StartManualMove(A4988_MOTOR_2, 1, EXP_SHORT_STEPS, nowMs);
            break;
        case '8':
            ExpansionActuator_StartManualMove(A4988_MOTOR_2, -1, EXP_SHORT_STEPS, nowMs);
            break;
        case 'S':
        case 's':
            ExpansionActuator_StopAll();
            DiagUart_WriteString("ACT,event,stopped,all_drivers_disabled=1\r\n");
            break;
        case 'H':
        case 'h':
        case '?':
            DiagUart_WriteString("ACT,commands,G=aux_raw_sequence,P=stepper_sequence,E=hold_5s,A/B=single_turn200,C/D=both_turn200,1/2/3/4=single_step,5/6/7/8=slow_16_steps,S=stop,H=help\r\n");
            break;
        default:
            break;
    }
}
