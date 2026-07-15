/**
 ******************************************************************************
 * @file    main_pwm_gyro_sweep.c
 * @brief   J-Link controlled low-duty mecanum calibration with WIT gyro logs.
 *
 * USART1 remap PB6/PB7: J-Link diagnostic link, 115200 8N1.
 * USART2 remap PD5/PD6: WIT gyro, default 9600 8N1.
 * This image starts stopped. Every motion command expires after 1200 ms.
 ******************************************************************************
 */

#include "app_ladrc.h"
#include "bsp_diag_uart.h"
#include "bsp_gyro_wit.h"
#include "bsp_mecanum.h"
#include "bsp_motor_safe.h"

#ifndef GYRO_BAUDRATE
#define GYRO_BAUDRATE 9600U
#endif

#define DIAG_BAUDRATE             115200U
#define PWM_STEP_PERCENT          5U
#define PWM_MIN_PERCENT           18U
#define PWM_MAX_PERCENT           100U
#define TURN_INNER_NUMERATOR      3U
#define TURN_INNER_DENOMINATOR    5U
#define MOTION_TIMEOUT_MS         285U
#define GYRO_REPORT_INTERVAL_MS   100U
#define LADRC_TARGET_TENTHS       0
#define LADRC_KP_NUMERATOR        1
#define LADRC_KP_DENOMINATOR      10

typedef enum
{
    PWM_SWEEP_STOPPED = 0,
    PWM_SWEEP_FORWARD,
    PWM_SWEEP_REVERSE,
    PWM_SWEEP_LEFT,
    PWM_SWEEP_RIGHT
} PwmSweepMotion_t;

static volatile uint32_t s_uptimeMs = 0U;
static uint8_t s_pwmPercent = PWM_MIN_PERCENT;
static PwmSweepMotion_t s_motion = PWM_SWEEP_STOPPED;
static char s_motionCode = 'S';
static uint32_t s_motionDeadlineMs = 0U;
static uint32_t s_lastGyroReportMs = 0U;
static int16_t s_yawBaselineTenths = 0;
static uint8_t s_yawBaselineValid = 0U;
static LadrcController_t s_yawLadrc;

void SysTick_Handler(void)
{
    ++s_uptimeMs;
}

static int16_t PwmSweep_AngleDeltaTenths(int16_t current, int16_t baseline)
{
    int16_t delta = (int16_t)(current - baseline);

    if (delta > 1800)
    {
        delta = (int16_t)(delta - 3600);
    }
    else if (delta < -1800)
    {
        delta = (int16_t)(delta + 3600);
    }
    return delta;
}

static void PwmSweep_PrintHelp(void)
{
    DiagUart_WriteString(
        "PWM_SWEEP F=forward B=reverse L=left R=right +=pwm_plus "
        "-=pwm_minus S/0=stop P=status H=help; raw_duty=18..100 step=5 "
        "duration_ms=285\r\n");
}

static void PwmSweep_PrintStatus(void)
{
    const GyroWitState_t *gyro = GyroWit_GetState();

    DiagUart_WriteString("PWM_STATUS,pwm=");
    DiagUart_WriteUInt32(s_pwmPercent);
    DiagUart_WriteString(",motion=");
    DiagUart_WriteChar(s_motionCode);
    DiagUart_WriteString(",gyro_frames=");
    DiagUart_WriteUInt32(gyro->validFrameCount);
    DiagUart_WriteString(",angle_frames=");
    DiagUart_WriteUInt32(gyro->angleFrameCount);
    DiagUart_WriteString(",checksum_errors=");
    DiagUart_WriteUInt32(gyro->checksumErrorCount);
    DiagUart_WriteString("\r\n");
}

static void PwmSweep_Stop(const char *reason)
{
    Mecanum_Enable(0U);
    s_motion = PWM_SWEEP_STOPPED;
    s_motionCode = 'S';
    s_yawBaselineValid = 0U;
    DiagUart_WriteString("PWM_STOP,at_ms=");
    DiagUart_WriteUInt32(s_uptimeMs);
    DiagUart_WriteString(",reason=");
    DiagUart_WriteString(reason);
    DiagUart_WriteString("\r\n");
}

static void PwmSweep_SetRawWheels(int16_t lf, int16_t rf,
                                  int16_t lr, int16_t rr)
{
    Mecanum_Enable(1U);
    Mecanum_SetWheelRaw(MECANUM_WHEEL_LF, lf);
    Mecanum_SetWheelRaw(MECANUM_WHEEL_RF, rf);
    Mecanum_SetWheelRaw(MECANUM_WHEEL_LR, lr);
    Mecanum_SetWheelRaw(MECANUM_WHEEL_RR, rr);
}

static void PwmSweep_Start(PwmSweepMotion_t motion)
{
    int16_t pwm = (int16_t)s_pwmPercent;
    int16_t inner = (int16_t)(((uint32_t)s_pwmPercent *
                               TURN_INNER_NUMERATOR) /
                              TURN_INNER_DENOMINATOR);
    const GyroWitState_t *gyro = GyroWit_GetState();

    if (inner < 1)
    {
        inner = 1;
    }

    switch (motion)
    {
        case PWM_SWEEP_FORWARD:
            PwmSweep_SetRawWheels(pwm, pwm, pwm, pwm);
            s_motionCode = 'F';
            break;
        case PWM_SWEEP_REVERSE:
            PwmSweep_SetRawWheels(-pwm, -pwm, -pwm, -pwm);
            s_motionCode = 'B';
            break;
        case PWM_SWEEP_LEFT:
            PwmSweep_SetRawWheels(inner, pwm, inner, pwm);
            s_motionCode = 'L';
            break;
        case PWM_SWEEP_RIGHT:
            PwmSweep_SetRawWheels(pwm, inner, pwm, inner);
            s_motionCode = 'R';
            break;
        default:
            PwmSweep_Stop("INVALID_MOTION");
            return;
    }

    s_motion = motion;
    s_motionDeadlineMs = s_uptimeMs + MOTION_TIMEOUT_MS;
    s_lastGyroReportMs = s_uptimeMs;
    s_yawBaselineTenths = gyro->yawTenthsDeg;
    s_yawBaselineValid = (gyro->angleFrameCount != 0U) ? 1U : 0U;
    Ladrc_Reset(&s_yawLadrc, LADRC_TARGET_TENTHS, 0);

    DiagUart_WriteString("PWM_START,at_ms=");
    DiagUart_WriteUInt32(s_uptimeMs);
    DiagUart_WriteString(",motion=");
    DiagUart_WriteChar(s_motionCode);
    DiagUart_WriteString(",raw_pwm=");
    DiagUart_WriteUInt32(s_pwmPercent);
    DiagUart_WriteString(",inner_pwm=");
    DiagUart_WriteInt32(inner);
    DiagUart_WriteString(",gyro_baseline_tenths=");
    DiagUart_WriteInt32(s_yawBaselineTenths);
    DiagUart_WriteString("\r\n");
}

static void PwmSweep_HandleCommand(char command)
{
    if ((command >= 'a') && (command <= 'z'))
    {
        command = (char)(command - 'a' + 'A');
    }

    if ((command == '\r') || (command == '\n') || (command == ' '))
    {
        return;
    }

    DiagUart_WriteString("PWM_CMD,at_ms=");
    DiagUart_WriteUInt32(s_uptimeMs);
    DiagUart_WriteString(",command=");
    DiagUart_WriteChar(command);
    DiagUart_WriteString("\r\n");

    switch (command)
    {
        case 'F': PwmSweep_Start(PWM_SWEEP_FORWARD); break;
        case 'B': PwmSweep_Start(PWM_SWEEP_REVERSE); break;
        case 'L': PwmSweep_Start(PWM_SWEEP_LEFT); break;
        case 'R': PwmSweep_Start(PWM_SWEEP_RIGHT); break;
        case '+':
            PwmSweep_Stop("PWM_CHANGE");
            if (s_pwmPercent <= (PWM_MAX_PERCENT - PWM_STEP_PERCENT))
            {
                s_pwmPercent = (uint8_t)(s_pwmPercent + PWM_STEP_PERCENT);
            }
            PwmSweep_PrintStatus();
            break;
        case '-':
            PwmSweep_Stop("PWM_CHANGE");
            if (s_pwmPercent > PWM_MIN_PERCENT)
            {
                s_pwmPercent = (uint8_t)(s_pwmPercent - PWM_STEP_PERCENT);
            }
            PwmSweep_PrintStatus();
            break;
        case 'S': case '0': PwmSweep_Stop("COMMAND"); break;
        case 'P': PwmSweep_PrintStatus(); break;
        case 'H': case '?': PwmSweep_PrintHelp(); break;
        default: DiagUart_WriteString("PWM_CMD_UNKNOWN\r\n"); break;
    }
}

static void PwmSweep_LogGyro(void)
{
    const GyroWitState_t *gyro = GyroWit_GetState();
    int16_t delta = 0;
    int16_t ladrcOutput = 0;

    if ((s_motion == PWM_SWEEP_STOPPED) ||
        ((uint32_t)(s_uptimeMs - s_lastGyroReportMs) <
         GYRO_REPORT_INTERVAL_MS))
    {
        return;
    }

    s_lastGyroReportMs = s_uptimeMs;
    if (s_yawBaselineValid != 0U)
    {
        delta = PwmSweep_AngleDeltaTenths(gyro->yawTenthsDeg,
                                           s_yawBaselineTenths);
    }

    ladrcOutput = Ladrc_Update(&s_yawLadrc, LADRC_TARGET_TENTHS, delta);

    DiagUart_WriteString("GYRO,at_ms=");
    DiagUart_WriteUInt32(s_uptimeMs);
    DiagUart_WriteString(",motion=");
    DiagUart_WriteChar(s_motionCode);
    DiagUart_WriteString(",yaw_tenths=");
    DiagUart_WriteInt32(gyro->yawTenthsDeg);
    DiagUart_WriteString(",yaw_delta_tenths=");
    DiagUart_WriteInt32(delta);
    DiagUart_WriteString(",angle_frames=");
    DiagUart_WriteUInt32(gyro->angleFrameCount);
    DiagUart_WriteString("\r\n");

    DiagUart_WriteString("LADRC,at_ms=");
    DiagUart_WriteUInt32(s_uptimeMs);
    DiagUart_WriteString(",motion=");
    DiagUart_WriteChar(s_motionCode);
    DiagUart_WriteString(",target_tenths=");
    DiagUart_WriteInt32(s_yawLadrc.targetTenthsDeg);
    DiagUart_WriteString(",measured_tenths=");
    DiagUart_WriteInt32(s_yawLadrc.measuredTenthsDeg);
    DiagUart_WriteString(",estimate_tenths=");
    DiagUart_WriteInt32(s_yawLadrc.estimateTenthsDeg);
    DiagUart_WriteString(",error_tenths=");
    DiagUart_WriteInt32(s_yawLadrc.errorTenthsDeg);
    DiagUart_WriteString(",output_percent=");
    DiagUart_WriteInt32(ladrcOutput);
    DiagUart_WriteString("\r\n");
}

int main(void)
{
    char command;

    MotorSafe_InitOff();
    SystemCoreClockUpdate();
    Mecanum_InitOff();
    DiagUart_Init(DIAG_BAUDRATE);
    GyroWit_Init(GYRO_BAUDRATE);
    Ladrc_Init(&s_yawLadrc, LADRC_KP_NUMERATOR, LADRC_KP_DENOMINATOR);
    Ladrc_Reset(&s_yawLadrc, LADRC_TARGET_TENTHS, 0);

    if (SysTick_Config(SystemCoreClock / 1000U) != 0U)
    {
        MotorSafe_ForceOff();
        while (1)
        {
        }
    }

    DiagUart_WriteString("BOOT,pwm_gyro_sweep,diag=USART1_PB6_PB7,gyro=USART2_PD5_PD6,gyro_baud=");
    DiagUart_WriteUInt32(GYRO_BAUDRATE);
    DiagUart_WriteString("\r\n");
    PwmSweep_PrintHelp();
    PwmSweep_PrintStatus();

    while (1)
    {
        (void)GyroWit_Poll();

        while (DiagUart_TryReadChar(&command) != 0U)
        {
            PwmSweep_HandleCommand(command);
        }

        if ((s_motion != PWM_SWEEP_STOPPED) &&
            ((int32_t)(s_uptimeMs - s_motionDeadlineMs) >= 0))
        {
            PwmSweep_Stop("TIMEOUT");
        }

        Mecanum_Update(s_uptimeMs);
        PwmSweep_LogGyro();
    }
}
