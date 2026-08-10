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
#include "stm32f10x_exti.h"

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
static volatile uint32_t s_gyroPd5FallingEdgeCount = 0U;
static volatile uint32_t s_gyroPd6FallingEdgeCount = 0U;
static uint8_t s_gyroPd5EdgeMonitorActive = 0U;

/* Diagnostic-only electrical evidence: count PD6 UART start/data edges
 * independently of the USART2 receiver and WIT frame parser. */
static void PwmSweep_InitGyroEdgeDiagnostic(void)
{
    s_gyroPd5FallingEdgeCount = 0U;
    s_gyroPd6FallingEdgeCount = 0U;
    s_gyroPd5EdgeMonitorActive = 0U;
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource6);
    EXTI->IMR |= EXTI_Line6;
    EXTI->EMR &= ~EXTI_Line6;
    EXTI->RTSR &= ~EXTI_Line6;
    EXTI->FTSR |= EXTI_Line6;
    EXTI->PR = EXTI_Line6;
    NVIC_SetPriority(EXTI9_5_IRQn, 0x60U);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void EXTI9_5_IRQHandler(void)
{
    uint32_t pending = EXTI->PR & (EXTI_Line5 | EXTI_Line6);

    if (pending != 0U)
    {
        EXTI->PR = pending;
    }
    if ((pending & EXTI_Line5) != 0U)
    {
        ++s_gyroPd5FallingEdgeCount;
    }
    if ((pending & EXTI_Line6) != 0U)
    {
        ++s_gyroPd6FallingEdgeCount;
    }
}

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
        "-=pwm_minus S/0=stop P=status E=pd5_pd6_edge_test T=gyro_loopback W=gyro_wire_test D=gyro_uart_diag H=help; raw_duty=18..100 step=5 "
        "duration_ms=285\r\n");
}

static void PwmSweep_PrintStatus(void)
{
    const volatile GyroWitState_t *gyro = GyroWit_GetState();

    DiagUart_WriteString("PWM_STATUS,pwm=");
    DiagUart_WriteUInt32(s_pwmPercent);
    DiagUart_WriteString(",motion=");
    DiagUart_WriteChar(s_motionCode);
    DiagUart_WriteString(",gyro_frames=");
    DiagUart_WriteUInt32(gyro->validFrameCount);
    DiagUart_WriteString(",angle_frames=");
    DiagUart_WriteUInt32(gyro->angleFrameCount);
    DiagUart_WriteString(",rate_frames=");
    DiagUart_WriteUInt32(gyro->angularVelocityFrameCount);
    DiagUart_WriteString(",raw_bytes=");
    DiagUart_WriteUInt32(gyro->rawByteCount);
    DiagUart_WriteString(",pd5_fall_edges=");
    DiagUart_WriteUInt32(s_gyroPd5FallingEdgeCount);
    DiagUart_WriteString(",pd6_fall_edges=");
    DiagUart_WriteUInt32(s_gyroPd6FallingEdgeCount);
    DiagUart_WriteString(",frame_heads=");
    DiagUart_WriteUInt32(gyro->frameHeadCount);
    DiagUart_WriteString(",discarded_bytes=");
    DiagUart_WriteUInt32(gyro->discardedByteCount);
    DiagUart_WriteString(",checksum_errors=");
    DiagUart_WriteUInt32(gyro->checksumErrorCount);
    DiagUart_WriteString("\r\n");
}

/* Monitor PD5 as an input as well, to detect a TX/RX connection that is
 * physically swapped despite its wire label.  UART2 TX is disabled only in
 * this PwmGyroSweep diagnostic mode; it is never used by line-following. */
static void PwmSweep_EnableDualPinEdgeMonitor(void)
{
    GPIO_InitTypeDef gpio;

    USART_Cmd(USART2, DISABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_5;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &gpio);

    s_gyroPd5FallingEdgeCount = 0U;
    s_gyroPd6FallingEdgeCount = 0U;
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource5);
    EXTI->IMR |= EXTI_Line5;
    EXTI->EMR &= ~EXTI_Line5;
    EXTI->RTSR &= ~EXTI_Line5;
    EXTI->FTSR |= EXTI_Line5;
    EXTI->PR = EXTI_Line5 | EXTI_Line6;
    s_gyroPd5EdgeMonitorActive = 1U;
    DiagUart_WriteString("GYRO_EDGE_TEST,pd5_pd6_monitor=enabled\r\n");
}

static void PwmSweep_DisablePd5EdgeMonitor(void)
{
    GPIO_InitTypeDef gpio;

    if (s_gyroPd5EdgeMonitorActive == 0U)
    {
        return;
    }

    EXTI->IMR &= ~EXTI_Line5;
    EXTI->PR = EXTI_Line5;
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_5;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &gpio);
    USART_Cmd(USART2, ENABLE);
    s_gyroPd5EdgeMonitorActive = 0U;
}

/*
 * Hardware-isolation diagnostic only.  With the gyro TX wire removed from
 * PD6 and PD5 shorted to PD6, this valid WIT 0x55/0x53 frame must be received
 * by USART2 itself.  It proves the MCU remap, RX pin and RXNE path without
 * needing a gyro data stream.  Do not run it while the gyro TX still drives
 * PD6 because two push-pull outputs would then be shorted together.
 */
static void PwmSweep_TestGyroUartLoopback(void)
{
    static const uint8_t loopbackFrame[11] = {
        0x55U, 0x53U, 0x00U, 0x00U, 0x00U, 0x00U,
        0x00U, 0x00U, 0x00U, 0x00U, 0xA8U
    };
    const volatile GyroWitState_t *gyro = GyroWit_GetState();
    uint32_t rawBefore = gyro->rawByteCount;
    uint8_t index;

    PwmSweep_DisablePd5EdgeMonitor();
    for (index = 0U; index < (uint8_t)sizeof(loopbackFrame); ++index)
    {
        USART_SendData(USART2, loopbackFrame[index]);
        while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
        {
        }
    }

    DiagUart_WriteString("GYRO_LOOPBACK,sent=11,raw_before=");
    DiagUart_WriteUInt32(rawBefore);
    DiagUart_WriteString(",raw_after=");
    DiagUart_WriteUInt32(gyro->rawByteCount);
    DiagUart_WriteString("\r\n");
    PwmSweep_PrintStatus();
}

static void PwmSweep_PrintGyroUartDiagnostic(void)
{
    uint32_t nvicWord = NVIC->ISER[(uint32_t)USART2_IRQn >> 5U];
    uint32_t nvicMask = 1UL << ((uint32_t)USART2_IRQn & 0x1FU);

    DiagUart_WriteString("GYRO_UART_DIAG,pd6_level=");
    DiagUart_WriteUInt32((uint32_t)GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6));
    DiagUart_WriteString(",mapr_usart2_remap=");
    DiagUart_WriteUInt32((AFIO->MAPR & AFIO_MAPR_USART2_REMAP) != 0U);
    DiagUart_WriteString(",cr1=");
    DiagUart_WriteUInt32(USART2->CR1);
    DiagUart_WriteString(",sr=");
    DiagUart_WriteUInt32(USART2->SR);
    DiagUart_WriteString(",nvic_rx_irq=");
    DiagUart_WriteUInt32((nvicWord & nvicMask) != 0U);
    DiagUart_WriteString("\r\n");
}

static void PwmSweep_TestGyroWire(void)
{
    GPIO_InitTypeDef gpio;
    volatile uint32_t settle;
    uint8_t lowLevel;
    uint8_t highLevel;

    /* PD5 is driven as a GPIO only for this isolated PD5--PD6 jumper test. */
    USART_Cmd(USART2, DISABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_5;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &gpio);

    GPIO_ResetBits(GPIOD, GPIO_Pin_5);
    for (settle = 0U; settle < 1000U; ++settle) { }
    lowLevel = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6);

    GPIO_SetBits(GPIOD, GPIO_Pin_5);
    for (settle = 0U; settle < 1000U; ++settle) { }
    highLevel = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6);

    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &gpio);
    USART_Cmd(USART2, ENABLE);

    DiagUart_WriteString("GYRO_WIRE_TEST,pd6_low=");
    DiagUart_WriteUInt32((uint32_t)lowLevel);
    DiagUart_WriteString(",pd6_high=");
    DiagUart_WriteUInt32((uint32_t)highLevel);
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
    const volatile GyroWitState_t *gyro = GyroWit_GetState();

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
        case 'E': PwmSweep_EnableDualPinEdgeMonitor(); break;
        case 'T': PwmSweep_TestGyroUartLoopback(); break;
        case 'W': PwmSweep_TestGyroWire(); break;
        case 'D': PwmSweep_PrintGyroUartDiagnostic(); break;
        case 'S': case '0': PwmSweep_Stop("COMMAND"); break;
        case 'P': PwmSweep_PrintStatus(); break;
        case 'H': case '?': PwmSweep_PrintHelp(); break;
        default: DiagUart_WriteString("PWM_CMD_UNKNOWN\r\n"); break;
    }
}

static void PwmSweep_LogGyro(void)
{
    const volatile GyroWitState_t *gyro = GyroWit_GetState();
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
    PwmSweep_InitGyroEdgeDiagnostic();
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
