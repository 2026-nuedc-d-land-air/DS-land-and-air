/**
 ******************************************************************************
 * @file    main_bluetooth_car_arm.c
 * @brief   Bluetooth-controlled mecanum car and bus-servo action-group image.
 *
 * USART3 full remap: PD8=TX, PD9=RX, Bluetooth module, 9600 8N1.
 * UART4: PC10=TX, PC11=RX, bus-servo controller, 115200 8N1.
 *
 * Drive mode commands: I/S/0 arm+stop; F/G forward; B/K reverse; L/H left;
 * R/J right; Q rotate-left; E rotate-right.  P enters action mode and stops.
 * Action mode commands: 0/I initial; 1/G grab; 2/O open; 3/U lift;
 * 4/D lower; 5/K stack.  P/X returns to drive mode; S stops all servos.
 ******************************************************************************
 */

#include "app_bluetooth_motor_test.h"
#include "app_ladrc.h"
#include "app_servo_test.h"
#include "bsp_diag_uart.h"
#include "bsp_gyro_wit.h"
#include "bsp_mecanum.h"
#include "bsp_motor_safe.h"
#include "bsp_robot_uart.h"

#ifndef BLUETOOTH_BAUDRATE
#define BLUETOOTH_BAUDRATE 9600U
#endif

#define DIAG_BAUDRATE 115200U
#ifndef GYRO_BAUDRATE
#define GYRO_BAUDRATE 9600U
#endif
#define LADRC_TARGET_TENTHS          0
#define LADRC_KP_NUMERATOR           1
#define LADRC_KP_DENOMINATOR         10
#define LADRC_REPORT_INTERVAL_MS   100U
#define ACTION_SEQUENCE_TIMEOUT_MS 120U
#define ACTION_SEQUENCE_MAX_LENGTH   4U
#define ACTION_REPEAT_INTERVAL_MS      2500U
#define ACTION_REPEAT_ARM_WINDOW_MS     800U
#define ACTION_REPEAT_IDLE_TIMEOUT_MS   900U

static volatile uint32_t s_uptimeMs = 0U;
static uint8_t s_actionMode = 0U;
static uint8_t s_repeatActionActive = 0U;
static uint8_t s_repeatActionCommand = 0U;
static uint8_t s_lastActionCommand = 0U;
static uint32_t s_lastActionCommandMs = 0U;
static uint32_t s_lastRepeatTriggerMs = 0U;
static uint32_t s_nextRepeatActionMs = 0U;
static LadrcController_t s_yawLadrc;
static int16_t s_yawBaselineTenths = 0;
static uint8_t s_yawBaselineValid = 0U;
static uint8_t s_ladrcMotionActive = 0U;
static uint32_t s_lastLadrcReportMs = 0U;
static uint8_t s_actionSequence[ACTION_SEQUENCE_MAX_LENGTH];
static uint8_t s_actionSequenceLength = 0U;
static uint32_t s_actionSequenceLastMs = 0U;

typedef struct
{
    const char *wireSequence;
    uint8_t servoCommand;
} BluetoothActionSequence_t;

/* Lower-case phone button payloads, parsed atomically before drive commands. */
static const BluetoothActionSequence_t s_actionSequences[] =
{
    {"ZQ",   (uint8_t)'G'}, /* grab */
    {"FK",   (uint8_t)'O'}, /* open */
    {"JQL",  (uint8_t)'U'}, /* lift */
    {"FYD",  (uint8_t)'D'}, /* lower */
    {"DJ",   (uint8_t)'K'}, /* stack */
    {"HDCS", (uint8_t)'I'}  /* initial */
};

static void BluetoothCarArm_HandleStandardCommand(uint8_t command,
                                                   uint32_t nowMs);

void SysTick_Handler(void)
{
    ++s_uptimeMs;
}

static uint8_t BluetoothCarArm_RestoreSysTick(void)
{
    /* ServoTest uses short blocking frame gaps through Delay_ms(). */
    return (SysTick_Config(SystemCoreClock / 1000U) == 0U) ? 1U : 0U;
}

static uint8_t BluetoothCarArm_Normalize(uint8_t command)
{
    if ((command >= (uint8_t)'a') && (command <= (uint8_t)'z'))
    {
        command = (uint8_t)(command - (uint8_t)'a' + (uint8_t)'A');
    }
    return command;
}

static int16_t BluetoothCarArm_AngleDeltaTenths(int16_t current,
                                                int16_t baseline)
{
    int32_t delta = (int32_t)current - (int32_t)baseline;

    while (delta > 1800)
    {
        delta -= 3600;
    }
    while (delta < -1800)
    {
        delta += 3600;
    }
    return (int16_t)delta;
}

static void BluetoothCarArm_UpdateLadrc(uint32_t nowMs)
{
    const GyroWitState_t *gyro = GyroWit_GetState();
    int16_t yawDelta = 0;
    int16_t output;

    if (BluetoothMotorTest_IsMotionActive() == 0U)
    {
        s_ladrcMotionActive = 0U;
        return;
    }

    if (s_ladrcMotionActive == 0U)
    {
        s_ladrcMotionActive = 1U;
        s_yawBaselineTenths = gyro->yawTenthsDeg;
        s_yawBaselineValid = (gyro->angleFrameCount != 0U) ? 1U : 0U;
        Ladrc_Reset(&s_yawLadrc, LADRC_TARGET_TENTHS, 0);
        s_lastLadrcReportMs = nowMs;
        DiagUart_WriteString("BT_LADRC_START,command=");
        DiagUart_WriteChar((char)BluetoothMotorTest_GetActiveCommand());
        DiagUart_WriteString(",yaw_baseline_tenths=");
        DiagUart_WriteInt32(s_yawBaselineTenths);
        DiagUart_WriteString(",angle_frames=");
        DiagUart_WriteUInt32(gyro->angleFrameCount);
        DiagUart_WriteString("\r\n");
    }

    if ((uint32_t)(nowMs - s_lastLadrcReportMs) < LADRC_REPORT_INTERVAL_MS)
    {
        return;
    }

    s_lastLadrcReportMs = nowMs;
    if (s_yawBaselineValid != 0U)
    {
        yawDelta = BluetoothCarArm_AngleDeltaTenths(gyro->yawTenthsDeg,
                                                     s_yawBaselineTenths);
    }
    output = Ladrc_Update(&s_yawLadrc, LADRC_TARGET_TENTHS, yawDelta);

    DiagUart_WriteString("BT_LADRC,at_ms=");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",command=");
    DiagUart_WriteChar((char)BluetoothMotorTest_GetActiveCommand());
    DiagUart_WriteString(",target_tenths=");
    DiagUart_WriteInt32(s_yawLadrc.targetTenthsDeg);
    DiagUart_WriteString(",measured_tenths=");
    DiagUart_WriteInt32(s_yawLadrc.measuredTenthsDeg);
    DiagUart_WriteString(",estimate_tenths=");
    DiagUart_WriteInt32(s_yawLadrc.estimateTenthsDeg);
    DiagUart_WriteString(",error_tenths=");
    DiagUart_WriteInt32(s_yawLadrc.errorTenthsDeg);
    DiagUart_WriteString(",output_percent=");
    DiagUart_WriteInt32(output);
    DiagUart_WriteString(",angle_frames=");
    DiagUart_WriteUInt32(gyro->angleFrameCount);
    DiagUart_WriteString("\r\n");
}

static void BluetoothCarArm_SendBluetooth(const char *message)
{
    RobotUart_BluetoothWriteString(message);
}

static void BluetoothCarArm_StopDrive(uint32_t nowMs)
{
    /* Keep the existing motor-controller arming and timeout state coherent. */
    BluetoothMotorTest_HandleByte('B', (uint8_t)'S', nowMs);
}

static void BluetoothCarArm_PrintHelp(void)
{
    DiagUart_WriteString(
        "BT_COMBINED DRIVE:I/S/0=stop F/G=forward B/K=back L/H=left "
        "R/J=right Q/E=rotate P=actions | ACTION:0/I=initial 1/G=grab "
        "2/O=open 3/U=lift 4/D=lower 5/K=stack | SEQ:zq=grab fk=open "
        "jql=lift fyd=lower dj=stack hdcs=initial P/X=drive S=all_stop\r\n");
}

static void BluetoothCarArm_PrintActionParam(uint8_t command,
                                             const char *trigger)
{
    DiagUart_WriteString("BT_ACTION_PARAM,trigger=");
    DiagUart_WriteString(trigger);
    DiagUart_WriteString(",command=");
    DiagUart_WriteChar((char)command);
    DiagUart_WriteString(",name=");
    DiagUart_WriteString(ServoTest_GetActionName(command));
    DiagUart_WriteString(",move_ms=");
    DiagUart_WriteUInt32(ServoTest_GetMoveTimeMs());
    DiagUart_WriteString(",frame_gap_ms=");
    DiagUart_WriteUInt32(ServoTest_GetFrameGapMs());
    DiagUart_WriteString(",repeat_interval_ms=");
    DiagUart_WriteUInt32(ACTION_REPEAT_INTERVAL_MS);
    DiagUart_WriteString("\r\n");
}

static void BluetoothCarArm_ClearActionRepeat(const char *reason)
{
    if ((s_repeatActionActive != 0U) || (s_lastActionCommand != 0U))
    {
        DiagUart_WriteString("BT_ACTION_REPEAT_CLEAR,reason=");
        DiagUart_WriteString(reason);
        DiagUart_WriteString("\r\n");
    }

    s_repeatActionActive = 0U;
    s_repeatActionCommand = 0U;
    s_lastActionCommand = 0U;
    s_lastActionCommandMs = 0U;
    s_lastRepeatTriggerMs = 0U;
    s_nextRepeatActionMs = 0U;
}

static void BluetoothCarArm_EnterActionMode(uint32_t nowMs)
{
    BluetoothCarArm_ClearActionRepeat("ENTER_ACTION");
    BluetoothCarArm_StopDrive(nowMs);
    s_actionMode = 1U;
    DiagUart_WriteString("BT_MODE,ACTION\r\n");
    BluetoothCarArm_SendBluetooth("MODE,ACTION\r\n");
}

static void BluetoothCarArm_ExitActionMode(uint32_t nowMs, const char *reason)
{
    BluetoothCarArm_ClearActionRepeat(reason);
    BluetoothCarArm_StopDrive(nowMs);
    s_actionMode = 0U;
    DiagUart_WriteString("BT_MODE,DRIVE,");
    DiagUart_WriteString(reason);
    DiagUart_WriteString("\r\n");
    BluetoothCarArm_SendBluetooth("MODE,DRIVE\r\n");
}

static void BluetoothCarArm_RunAction(uint8_t command, uint32_t nowMs,
                                       const char *trigger)
{
    BluetoothCarArm_PrintActionParam(command, trigger);
    /* An action group is only entered after motors are positively stopped. */
    BluetoothCarArm_StopDrive(nowMs);
    ServoTest_HandleCommand('B', command);
    if (BluetoothCarArm_RestoreSysTick() == 0U)
    {
        MotorSafe_ForceOff();
        while (1)
        {
        }
    }
}

static uint8_t BluetoothCarArm_IsActionSequencePrefix(void)
{
    uint8_t sequenceIndex;
    uint8_t characterIndex;

    for (sequenceIndex = 0U;
         sequenceIndex < (uint8_t)(sizeof(s_actionSequences) /
                                   sizeof(s_actionSequences[0]));
         ++sequenceIndex)
    {
        for (characterIndex = 0U;
             characterIndex < s_actionSequenceLength;
             ++characterIndex)
        {
            if ((uint8_t)s_actionSequences[sequenceIndex].wireSequence[characterIndex] !=
                s_actionSequence[characterIndex])
            {
                break;
            }
        }

        if (characterIndex == s_actionSequenceLength)
        {
            return 1U;
        }
    }

    return 0U;
}

static const BluetoothActionSequence_t *BluetoothCarArm_FindActionSequence(void)
{
    uint8_t sequenceIndex;
    uint8_t characterIndex;
    const char *sequence;

    for (sequenceIndex = 0U;
         sequenceIndex < (uint8_t)(sizeof(s_actionSequences) /
                                   sizeof(s_actionSequences[0]));
         ++sequenceIndex)
    {
        sequence = s_actionSequences[sequenceIndex].wireSequence;
        for (characterIndex = 0U;
             characterIndex < s_actionSequenceLength;
             ++characterIndex)
        {
            if ((uint8_t)sequence[characterIndex] !=
                s_actionSequence[characterIndex])
            {
                break;
            }
        }

        if ((characterIndex == s_actionSequenceLength) &&
            (sequence[characterIndex] == '\0'))
        {
            return &s_actionSequences[sequenceIndex];
        }
    }

    return 0;
}

static void BluetoothCarArm_ClearActionSequence(void)
{
    s_actionSequenceLength = 0U;
    s_actionSequenceLastMs = 0U;
}

static void BluetoothCarArm_RunActionSequence(
    const BluetoothActionSequence_t *actionSequence, uint32_t nowMs)
{
    DiagUart_WriteString("BT_ACTION_SEQUENCE,payload=");
    DiagUart_WriteString(actionSequence->wireSequence);
    DiagUart_WriteString(",command=");
    DiagUart_WriteChar((char)actionSequence->servoCommand);
    DiagUart_WriteString(",name=");
    DiagUart_WriteString(ServoTest_GetActionName(actionSequence->servoCommand));
    DiagUart_WriteString("\r\n");

    BluetoothCarArm_ClearActionRepeat("SEQUENCE");
    BluetoothCarArm_RunAction(actionSequence->servoCommand, nowMs, "SEQUENCE");
}

static void BluetoothCarArm_FlushActionSequence(uint32_t nowMs)
{
    uint8_t pending[ACTION_SEQUENCE_MAX_LENGTH];
    uint8_t length = s_actionSequenceLength;
    uint8_t index;

    for (index = 0U; index < length; ++index)
    {
        pending[index] = s_actionSequence[index];
    }
    BluetoothCarArm_ClearActionSequence();

    for (index = 0U; index < length; ++index)
    {
        BluetoothCarArm_HandleStandardCommand(pending[index], nowMs);
    }
}

static void BluetoothCarArm_UpdateActionSequenceTimeout(uint32_t nowMs)
{
    if ((s_actionSequenceLength != 0U) &&
        ((uint32_t)(nowMs - s_actionSequenceLastMs) >=
         ACTION_SEQUENCE_TIMEOUT_MS))
    {
        BluetoothCarArm_FlushActionSequence(nowMs);
    }
}

static uint8_t BluetoothCarArm_RecordActionCommand(uint8_t command,
                                                   uint32_t nowMs)
{
    if ((s_lastActionCommand == command) &&
        ((uint32_t)(nowMs - s_lastActionCommandMs) <=
         ACTION_REPEAT_ARM_WINDOW_MS))
    {
        s_repeatActionActive = 1U;
        s_repeatActionCommand = command;
        s_lastRepeatTriggerMs = nowMs;
        if ((s_nextRepeatActionMs == 0U) ||
            ((int32_t)(nowMs - s_nextRepeatActionMs) >= 0))
        {
            s_nextRepeatActionMs = nowMs + ACTION_REPEAT_INTERVAL_MS;
        }
        DiagUart_WriteString("BT_ACTION_REPEAT_ARM,command=");
        DiagUart_WriteChar((char)command);
        DiagUart_WriteString(",name=");
        DiagUart_WriteString(ServoTest_GetActionName(command));
        DiagUart_WriteString("\r\n");
        s_lastActionCommandMs = nowMs;
        return 1U;
    }

    if (s_repeatActionActive != 0U)
    {
        BluetoothCarArm_ClearActionRepeat("NEW_ACTION");
    }

    s_lastActionCommand = command;
    s_lastActionCommandMs = nowMs;
    return 0U;
}

static void BluetoothCarArm_UpdateActionRepeat(uint32_t nowMs)
{
    if (s_repeatActionActive == 0U)
    {
        return;
    }

    if ((uint32_t)(nowMs - s_lastRepeatTriggerMs) >=
        ACTION_REPEAT_IDLE_TIMEOUT_MS)
    {
        BluetoothCarArm_ClearActionRepeat("IDLE_TIMEOUT");
        return;
    }

    if ((int32_t)(nowMs - s_nextRepeatActionMs) >= 0)
    {
        BluetoothCarArm_RunAction(s_repeatActionCommand, nowMs, "REPEAT");
        s_nextRepeatActionMs = nowMs + ACTION_REPEAT_INTERVAL_MS;
    }
}

static void BluetoothCarArm_HandleStandardCommand(uint8_t command,
                                                  uint32_t nowMs)
{
    if (s_actionMode == 0U)
    {
        if (command == (uint8_t)'P')
        {
            BluetoothCarArm_EnterActionMode(nowMs);
            return;
        }

        BluetoothMotorTest_HandleByte('B', command, nowMs);
        return;
    }

    if ((command == (uint8_t)'P') || (command == (uint8_t)'X'))
    {
        BluetoothCarArm_ExitActionMode(nowMs, "COMMAND");
    }
    else if (command == (uint8_t)'S')
    {
        BluetoothCarArm_ClearActionRepeat("ALL_STOP");
        ServoTest_StopAll();
        if (BluetoothCarArm_RestoreSysTick() == 0U)
        {
            MotorSafe_ForceOff();
            while (1)
            {
            }
        }
        BluetoothCarArm_ExitActionMode(nowMs, "ALL_STOP");
    }
    else if (ServoTest_IsActionCommand(command) != 0U)
    {
        if (BluetoothCarArm_RecordActionCommand(command, nowMs) == 0U)
        {
            BluetoothCarArm_RunAction(command, nowMs, "PRESS");
        }
    }
    else if ((command == (uint8_t)'?') || (command == (uint8_t)'H'))
    {
        BluetoothCarArm_PrintHelp();
        BluetoothCarArm_SendBluetooth("HELP,ACTION,0-5\r\n");
    }
    else
    {
        DiagUart_WriteString("BT_ACTION_UNKNOWN\r\n");
        BluetoothCarArm_SendBluetooth("ERR,ACTION\r\n");
    }
}

static void BluetoothCarArm_HandleBluetoothByte(uint8_t byte, uint32_t nowMs)
{
    const BluetoothActionSequence_t *actionSequence;
    uint8_t command;

    if ((byte == (uint8_t)'\r') || (byte == (uint8_t)'\n') ||
        (byte == (uint8_t)' '))
    {
        return;
    }

    BluetoothCarArm_UpdateActionSequenceTimeout(nowMs);

    /*
     * Keep the phone's lower-case action payloads separate from the existing
     * upper-case motor buttons.  In particular, motor F followed by K must
     * never be interpreted as the lower-case "fk" open action.
     */
    if ((byte < (uint8_t)'a') || (byte > (uint8_t)'z'))
    {
        if (s_actionSequenceLength != 0U)
        {
            BluetoothCarArm_FlushActionSequence(nowMs);
        }
        BluetoothCarArm_HandleStandardCommand(BluetoothCarArm_Normalize(byte),
                                              nowMs);
        return;
    }

    command = BluetoothCarArm_Normalize(byte);

    if (s_actionSequenceLength >= ACTION_SEQUENCE_MAX_LENGTH)
    {
        BluetoothCarArm_FlushActionSequence(nowMs);
    }

    s_actionSequence[s_actionSequenceLength++] = command;
    s_actionSequenceLastMs = nowMs;

    if (BluetoothCarArm_IsActionSequencePrefix() == 0U)
    {
        /* Not one of the phone's multi-byte action payloads: replay normally. */
        BluetoothCarArm_FlushActionSequence(nowMs);
        return;
    }

    actionSequence = BluetoothCarArm_FindActionSequence();
    if (actionSequence != 0)
    {
        BluetoothCarArm_ClearActionSequence();
        BluetoothCarArm_RunActionSequence(actionSequence, nowMs);
    }
}

int main(void)
{
    uint8_t bluetoothByte;

    /* Clamp every motor pin before UART, PWM, or action initialization. */
    MotorSafe_InitOff();
    SystemCoreClockUpdate();
    Mecanum_InitOff();
    DiagUart_Init(DIAG_BAUDRATE);
    RobotUart_BluetoothInit(BLUETOOTH_BAUDRATE);
    RobotUart_ServoInit(115200U);
    GyroWit_Init(GYRO_BAUDRATE);

    if (SysTick_Config(SystemCoreClock / 1000U) != 0U)
    {
        MotorSafe_ForceOff();
        while (1)
        {
        }
    }

    BluetoothMotorTest_Init(s_uptimeMs);
    Ladrc_Init(&s_yawLadrc, LADRC_KP_NUMERATOR, LADRC_KP_DENOMINATOR);
    Ladrc_Reset(&s_yawLadrc, LADRC_TARGET_TENTHS, 0);
    ServoTest_Init();
    DiagUart_WriteString("BOOT,bluetooth_car_arm,bt=USART3_PD8_PD9,servo=UART4_PC10_PC11,gyro=USART2_PD5_PD6\r\n");
    BluetoothCarArm_PrintHelp();

    /* Discard Bluetooth bytes emitted while its module powers up. */
    while (RobotUart_BluetoothTryReadByte(&bluetoothByte) != 0U)
    {
    }

    while (1)
    {
        while (RobotUart_BluetoothTryReadByte(&bluetoothByte) != 0U)
        {
            BluetoothCarArm_HandleBluetoothByte(bluetoothByte, s_uptimeMs);
        }

        BluetoothMotorTest_Update(s_uptimeMs);
        BluetoothCarArm_UpdateActionSequenceTimeout(s_uptimeMs);
        BluetoothCarArm_UpdateActionRepeat(s_uptimeMs);
        Mecanum_Update(s_uptimeMs);
        GyroWit_Poll();
        BluetoothCarArm_UpdateLadrc(s_uptimeMs);
        ServoTest_PollServoResponse();
    }
}
