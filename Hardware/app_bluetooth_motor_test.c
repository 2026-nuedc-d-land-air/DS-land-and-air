#include "app_bluetooth_motor_test.h"

#include "bsp_diag_uart.h"
#include "bsp_mecanum.h"
#include "bsp_robot_uart.h"

#define MOTOR_TEST_SPEED_PERCENT 18
#define MOTOR_TURN_SLOW_PERCENT  8
#define MOTOR_COMMAND_TIMEOUT_MS 800U
#define BLUETOOTH_SEND_ACK_ENABLE 0U

#ifndef BLUETOOTH_MOTOR_OUTPUT_ENABLE
#define BLUETOOTH_MOTOR_OUTPUT_ENABLE 0U
#endif

static uint32_t s_lastMotionCommandMs = 0U;
static uint32_t s_lastStatMs = 0U;
static uint8_t s_motionActive = 0U;
static uint8_t s_controlArmed = 0U;

volatile uint32_t g_bluetoothRxCount = 0U;
volatile uint32_t g_bluetoothRxLastMs = 0U;
volatile uint8_t g_bluetoothRxLastByte = 0U;
volatile uint8_t g_bluetoothRxHistory[BLUETOOTH_RX_HISTORY_CAPACITY] = {0U};
volatile uint8_t g_bluetoothRxHistoryCount = 0U;
volatile uint8_t g_bluetoothRxHistoryWriteIndex = 0U;

static void BluetoothMotorTest_RecordByte(uint8_t byte, uint32_t nowMs)
{
    g_bluetoothRxLastByte = byte;
    g_bluetoothRxLastMs = nowMs;
    ++g_bluetoothRxCount;

    g_bluetoothRxHistory[g_bluetoothRxHistoryWriteIndex] = byte;
    g_bluetoothRxHistoryWriteIndex =
        (uint8_t)((g_bluetoothRxHistoryWriteIndex + 1U) %
                  BLUETOOTH_RX_HISTORY_CAPACITY);
    if (g_bluetoothRxHistoryCount < BLUETOOTH_RX_HISTORY_CAPACITY)
    {
        ++g_bluetoothRxHistoryCount;
    }
}

static void BluetoothMotorTest_WriteHexByte(uint8_t byte)
{
    static const char digits[] = "0123456789ABCDEF";
    DiagUart_WriteChar(digits[(byte >> 4) & 0x0FU]);
    DiagUart_WriteChar(digits[byte & 0x0FU]);
}

static uint8_t BluetoothMotorTest_Normalize(uint8_t byte)
{
    if ((byte >= (uint8_t)'a') && (byte <= (uint8_t)'z'))
    {
        byte = (uint8_t)(byte - (uint8_t)'a' + (uint8_t)'A');
    }
    return byte;
}

static void BluetoothMotorTest_LogRaw(char source,
                                      uint8_t byte,
                                      uint32_t nowMs)
{
    DiagUart_WriteString("BT_RX,");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteChar(',');
    DiagUart_WriteChar(source);
    DiagUart_WriteString(",0x");
    BluetoothMotorTest_WriteHexByte(byte);
    DiagUart_WriteChar(',');
    if ((byte >= 32U) && (byte <= 126U))
    {
        DiagUart_WriteChar((char)byte);
    }
    else
    {
        DiagUart_WriteChar('.');
    }
    DiagUart_WriteString("\r\n");
}

static void BluetoothMotorTest_LogKey(uint32_t nowMs, uint8_t command)
{
    const char *name = "UNKNOWN";

    switch (command)
    {
        case 'G': case 'F': name = "UP"; break;
        case 'K': case 'B': name = "DOWN"; break;
        case 'H': case 'L': name = "LEFT"; break;
        case 'J': case 'R': name = "RIGHT"; break;
        case 'I': case 'S': case '0': name = "OK_STOP"; break;
        case 'Q': name = "ROTATE_LEFT"; break;
        case 'E': name = "ROTATE_RIGHT"; break;
        default: break;
    }

    DiagUart_WriteString("BT_KEY,");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",0x");
    BluetoothMotorTest_WriteHexByte(command);
    DiagUart_WriteChar(',');
    DiagUart_WriteChar((char)command);
    DiagUart_WriteChar(',');
    DiagUart_WriteString(name);
    DiagUart_WriteString("\r\n");
}

static void BluetoothMotorTest_LogMotor(uint32_t nowMs,
                                        uint8_t command,
                                        int16_t lf,
                                        int16_t rf,
                                        int16_t lr,
                                        int16_t rr)
{
    DiagUart_WriteString(BLUETOOTH_MOTOR_OUTPUT_ENABLE != 0U ?
                         "MOTOR_CMD," : "MOTOR_PREVIEW,");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteChar(',');
    DiagUart_WriteChar((char)command);
    DiagUart_WriteChar(',');
    DiagUart_WriteInt32(lf);
    DiagUart_WriteChar(',');
    DiagUart_WriteInt32(rf);
    DiagUart_WriteChar(',');
    DiagUart_WriteInt32(lr);
    DiagUart_WriteChar(',');
    DiagUart_WriteInt32(rr);
    DiagUart_WriteString("\r\n");
}

static void BluetoothMotorTest_SendAck(char source,
                                       uint8_t accepted,
                                       uint8_t command)
{
    if ((source != 'B') || (BLUETOOTH_SEND_ACK_ENABLE == 0U))
    {
        (void)accepted;
        (void)command;
        return;
    }

    RobotUart_BluetoothWriteString(accepted != 0U ? "ACK," : "UNKNOWN,");
    RobotUart_BluetoothWriteByte(command);
    RobotUart_BluetoothWriteString("\r\n");
}

static void BluetoothMotorTest_Stop(uint32_t nowMs, const char *reason)
{
    Mecanum_Enable(0U);
    s_motionActive = 0U;
    DiagUart_WriteString("MOTOR_STOP,");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteChar(',');
    DiagUart_WriteString(reason);
    DiagUart_WriteString("\r\n");
}

static void BluetoothMotorTest_SetMotion(uint32_t nowMs,
                                         uint8_t command,
                                         int16_t lf,
                                         int16_t rf,
                                         int16_t lr,
                                         int16_t rr)
{
    BluetoothMotorTest_LogMotor(nowMs, command, lf, rf, lr, rr);

    if (BLUETOOTH_MOTOR_OUTPUT_ENABLE != 0U)
    {
        Mecanum_Enable(1U);
        Mecanum_SetWheel(MECANUM_WHEEL_LF, lf);
        Mecanum_SetWheel(MECANUM_WHEEL_RF, rf);
        Mecanum_SetWheel(MECANUM_WHEEL_LR, lr);
        Mecanum_SetWheel(MECANUM_WHEEL_RR, rr);
        s_lastMotionCommandMs = nowMs;
        s_motionActive = 1U;
    }
    else
    {
        Mecanum_Enable(0U);
        s_motionActive = 0U;
    }
}

void BluetoothMotorTest_Init(uint32_t nowMs)
{
    uint8_t index;

    s_lastMotionCommandMs = nowMs;
    s_lastStatMs = nowMs;
    s_motionActive = 0U;
    s_controlArmed = 0U;
    g_bluetoothRxCount = 0U;
    g_bluetoothRxLastMs = 0U;
    g_bluetoothRxLastByte = 0U;
    g_bluetoothRxHistoryCount = 0U;
    g_bluetoothRxHistoryWriteIndex = 0U;
    for (index = 0U; index < BLUETOOTH_RX_HISTORY_CAPACITY; ++index)
    {
        g_bluetoothRxHistory[index] = 0U;
    }
    Mecanum_Enable(0U);

    DiagUart_WriteString(
        "BT_MOTOR_READY output_enabled=");
    DiagUart_WriteUInt32(BLUETOOTH_MOTOR_OUTPUT_ENABLE);
    DiagUart_WriteString(
        " speed=18 turn_slow=8 timeout_ms=800 armed=0 "
        "F=forward B=back L=left R=right Q=rotate_left "
        "E=rotate_right S/0=stop\r\n");
}

void BluetoothMotorTest_HandleByte(char source,
                                   uint8_t byte,
                                   uint32_t nowMs)
{
    uint8_t command;
    int16_t speed = MOTOR_TEST_SPEED_PERCENT;

    BluetoothMotorTest_RecordByte(byte, nowMs);

    if ((byte == (uint8_t)'\r') || (byte == (uint8_t)'\n') ||
        (byte == (uint8_t)' '))
    {
        return;
    }

    BluetoothMotorTest_LogRaw(source, byte, nowMs);
    command = BluetoothMotorTest_Normalize(byte);
    BluetoothMotorTest_LogKey(nowMs, command);

    if ((command == (uint8_t)'I') ||
        (command == (uint8_t)'S') ||
        (command == (uint8_t)'0'))
    {
        s_controlArmed = 1U;
        BluetoothMotorTest_Stop(nowMs, "COMMAND_ARMED");
        BluetoothMotorTest_SendAck(source, 1U, command);
        return;
    }

    if (s_controlArmed == 0U)
    {
        BluetoothMotorTest_Stop(nowMs, "DISARMED_PRESS_OK");
        BluetoothMotorTest_SendAck(source, 0U, command);
        return;
    }

    switch (command)
    {
        case 'G':
        case 'F':
            BluetoothMotorTest_SetMotion(nowMs, command,
                                         speed, speed, speed, speed);
            break;

        case 'K':
        case 'B':
            BluetoothMotorTest_SetMotion(nowMs, command,
                                         -speed, -speed, -speed, -speed);
            break;

        case 'H':
        case 'L':
            BluetoothMotorTest_SetMotion(nowMs, command,
                                         MOTOR_TURN_SLOW_PERCENT, speed,
                                         MOTOR_TURN_SLOW_PERCENT, speed);
            break;

        case 'J':
        case 'R':
            BluetoothMotorTest_SetMotion(nowMs, command,
                                         speed, MOTOR_TURN_SLOW_PERCENT,
                                         speed, MOTOR_TURN_SLOW_PERCENT);
            break;

        case 'Q':
            BluetoothMotorTest_SetMotion(nowMs, command,
                                         -speed, speed, -speed, speed);
            break;

        case 'E':
            BluetoothMotorTest_SetMotion(nowMs, command,
                                         speed, -speed, speed, -speed);
            break;

        default:
            BluetoothMotorTest_Stop(nowMs, "UNKNOWN_COMMAND");
            BluetoothMotorTest_SendAck(source, 0U, command);
            return;
    }

    BluetoothMotorTest_SendAck(source, 1U, command);
}

void BluetoothMotorTest_Update(uint32_t nowMs)
{
    if ((s_motionActive != 0U) &&
        ((uint32_t)(nowMs - s_lastMotionCommandMs) >= MOTOR_COMMAND_TIMEOUT_MS))
    {
        BluetoothMotorTest_Stop(nowMs, "TIMEOUT");
    }

    if ((uint32_t)(nowMs - s_lastStatMs) >= 1000U)
    {
        s_lastStatMs = nowMs;
        DiagUart_WriteString("BT_STAT,");
        DiagUart_WriteUInt32(nowMs);
        DiagUart_WriteString(",rx_count=");
        DiagUart_WriteUInt32(g_bluetoothRxCount);
        DiagUart_WriteString(",last_ms=");
        DiagUart_WriteUInt32(g_bluetoothRxLastMs);
        DiagUart_WriteString(",last_byte=0x");
        BluetoothMotorTest_WriteHexByte(g_bluetoothRxLastByte);
        DiagUart_WriteString(",history_count=");
        DiagUart_WriteUInt32(g_bluetoothRxHistoryCount);
        DiagUart_WriteString(",output_enabled=");
        DiagUart_WriteUInt32(BLUETOOTH_MOTOR_OUTPUT_ENABLE);
        DiagUart_WriteString(",armed=");
        DiagUart_WriteUInt32(s_controlArmed);
        DiagUart_WriteString("\r\n");
    }
}
