#include "app_servo_test.h"

#include "bsp_diag_uart.h"
#include "bsp_robot_uart.h"
#include "Delay.h"

#define SERVO_COUNT              6U
#define SERVO_MOVE_TIME_MS       1500U
#define SERVO_FRAME_GAP_MS       3U
#define SERVO_RESPONSE_CAPACITY  64U

typedef enum
{
    SERVO_POSE_INITIAL = 0,
    SERVO_POSE_GRAB,
    SERVO_POSE_OPEN,
    SERVO_POSE_LIFT,
    SERVO_POSE_LOWER,
    SERVO_POSE_STACK,
    SERVO_POSE_COUNT
} ServoPoseIndex_t;

typedef struct
{
    const char *name;
    uint16_t position[SERVO_COUNT];
} ServoPose_t;

static const ServoPose_t s_poses[SERVO_POSE_COUNT] =
{
    {"INITIAL", {1700U, 1800U,  986U, 1600U, 1178U, 1400U}},
    {"GRAB",    {1550U, 1450U, 1020U, 1450U,  846U, 1485U}},
    {"OPEN",    {1650U, 1450U, 1020U, 1550U,  846U, 1485U}},
    {"LIFT",    {1520U, 1720U, 1020U, 1400U, 1100U, 1485U}},
    {"LOWER",   {1520U, 1700U, 1020U, 1400U, 1100U, 1485U}},
    {"STACK",   {1560U, 1700U, 1020U, 1450U, 1100U, 1485U}}
};

static uint32_t s_logSequence = 0U;
static char s_servoResponse[SERVO_RESPONSE_CAPACITY];
static uint8_t s_servoResponseLength = 0U;

static void ServoTest_LogSequence(void)
{
    DiagUart_WriteUInt32(++s_logSequence);
}

static void ServoTest_WriteHexByte(uint8_t byte)
{
    static const char digits[] = "0123456789ABCDEF";
    DiagUart_WriteChar(digits[(byte >> 4) & 0x0FU]);
    DiagUart_WriteChar(digits[byte & 0x0FU]);
}

static void ServoTest_Put3(char *output, uint16_t value)
{
    output[0] = (char)('0' + ((value / 100U) % 10U));
    output[1] = (char)('0' + ((value / 10U) % 10U));
    output[2] = (char)('0' + (value % 10U));
}

static void ServoTest_Put4(char *output, uint16_t value)
{
    output[0] = (char)('0' + ((value / 1000U) % 10U));
    output[1] = (char)('0' + ((value / 100U) % 10U));
    output[2] = (char)('0' + ((value / 10U) % 10U));
    output[3] = (char)('0' + (value % 10U));
}

static void ServoTest_BuildMoveFrame(uint8_t id,
                                     uint16_t position,
                                     uint16_t timeMs,
                                     char frame[16])
{
    frame[0] = '#';
    ServoTest_Put3(&frame[1], id);
    frame[4] = 'P';
    ServoTest_Put4(&frame[5], position);
    frame[9] = 'T';
    ServoTest_Put4(&frame[10], timeMs);
    frame[14] = '!';
    frame[15] = '\0';
}

static void ServoTest_BuildSimpleFrame(uint8_t id,
                                       const char command[4],
                                       char frame[10])
{
    frame[0] = '#';
    ServoTest_Put3(&frame[1], id);
    frame[4] = 'P';
    frame[5] = command[0];
    frame[6] = command[1];
    frame[7] = command[2];
    frame[8] = '!';
    frame[9] = '\0';
}

static void ServoTest_LogTx(const char *poseName,
                            uint8_t id,
                            uint16_t position,
                            uint16_t timeMs,
                            const char *frame)
{
    DiagUart_WriteString("SERVO_TX,");
    ServoTest_LogSequence();
    DiagUart_WriteChar(',');
    DiagUart_WriteString(poseName);
    DiagUart_WriteChar(',');
    DiagUart_WriteUInt32(id);
    DiagUart_WriteChar(',');
    DiagUart_WriteUInt32(position);
    DiagUart_WriteChar(',');
    DiagUart_WriteUInt32(timeMs);
    DiagUart_WriteChar(',');
    DiagUart_WriteString(frame);
    DiagUart_WriteString("\r\n");
}

static void ServoTest_SendPose(ServoPoseIndex_t poseIndex)
{
    uint8_t id;
    char frame[16];
    const ServoPose_t *pose;

    if (poseIndex >= SERVO_POSE_COUNT)
    {
        return;
    }

    pose = &s_poses[poseIndex];
    DiagUart_WriteString("POSE_BEGIN,");
    ServoTest_LogSequence();
    DiagUart_WriteChar(',');
    DiagUart_WriteString(pose->name);
    DiagUart_WriteString("\r\n");

    for (id = 0U; id < SERVO_COUNT; ++id)
    {
        ServoTest_BuildMoveFrame(id, pose->position[id],
                                 SERVO_MOVE_TIME_MS, frame);
        ServoTest_LogTx(pose->name, id, pose->position[id],
                        SERVO_MOVE_TIME_MS, frame);
        RobotUart_ServoWriteString(frame);
        Delay_ms(SERVO_FRAME_GAP_MS);
    }

    DiagUart_WriteString("POSE_END,");
    ServoTest_LogSequence();
    DiagUart_WriteChar(',');
    DiagUart_WriteString(pose->name);
    DiagUart_WriteString("\r\n");
}

static void ServoTest_SendSimpleToAll(const char *name,
                                      const char command[4])
{
    uint8_t id;
    char frame[10];

    for (id = 0U; id < SERVO_COUNT; ++id)
    {
        ServoTest_BuildSimpleFrame(id, command, frame);
        DiagUart_WriteString("SERVO_TX,");
        ServoTest_LogSequence();
        DiagUart_WriteChar(',');
        DiagUart_WriteString(name);
        DiagUart_WriteChar(',');
        DiagUart_WriteUInt32(id);
        DiagUart_WriteString(",NA,NA,");
        DiagUart_WriteString(frame);
        DiagUart_WriteString("\r\n");
        RobotUart_ServoWriteString(frame);
        Delay_ms(SERVO_FRAME_GAP_MS);
    }
}

static uint8_t ServoTest_NormalizeCommand(uint8_t command)
{
    if ((command >= (uint8_t)'a') && (command <= (uint8_t)'z'))
    {
        command = (uint8_t)(command - (uint8_t)'a' + (uint8_t)'A');
    }
    return command;
}

static void ServoTest_SendBluetoothAck(uint8_t command, uint8_t accepted)
{
    RobotUart_BluetoothWriteString(accepted != 0U ? "ACK," : "ERR,");
    RobotUart_BluetoothWriteByte(command);
    RobotUart_BluetoothWriteString("\r\n");
}

static void ServoTest_PrintHelp(void)
{
    DiagUart_WriteString(
        "CMD_HELP I/0=initial G/1=grab O/2=open U/3=lift "
        "D/4=lower K/5=stack S=stop R=release Q=query H=help\r\n");
}

void ServoTest_Init(void)
{
    DiagUart_WriteString("servo_test.ready automatic_motion=0 move_ms=");
    DiagUart_WriteUInt32(SERVO_MOVE_TIME_MS);
    DiagUart_WriteString(" ids=0..5\r\n");
    ServoTest_PrintHelp();
}

void ServoTest_StopAll(void)
{
    ServoTest_SendSimpleToAll("STOP", "DST");
}

void ServoTest_HandleCommand(char source, uint8_t command)
{
    uint8_t accepted = 1U;

    if ((command == (uint8_t)'\r') || (command == (uint8_t)'\n') ||
        (command == (uint8_t)' '))
    {
        return;
    }

    command = ServoTest_NormalizeCommand(command);

    DiagUart_WriteString("CMD_RX,");
    ServoTest_LogSequence();
    DiagUart_WriteChar(',');
    DiagUart_WriteChar(source);
    DiagUart_WriteString(",0x");
    ServoTest_WriteHexByte(command);
    DiagUart_WriteChar(',');
    DiagUart_WriteChar((char)command);
    DiagUart_WriteString("\r\n");

    switch (command)
    {
        case 'I': case '0': ServoTest_SendPose(SERVO_POSE_INITIAL); break;
        case 'G': case '1': ServoTest_SendPose(SERVO_POSE_GRAB); break;
        case 'O': case '2': ServoTest_SendPose(SERVO_POSE_OPEN); break;
        case 'U': case '3': ServoTest_SendPose(SERVO_POSE_LIFT); break;
        case 'D': case '4': ServoTest_SendPose(SERVO_POSE_LOWER); break;
        case 'K': case '5': ServoTest_SendPose(SERVO_POSE_STACK); break;
        case 'S': ServoTest_SendSimpleToAll("STOP", "DST"); break;
        case 'R': ServoTest_SendSimpleToAll("RELEASE", "ULK"); break;
        case 'Q': ServoTest_SendSimpleToAll("QUERY_ANGLE", "RAD"); break;
        case 'H': case '?': ServoTest_PrintHelp(); break;
        default:
            accepted = 0U;
            DiagUart_WriteString("CMD_UNKNOWN,");
            ServoTest_LogSequence();
            DiagUart_WriteString(",0x");
            ServoTest_WriteHexByte(command);
            DiagUart_WriteString("\r\n");
            break;
    }

    if (source == 'B')
    {
        ServoTest_SendBluetoothAck(command, accepted);
    }
}

void ServoTest_PollServoResponse(void)
{
    uint8_t byte;

    while (RobotUart_ServoTryReadByte(&byte) != 0U)
    {
        if ((byte == (uint8_t)'\r') || (byte == (uint8_t)'\n'))
        {
            if (s_servoResponseLength == 0U)
            {
                continue;
            }
        }
        else if (s_servoResponseLength < (SERVO_RESPONSE_CAPACITY - 1U))
        {
            s_servoResponse[s_servoResponseLength++] = (char)byte;
        }

        if ((byte == (uint8_t)'!') ||
            (byte == (uint8_t)'\r') ||
            (byte == (uint8_t)'\n') ||
            (s_servoResponseLength >= (SERVO_RESPONSE_CAPACITY - 1U)))
        {
            s_servoResponse[s_servoResponseLength] = '\0';
            DiagUart_WriteString("SERVO_RX,");
            ServoTest_LogSequence();
            DiagUart_WriteChar(',');
            DiagUart_WriteString(s_servoResponse);
            DiagUart_WriteString("\r\n");
            s_servoResponseLength = 0U;
        }
    }
}

void ServoTest_LogNanoByte(uint8_t byte)
{
    DiagUart_WriteString("NANO_RX,");
    ServoTest_LogSequence();
    DiagUart_WriteString(",0x");
    ServoTest_WriteHexByte(byte);
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
