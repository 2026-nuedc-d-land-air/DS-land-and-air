#include "app_lora_protocol_test.h"

#include "bsp_diag_uart.h"
#include "bsp_robot_uart.h"
#include "bsp_v22_protocol.h"

#define LORA_PROTOCOL_STAT_PERIOD_MS 1000U

#ifndef LORA_UART_BAUDRATE
#define LORA_UART_BAUDRATE 115200U
#endif

/* Exact V2.2 document vectors. They are bench fixtures, not live pose data. */
static const uint8_t s_documentCarPoseVector[] =
{
    0xAAU, 0x55U, 0x02U, 0x80U, 0x30U, 0x10U, 0x42U, 0x00U, 0x16U,
    0x01U, 0x0FU, 0x00U, 0x01U, 0x00U, 0x00U, 0x00U, 0x78U,
    0xFFU, 0xFFU, 0xFFU, 0xCEU, 0x03U, 0x84U, 0x00U, 0x0AU,
    0x00U, 0x00U, 0x00U, 0x00U, 0x04U, 0xD2U, 0xB2U, 0x2DU
};

static const uint8_t s_documentCalibrationVector[] =
{
    0xAAU, 0x55U, 0x02U, 0x83U, 0x40U, 0x30U, 0x16U, 0x01U, 0x0CU,
    0x00U, 0x00U, 0x00U, 0x64U, 0xFFU, 0xFFU, 0xFFU, 0xCEU,
    0x00U, 0x01U, 0x01U, 0x00U, 0xF1U, 0x24U
};

static V22StreamParser_t s_parser;
static uint32_t s_rxBytes;
static uint32_t s_rxFrames;
static uint32_t s_rxAccepted;
static uint32_t s_rxIgnoredDestination;
static uint32_t s_rxVersionErrors;
static uint32_t s_rxLengthErrors;
static uint32_t s_rxCrcErrors;
static uint32_t s_rxTimeouts;
static uint32_t s_txFrames;
static uint32_t s_uartErrorFlags;
static uint32_t s_lastStatsMs;
static uint8_t s_nextTxSequence;
static uint8_t s_selfTestPassed;

static void LoraProtocolTest_WriteFrameFields(const V22Frame_t *frame)
{
    DiagUart_WriteString(",type=");
    DiagUart_WriteUInt32(frame->type);
    DiagUart_WriteString(",src=");
    DiagUart_WriteUInt32(frame->source);
    DiagUart_WriteString(",dst=");
    DiagUart_WriteUInt32(frame->destination);
    DiagUart_WriteString(",seq=");
    DiagUart_WriteUInt32(frame->sequence);
    DiagUart_WriteString(",flags=");
    DiagUart_WriteUInt32(frame->flags);
    DiagUart_WriteString(",len=");
    DiagUart_WriteUInt32(frame->length);
}

static void LoraProtocolTest_LogParseResult(V22ParseResult_t result,
                                            uint32_t nowMs)
{
    if (result == V22_PARSE_NONE)
    {
        return;
    }

    DiagUart_WriteString("LORA,event=rx_drop,t_ms=");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",reason=");
    DiagUart_WriteString(V22Protocol_ParseResultName(result));
    DiagUart_WriteString(",motors_safe=1\r\n");

    if (result == V22_PARSE_VERSION_ERROR)
    {
        ++s_rxVersionErrors;
    }
    else if (result == V22_PARSE_LENGTH_ERROR)
    {
        ++s_rxLengthErrors;
    }
    else if (result == V22_PARSE_CRC_ERROR)
    {
        ++s_rxCrcErrors;
    }
    else if (result == V22_PARSE_TIMEOUT)
    {
        ++s_rxTimeouts;
    }
}

static uint8_t LoraProtocolTest_ParseDocumentVector(const uint8_t *vector,
                                                    uint8_t vectorLength,
                                                    uint8_t expectedType,
                                                    uint8_t expectedSource,
                                                    uint8_t expectedDestination,
                                                    uint8_t expectedSequence,
                                                    uint8_t expectedLength)
{
    V22StreamParser_t parser;
    V22Frame_t frame;
    V22ParseResult_t result = V22_PARSE_NONE;
    uint8_t index;

    V22Protocol_ParserInit(&parser);
    for (index = 0U; index < vectorLength; ++index)
    {
        result = V22Protocol_ParserPush(&parser,
                                        vector[index],
                                        (uint32_t)(1000U + index),
                                        &frame);
    }

    if (result != V22_PARSE_FRAME)
    {
        return 0U;
    }
    return ((frame.type == expectedType) &&
            (frame.source == expectedSource) &&
            (frame.destination == expectedDestination) &&
            (frame.sequence == expectedSequence) &&
            (frame.length == expectedLength)) ? 1U : 0U;
}

static uint8_t LoraProtocolTest_EncodeMatchesDocumentPose(void)
{
    V22Frame_t frame;
    uint8_t encoded[V22_MAX_FRAME_BYTES];
    uint16_t encodedLength;
    uint8_t index;

    frame.version = V22_VERSION;
    frame.type = V22_TYPE_CAR_POSE;
    frame.source = V22_ADDR_CAR_RADIO;
    frame.destination = V22_ADDR_BROADCAST;
    frame.sequence = 0x42U;
    frame.flags = 0U;
    frame.length = 22U;
    for (index = 0U; index < frame.length; ++index)
    {
        frame.payload[index] = s_documentCarPoseVector[(uint16_t)9U + index];
    }

    encodedLength = V22Protocol_Encode(encoded, sizeof(encoded), &frame);
    if (encodedLength != sizeof(s_documentCarPoseVector))
    {
        return 0U;
    }
    for (index = 0U; index < encodedLength; ++index)
    {
        if (encoded[index] != s_documentCarPoseVector[index])
        {
            return 0U;
        }
    }
    return 1U;
}

static void LoraProtocolTest_SendHeartbeat(uint32_t nowMs)
{
    V22Frame_t frame;
    uint8_t encoded[V22_MAX_FRAME_BYTES];
    uint16_t length;

    frame.version = V22_VERSION;
    frame.type = V22_TYPE_HEARTBEAT;
    frame.source = V22_ADDR_CAR_RADIO;
    frame.destination = V22_ADDR_BROADCAST;
    frame.sequence = s_nextTxSequence++;
    frame.flags = 0U;
    frame.length = 0U;
    length = V22Protocol_Encode(encoded, sizeof(encoded), &frame);
    if (length == 0U)
    {
        DiagUart_WriteString("LORA,event=tx_drop,t_ms=");
        DiagUart_WriteUInt32(nowMs);
        DiagUart_WriteString(",reason=ENCODE,motors_safe=1\r\n");
        return;
    }

    RobotUart_NanoWriteBuffer(encoded, length);
    ++s_txFrames;
    DiagUart_WriteString("LORA,event=tx_heartbeat,t_ms=");
    DiagUart_WriteUInt32(nowMs);
    LoraProtocolTest_WriteFrameFields(&frame);
    DiagUart_WriteString(",bytes=");
    DiagUart_WriteUInt32(length);
    DiagUart_WriteString(",bench_only=1,motors_safe=1\r\n");
}

static void LoraProtocolTest_SendDocumentPose(uint32_t nowMs)
{
    if (s_selfTestPassed == 0U)
    {
        DiagUart_WriteString("LORA,event=tx_drop,t_ms=");
        DiagUart_WriteUInt32(nowMs);
        DiagUart_WriteString(",reason=SELFTEST,motors_safe=1\r\n");
        return;
    }

    RobotUart_NanoWriteBuffer(s_documentCarPoseVector,
                              sizeof(s_documentCarPoseVector));
    ++s_txFrames;
    DiagUart_WriteString("LORA,event=tx_pose_vector,t_ms=");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",type=128,src=48,dst=16,seq=66,flags=0,len=22,bytes=");
    DiagUart_WriteUInt32(sizeof(s_documentCarPoseVector));
    DiagUart_WriteString(",bench_only=1,motors_safe=1\r\n");
}

static void LoraProtocolTest_WriteStats(uint32_t nowMs)
{
    DiagUart_WriteString("LORA,event=stat,t_ms=");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",rx_bytes=");
    DiagUart_WriteUInt32(s_rxBytes);
    DiagUart_WriteString(",rx_frames=");
    DiagUart_WriteUInt32(s_rxFrames);
    DiagUart_WriteString(",rx_accepted=");
    DiagUart_WriteUInt32(s_rxAccepted);
    DiagUart_WriteString(",rx_ignored_dst=");
    DiagUart_WriteUInt32(s_rxIgnoredDestination);
    DiagUart_WriteString(",crc_err=");
    DiagUart_WriteUInt32(s_rxCrcErrors);
    DiagUart_WriteString(",len_err=");
    DiagUart_WriteUInt32(s_rxLengthErrors);
    DiagUart_WriteString(",ver_err=");
    DiagUart_WriteUInt32(s_rxVersionErrors);
    DiagUart_WriteString(",timeout=");
    DiagUart_WriteUInt32(s_rxTimeouts);
    DiagUart_WriteString(",tx_frames=");
    DiagUart_WriteUInt32(s_txFrames);
    DiagUart_WriteString(",uart_flags=");
    DiagUart_WriteUInt32(s_uartErrorFlags);
    DiagUart_WriteString(",motors_safe=1\r\n");
}

void LoraProtocolTest_Init(uint32_t nowMs)
{
    uint8_t crcOk;
    uint8_t poseOk;
    uint8_t calibrationOk;
    uint8_t encodeOk;

    V22Protocol_ParserInit(&s_parser);
    s_rxBytes = 0U;
    s_rxFrames = 0U;
    s_rxAccepted = 0U;
    s_rxIgnoredDestination = 0U;
    s_rxVersionErrors = 0U;
    s_rxLengthErrors = 0U;
    s_rxCrcErrors = 0U;
    s_rxTimeouts = 0U;
    s_txFrames = 0U;
    s_uartErrorFlags = 0U;
    s_lastStatsMs = nowMs;
    s_nextTxSequence = 0U;

    crcOk = (V22Protocol_Crc16CcittFalse((const uint8_t *)"123456789", 9U) ==
             0x29B1U) ? 1U : 0U;
    poseOk = LoraProtocolTest_ParseDocumentVector(
        s_documentCarPoseVector, sizeof(s_documentCarPoseVector),
        V22_TYPE_CAR_POSE, V22_ADDR_CAR_RADIO, V22_ADDR_BROADCAST, 0x42U, 22U);
    calibrationOk = LoraProtocolTest_ParseDocumentVector(
        s_documentCalibrationVector, sizeof(s_documentCalibrationVector),
        V22_TYPE_CALIBRATION_SET, 0x40U, V22_ADDR_CAR_RADIO, 0x16U, 12U);
    encodeOk = LoraProtocolTest_EncodeMatchesDocumentPose();
    s_selfTestPassed = ((crcOk != 0U) && (poseOk != 0U) &&
                        (calibrationOk != 0U) && (encodeOk != 0U)) ? 1U : 0U;

    DiagUart_WriteString("LORA,event=boot,t_ms=");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",uart=UART5,tx=PC12,rx=PD2,baud=");
    DiagUart_WriteUInt32(LORA_UART_BAUDRATE);
    DiagUart_WriteString(",role=candidate,auto_tx=0,motors_safe=1\r\n");
    DiagUart_WriteString("LORA,event=selftest,crc=");
    DiagUart_WriteString((crcOk != 0U) ? "PASS" : "FAIL");
    DiagUart_WriteString(",pose_parse=");
    DiagUart_WriteString((poseOk != 0U) ? "PASS" : "FAIL");
    DiagUart_WriteString(",cal_parse=");
    DiagUart_WriteString((calibrationOk != 0U) ? "PASS" : "FAIL");
    DiagUart_WriteString(",encode=");
    DiagUart_WriteString((encodeOk != 0U) ? "PASS" : "FAIL");
    DiagUart_WriteString(",overall=");
    DiagUart_WriteString((s_selfTestPassed != 0U) ? "PASS" : "FAIL");
    DiagUart_WriteString("\r\n");
    DiagUart_WriteString("LORA,commands=P:document_pose_vector,B:bench_heartbeat,S:stats,H:help; no_motion_or_ack\r\n");
}

void LoraProtocolTest_HandleRadioByte(uint8_t byte, uint32_t nowMs)
{
    V22Frame_t frame;
    V22ParseResult_t result;
    uint8_t accepted;

    ++s_rxBytes;
    result = V22Protocol_ParserPush(&s_parser, byte, nowMs, &frame);
    if (result == V22_PARSE_FRAME)
    {
        ++s_rxFrames;
        accepted = V22Protocol_DestinationAccepted(&frame, V22_ADDR_CAR_RADIO);
        if (accepted != 0U)
        {
            ++s_rxAccepted;
            DiagUart_WriteString("LORA,event=rx_ok,t_ms=");
        }
        else
        {
            ++s_rxIgnoredDestination;
            DiagUart_WriteString("LORA,event=rx_ignored,t_ms=");
        }
        DiagUart_WriteUInt32(nowMs);
        LoraProtocolTest_WriteFrameFields(&frame);
        DiagUart_WriteString(",accepted=");
        DiagUart_WriteUInt32(accepted);
        DiagUart_WriteString(",state_changed=0,motors_safe=1\r\n");
        return;
    }

    LoraProtocolTest_LogParseResult(result, nowMs);
}

void LoraProtocolTest_HandleUartErrors(uint16_t flags, uint32_t nowMs)
{
    if (flags == 0U)
    {
        return;
    }

    s_uartErrorFlags |= flags;
    DiagUart_WriteString("LORA,event=uart_error,t_ms=");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",flags=");
    DiagUart_WriteUInt32(flags);
    DiagUart_WriteString(",motors_safe=1\r\n");
}

void LoraProtocolTest_HandleCommand(char command, uint32_t nowMs)
{
    if ((command == 'P') || (command == 'p'))
    {
        LoraProtocolTest_SendDocumentPose(nowMs);
        return;
    }
    if ((command == 'B') || (command == 'b'))
    {
        LoraProtocolTest_SendHeartbeat(nowMs);
        return;
    }
    if ((command == 'S') || (command == 's'))
    {
        LoraProtocolTest_WriteStats(nowMs);
        return;
    }

    DiagUart_WriteString("LORA,event=help,t_ms=");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",commands=P:document_pose_vector,B:bench_heartbeat,S:stats; motors_safe=1\r\n");
}

void LoraProtocolTest_Update(uint32_t nowMs)
{
    LoraProtocolTest_LogParseResult(
        V22Protocol_ParserCheckTimeout(&s_parser, nowMs), nowMs);

    if ((uint32_t)(nowMs - s_lastStatsMs) >= LORA_PROTOCOL_STAT_PERIOD_MS)
    {
        s_lastStatsMs = nowMs;
        LoraProtocolTest_WriteStats(nowMs);
    }
}
