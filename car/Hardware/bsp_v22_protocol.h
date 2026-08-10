#ifndef __BSP_V22_PROTOCOL_H
#define __BSP_V22_PROTOCOL_H

#include "stm32f10x.h"

#define V22_HEAD0              0xAAU
#define V22_HEAD1              0x55U
#define V22_VERSION            0x02U
#define V22_MAX_PAYLOAD        64U
#define V22_MAX_FRAME_BYTES    (11U + V22_MAX_PAYLOAD)
#define V22_STREAM_TIMEOUT_MS  100U

#define V22_ADDR_BROADCAST     0x10U
#define V22_ADDR_AIR_RADIO     0x20U
#define V22_ADDR_CAR_RADIO     0x30U
#define V22_ADDR_CAR_PI        0x31U
#define V22_ADDR_CAR_MCU       0x32U
#define V22_ADDR_GROUND        0x40U

#define V22_TYPE_FLIGHT_TELEMETRY 0x02U
#define V22_TYPE_HEARTBEAT       0x03U
#define V22_TYPE_ACK             0x11U
#define V22_TYPE_CAR_POSE        0x80U
#define V22_TYPE_TASK_REQUEST    0x81U
#define V22_TYPE_MISSION_STATUS  0x82U
#define V22_TYPE_CALIBRATION_SET 0x83U
#define V22_TYPE_MISSION_ABORT   0x84U
#define V22_TYPE_MAINTENANCE_RESET 0x85U

#define V22_FRAME_FLAG_ACK_REQUIRED   0x01U
#define V22_FRAME_FLAG_RETRANSMISSION 0x02U
#define V22_FRAME_FLAG_URGENT         0x04U

#define V22_ACK_RESULT_ACCEPTED       0x00U
#define V22_ACK_RESULT_DUPLICATE      0x01U
#define V22_ACK_RESULT_BUSY           0x02U
#define V22_ACK_RESULT_STATE_DENIED   0x03U
#define V22_ACK_RESULT_UNSUPPORTED    0x04U
#define V22_ACK_RESULT_PARAMETER      0x05U
#define V22_ACK_RESULT_INTERNAL       0x06U

#define V22_CALIBRATION_FLAG_APPLY            0x01U
#define V22_MAINT_RESET_FLAG_CLEAR_CALIBRATION 0x01U

/* MISSION_STATUS.Stage and FLIGHT_TELEMETRY.ModeCode share this mission
 * stage vocabulary.  The vehicle only uses these values for its bounded
 * speed profile; the aircraft remains the authority for flight actions. */
#define V22_MISSION_STAGE_IDLE             0U
#define V22_MISSION_STAGE_PRECHECK         1U
#define V22_MISSION_STAGE_TAKEOFF          2U
#define V22_MISSION_STAGE_INTERCEPT        3U
#define V22_MISSION_STAGE_FOLLOW           4U
#define V22_MISSION_STAGE_DROP_ALIGN       5U
#define V22_MISSION_STAGE_DROP_ACTION      6U
#define V22_MISSION_STAGE_LAND_ALIGN       7U
#define V22_MISSION_STAGE_DESCEND          8U
#define V22_MISSION_STAGE_ON_PLATFORM_5S   9U
#define V22_MISSION_STAGE_PLATFORM_TAKEOFF 10U
#define V22_MISSION_STAGE_RETURN_HOME      11U
#define V22_MISSION_STAGE_HOME_LAND        12U
#define V22_MISSION_STAGE_ABORT            13U

#define V22_COORDINATE_FIELD_GLOBAL 0x01U

#define V22_POSE_FLAG_POSITION_VALID 0x01U
#define V22_POSE_FLAG_CALIBRATED     0x02U
#define V22_POSE_FLAG_VELOCITY_VALID 0x04U
#define V22_POSE_FLAG_YAW_VALID      0x08U
#define V22_POSE_FLAG_CAR_RUNNING    0x10U

typedef struct
{
    uint8_t version;
    uint8_t type;
    uint8_t source;
    uint8_t destination;
    uint8_t sequence;
    uint8_t flags;
    uint8_t length;
    uint8_t payload[V22_MAX_PAYLOAD];
} V22Frame_t;

typedef enum
{
    V22_PARSE_NONE = 0,
    V22_PARSE_FRAME,
    V22_PARSE_VERSION_ERROR,
    V22_PARSE_LENGTH_ERROR,
    V22_PARSE_CRC_ERROR,
    V22_PARSE_TIMEOUT
} V22ParseResult_t;

typedef struct
{
    uint8_t buffer[V22_MAX_FRAME_BYTES];
    uint8_t index;
    uint8_t expectedLength;
    uint32_t lastByteMs;
} V22StreamParser_t;

uint16_t V22Protocol_Crc16CcittFalse(const uint8_t *data, uint16_t length);
uint16_t V22Protocol_Encode(uint8_t *buffer,
                            uint16_t capacity,
                            const V22Frame_t *frame);
void V22Protocol_ParserInit(V22StreamParser_t *parser);
V22ParseResult_t V22Protocol_ParserPush(V22StreamParser_t *parser,
                                        uint8_t byte,
                                        uint32_t nowMs,
                                        V22Frame_t *frame);
V22ParseResult_t V22Protocol_ParserCheckTimeout(V22StreamParser_t *parser,
                                                uint32_t nowMs);
uint8_t V22Protocol_DestinationAccepted(const V22Frame_t *frame,
                                        uint8_t localAddress);
const char *V22Protocol_ParseResultName(V22ParseResult_t result);

#endif /* __BSP_V22_PROTOCOL_H */
