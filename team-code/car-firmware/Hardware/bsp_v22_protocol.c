#include "bsp_v22_protocol.h"

static void V22Protocol_ResetOrRestart(V22StreamParser_t *parser, uint8_t byte)
{
    parser->index = 0U;
    parser->expectedLength = 0U;
    if (byte == V22_HEAD0)
    {
        parser->buffer[0] = byte;
        parser->index = 1U;
    }
}

uint16_t V22Protocol_Crc16CcittFalse(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint8_t bit;

    if (data == 0)
    {
        return 0U;
    }

    while (length-- != 0U)
    {
        crc ^= (uint16_t)(*data++) << 8;
        for (bit = 0U; bit < 8U; ++bit)
        {
            crc = ((crc & 0x8000U) != 0U) ?
                  (uint16_t)((crc << 1) ^ 0x1021U) :
                  (uint16_t)(crc << 1);
        }
    }
    return crc;
}

uint16_t V22Protocol_Encode(uint8_t *buffer,
                            uint16_t capacity,
                            const V22Frame_t *frame)
{
    uint16_t frameLength;
    uint16_t crc;
    uint8_t index;

    if ((buffer == 0) || (frame == 0) || (frame->length > V22_MAX_PAYLOAD))
    {
        return 0U;
    }

    frameLength = (uint16_t)(11U + frame->length);
    if (capacity < frameLength)
    {
        return 0U;
    }

    buffer[0] = V22_HEAD0;
    buffer[1] = V22_HEAD1;
    buffer[2] = V22_VERSION;
    buffer[3] = frame->type;
    buffer[4] = frame->source;
    buffer[5] = frame->destination;
    buffer[6] = frame->sequence;
    buffer[7] = frame->flags;
    buffer[8] = frame->length;
    for (index = 0U; index < frame->length; ++index)
    {
        buffer[(uint16_t)9U + index] = frame->payload[index];
    }

    crc = V22Protocol_Crc16CcittFalse(&buffer[2], (uint16_t)(7U + frame->length));
    buffer[(uint16_t)9U + frame->length] = (uint8_t)(crc >> 8);
    buffer[(uint16_t)10U + frame->length] = (uint8_t)crc;
    return frameLength;
}

void V22Protocol_ParserInit(V22StreamParser_t *parser)
{
    if (parser == 0)
    {
        return;
    }
    parser->index = 0U;
    parser->expectedLength = 0U;
    parser->lastByteMs = 0U;
}

V22ParseResult_t V22Protocol_ParserCheckTimeout(V22StreamParser_t *parser,
                                                uint32_t nowMs)
{
    if ((parser == 0) || (parser->index == 0U) ||
        ((uint32_t)(nowMs - parser->lastByteMs) <= V22_STREAM_TIMEOUT_MS))
    {
        return V22_PARSE_NONE;
    }

    parser->index = 0U;
    parser->expectedLength = 0U;
    return V22_PARSE_TIMEOUT;
}

V22ParseResult_t V22Protocol_ParserPush(V22StreamParser_t *parser,
                                        uint8_t byte,
                                        uint32_t nowMs,
                                        V22Frame_t *frame)
{
    uint8_t payloadLength;
    uint16_t receivedCrc;
    uint16_t calculatedCrc;
    uint8_t index;

    if ((parser == 0) || (frame == 0))
    {
        return V22_PARSE_NONE;
    }

    parser->lastByteMs = nowMs;

    if (parser->index == 0U)
    {
        if (byte == V22_HEAD0)
        {
            parser->buffer[0] = byte;
            parser->index = 1U;
        }
        return V22_PARSE_NONE;
    }

    if (parser->index == 1U)
    {
        if (byte == V22_HEAD1)
        {
            parser->buffer[1] = byte;
            parser->index = 2U;
        }
        else
        {
            V22Protocol_ResetOrRestart(parser, byte);
        }
        return V22_PARSE_NONE;
    }

    if (parser->index >= V22_MAX_FRAME_BYTES)
    {
        V22Protocol_ResetOrRestart(parser, byte);
        return V22_PARSE_LENGTH_ERROR;
    }

    parser->buffer[parser->index++] = byte;
    if (parser->index == 9U)
    {
        if (parser->buffer[2] != V22_VERSION)
        {
            V22Protocol_ResetOrRestart(parser, byte);
            return V22_PARSE_VERSION_ERROR;
        }

        payloadLength = parser->buffer[8];
        if (payloadLength > V22_MAX_PAYLOAD)
        {
            V22Protocol_ResetOrRestart(parser, byte);
            return V22_PARSE_LENGTH_ERROR;
        }
        parser->expectedLength = (uint8_t)(11U + payloadLength);
    }

    if ((parser->expectedLength != 0U) &&
        (parser->index == parser->expectedLength))
    {
        payloadLength = parser->buffer[8];
        receivedCrc = (uint16_t)(((uint16_t)parser->buffer[(uint16_t)9U + payloadLength] << 8) |
                                 parser->buffer[(uint16_t)10U + payloadLength]);
        calculatedCrc = V22Protocol_Crc16CcittFalse(&parser->buffer[2],
                                                    (uint16_t)(7U + payloadLength));
        if (receivedCrc != calculatedCrc)
        {
            V22Protocol_ResetOrRestart(parser, byte);
            return V22_PARSE_CRC_ERROR;
        }

        frame->version = parser->buffer[2];
        frame->type = parser->buffer[3];
        frame->source = parser->buffer[4];
        frame->destination = parser->buffer[5];
        frame->sequence = parser->buffer[6];
        frame->flags = parser->buffer[7];
        frame->length = payloadLength;
        for (index = 0U; index < payloadLength; ++index)
        {
            frame->payload[index] = parser->buffer[(uint16_t)9U + index];
        }
        parser->index = 0U;
        parser->expectedLength = 0U;
        return V22_PARSE_FRAME;
    }

    return V22_PARSE_NONE;
}

uint8_t V22Protocol_DestinationAccepted(const V22Frame_t *frame,
                                        uint8_t localAddress)
{
    if (frame == 0)
    {
        return 0U;
    }
    return ((frame->destination == V22_ADDR_BROADCAST) ||
            (frame->destination == localAddress)) ? 1U : 0U;
}

const char *V22Protocol_ParseResultName(V22ParseResult_t result)
{
    switch (result)
    {
        case V22_PARSE_FRAME:         return "FRAME";
        case V22_PARSE_VERSION_ERROR: return "VERSION";
        case V22_PARSE_LENGTH_ERROR:  return "LENGTH";
        case V22_PARSE_CRC_ERROR:     return "CRC";
        case V22_PARSE_TIMEOUT:       return "TIMEOUT";
        default:                      return "NONE";
    }
}
