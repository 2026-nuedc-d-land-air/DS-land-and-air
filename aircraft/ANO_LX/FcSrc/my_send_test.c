#include "my_send_test.h"
#include "my_uart.h"

#ifndef UNIT_TEST
#include "Drv_Uart.h"
#else
extern void DrvUart1SendBuf(unsigned char *data, uint8_t len);
extern void DrvUart3SendBuf(unsigned char *data, uint8_t len);
#define BYTE0(value) (*((char *)(&(value))))
#define BYTE1(value) (*((char *)(&(value)) + 1))
#endif

#define LX_EXT_MARKER                 0x30u
#define V21_HEAD_0                    0xAAu
#define V21_HEAD_1                    0x55u
#define V21_VERSION                   0x02u
#define V21_ADDR_FC                   0x21u
#define V21_ADDR_CAMERA               0x50u
#define V21_MAX_PAYLOAD               64u
#define ENABLE_LEGACY_CAMERA_BENCH    0

static uint8_t s_camera_tx_seq;

static void put_be16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value >> 8);
    dst[1] = (uint8_t)value;
}

static void put_be32(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value >> 24);
    dst[1] = (uint8_t)(value >> 16);
    dst[2] = (uint8_t)(value >> 8);
    dst[3] = (uint8_t)value;
}

uint16_t my_v21_crc16_ccitt_false(const uint8_t *data, uint8_t len)
{
    uint16_t crc = 0xFFFFu;
    uint8_t i;
    uint8_t bit;

    for (i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (bit = 0; bit < 8u; bit++)
        {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void append_lx_checksums(uint8_t *frame, uint8_t covered_len)
{
    uint8_t sc = 0;
    uint8_t ac = 0;
    uint8_t i;

    for (i = 0; i < covered_len; i++)
    {
        sc = (uint8_t)(sc + frame[i]);
        ac = (uint8_t)(ac + sc);
    }
    frame[covered_len] = sc;
    frame[covered_len + 1u] = ac;
}

void my_send_esp_4_test(int Data_A, int Data_B, int Data_C, int Data_D)
{
    uint8_t frame[15];
    uint8_t count = 0;

    frame[count++] = SEND_ESP_HEAD;
    frame[count++] = SEND_ESP_ADDR;
    frame[count++] = 0xF1u;
    frame[count++] = 0x08u;
    frame[count++] = BYTE0(Data_A);
    frame[count++] = BYTE1(Data_A);
    frame[count++] = BYTE0(Data_B);
    frame[count++] = BYTE1(Data_B);
    frame[count++] = BYTE0(Data_C);
    frame[count++] = BYTE1(Data_C);
    frame[count++] = BYTE0(Data_D);
    frame[count++] = BYTE1(Data_D);
    frame[count++] = SEND_ESP_END;
    append_lx_checksums(frame, count);
    DrvUart1SendBuf(frame, (uint8_t)(count + 2u));
}

void my_send_esp_1_test(int Data_A)
{
    uint8_t frame[9];
    uint8_t count = 0;

    frame[count++] = SEND_ESP_HEAD;
    frame[count++] = SEND_ESP_ADDR;
    frame[count++] = 0xF1u;
    frame[count++] = 0x02u;
    frame[count++] = BYTE0(Data_A);
    frame[count++] = BYTE1(Data_A);
    frame[count++] = SEND_ESP_END;
    append_lx_checksums(frame, count);
    DrvUart1SendBuf(frame, (uint8_t)(count + 2u));
}

void my_send_lx_extended(uint8_t subtype, const uint8_t *payload, uint8_t payload_len)
{
    uint8_t frame[73];
    uint8_t count = 0;
    uint8_t i;

    if (payload_len > V21_MAX_PAYLOAD || (payload_len != 0u && payload == 0))
    {
        return;
    }

    frame[count++] = SEND_ESP_HEAD;
    frame[count++] = SEND_ESP_ADDR;
    frame[count++] = 0xF1u;
    frame[count++] = LX_EXT_MARKER;
    frame[count++] = subtype;
    frame[count++] = payload_len;
    for (i = 0; i < payload_len; i++)
    {
        frame[count++] = payload[i];
    }
    frame[count++] = SEND_ESP_END;
    append_lx_checksums(frame, count);
    DrvUart1SendBuf(frame, (uint8_t)(count + 2u));
}

void my_send_esp_mission_response(uint8_t request_type,
                                  uint16_t mission_id,
                                  uint8_t result,
                                  uint8_t detail)
{
    uint8_t payload[6];

    payload[0] = request_type;
    put_be16(&payload[1], mission_id);
    payload[3] = result;
    payload[4] = detail;
    payload[5] = 0u;
    my_send_lx_extended(LX_EXT_MISSION_RESPONSE, payload, sizeof(payload));
}

void my_send_esp_mission_status(uint8_t task_type,
                                uint8_t stage,
                                uint16_t status_flags,
                                uint16_t mission_id,
                                uint8_t error_code,
                                uint32_t source_time_ms)
{
    uint8_t payload[12];

    payload[0] = task_type;
    payload[1] = stage;
    put_be16(&payload[2], status_flags);
    put_be16(&payload[4], mission_id);
    payload[6] = error_code;
    payload[7] = 0u;
    put_be32(&payload[8], source_time_ms);
    my_send_lx_extended(LX_EXT_MISSION_STATUS, payload, sizeof(payload));
}

void my_send_esp_mission_trace(const mission_trace_t *trace)
{
    uint8_t payload[35];

    if (trace == 0)
    {
        return;
    }

    payload[0] = trace->event;
    payload[1] = trace->stage;
    payload[2] = trace->error_code;
    payload[3] = trace->flags;
    put_be16(&payload[4], trace->mission_id);
    put_be32(&payload[6], trace->fc_time_ms);
    put_be32(&payload[10], trace->stage_elapsed_ms);
    put_be16(&payload[14], (uint16_t)trace->air_x_cm);
    put_be16(&payload[16], (uint16_t)trace->air_y_cm);
    put_be16(&payload[18], (uint16_t)trace->home_x_cm);
    put_be16(&payload[20], (uint16_t)trace->home_y_cm);
    put_be16(&payload[22], (uint16_t)trace->height_cm);
    put_be16(&payload[24], (uint16_t)trace->range_target_cm);
    payload[26] = trace->camera_flags;
    payload[27] = trace->camera_quality;
    put_be16(&payload[28], (uint16_t)trace->camera_err_x_cm);
    put_be16(&payload[30], (uint16_t)trace->camera_err_y_cm);
    put_be16(&payload[32], trace->camera_age_ms);
    payload[34] = trace->align_blockers;
    my_send_lx_extended(LX_EXT_MISSION_TRACE, payload, sizeof(payload));
}

uint8_t my_camera_allocate_seq(void)
{
    uint8_t seq = s_camera_tx_seq;
    s_camera_tx_seq = (uint8_t)(s_camera_tx_seq + 1u);
    return seq;
}

static void send_camera_v22(uint8_t type,
                            uint8_t seq,
                            uint8_t header_flags,
                            const uint8_t *payload,
                            uint8_t payload_len)
{
    uint8_t frame[75];
    uint8_t count = 0;
    uint8_t i;
    uint16_t crc;

    if (payload_len > V21_MAX_PAYLOAD || (payload_len != 0u && payload == 0))
    {
        return;
    }

    frame[count++] = V21_HEAD_0;
    frame[count++] = V21_HEAD_1;
    frame[count++] = V21_VERSION;
    frame[count++] = type;
    frame[count++] = V21_ADDR_FC;
    frame[count++] = V21_ADDR_CAMERA;
    frame[count++] = seq;
    frame[count++] = header_flags;
    frame[count++] = payload_len;
    for (i = 0; i < payload_len; i++)
    {
        frame[count++] = payload[i];
    }

    crc = my_v21_crc16_ccitt_false(&frame[2], (uint8_t)(7u + payload_len));
    frame[count++] = (uint8_t)(crc >> 8);
    frame[count++] = (uint8_t)crc;
    DrvUart3SendBuf(frame, count);
}

void my_send_camera_mode_v22(uint8_t seq,
                             uint8_t header_flags,
                             uint8_t mode,
                             uint8_t task_type,
                             uint16_t mission_id,
                             uint8_t mode_flags)
{
    uint8_t payload[6];

    payload[0] = mode;
    payload[1] = task_type;
    put_be16(&payload[2], mission_id);
    payload[4] = mode_flags;
    payload[5] = 0u;
    send_camera_v22(CAMERA_TYPE_MODE, seq, header_flags, payload, sizeof(payload));
}

void my_send_camera_action_v22(uint8_t seq,
                               uint8_t header_flags,
                               uint8_t action,
                               uint8_t action_flags,
                               uint16_t action_id,
                               uint8_t task_type,
                               uint16_t mission_id,
                               uint8_t mode_seq)
{
    uint8_t payload[8];

    payload[0] = action;
    payload[1] = action_flags;
    put_be16(&payload[2], action_id);
    payload[4] = task_type;
    put_be16(&payload[5], mission_id);
    payload[7] = mode_seq;
    send_camera_v22(CAMERA_TYPE_ACTION, seq, header_flags, payload, sizeof(payload));
}

void my_send_maixcam(uint8_t code_type)
{
#if ENABLE_LEGACY_CAMERA_BENCH
    uint8_t frame[3] = {0xBBu, code_type, 0xFFu};
    DrvUart3SendBuf(frame, sizeof(frame));
#else
    (void)code_type;
#endif
}
