/**
  ******************************************************************************
  * @file    arm_usb_protocol.c
  * @brief   Frame building and payload codecs for the arm USB link.
  ******************************************************************************
  */

#include "arm_usb_protocol.h"

/* ------------------------------------------------------- byte helpers */

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* ---------------------------------------------------------------- crc */

uint16_t arm_proto_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8U; bit++)
        {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
        }
    }

    return crc;
}

/* -------------------------------------------------------------- frame */

uint16_t arm_proto_build_frame(uint8_t *dst, uint16_t capacity, uint8_t type,
                               uint8_t sequence, uint32_t timestamp_ms,
                               const uint8_t *payload, uint16_t payload_len)
{
    uint16_t frame_len = (uint16_t)(ARM_PROTO_OVERHEAD + payload_len);

    if (dst == 0 || payload_len > ARM_PROTO_MAX_PAYLOAD || capacity < frame_len ||
        (payload_len > 0U && payload == 0))
    {
        return 0U;
    }

    dst[0] = ARM_PROTO_SOF;
    dst[1] = ARM_PROTO_VERSION;
    dst[2] = type;
    dst[3] = sequence;
    put_u16(&dst[4], payload_len);
    put_u32(&dst[6], timestamp_ms);

    for (uint16_t i = 0; i < payload_len; i++)
    {
        dst[ARM_PROTO_HEADER_SIZE + i] = payload[i];
    }

    put_u16(&dst[ARM_PROTO_HEADER_SIZE + payload_len],
            arm_proto_crc16(dst, (uint16_t)(ARM_PROTO_HEADER_SIZE + payload_len)));

    return frame_len;
}

void arm_proto_parse_header(const uint8_t *src, ArmFrameHeader_t *hdr)
{
    hdr->sof          = src[0];
    hdr->version      = src[1];
    hdr->type         = src[2];
    hdr->sequence     = src[3];
    hdr->payload_len  = get_u16(&src[4]);
    hdr->timestamp_ms = get_u32(&src[6]);
}

uint8_t arm_proto_frame_crc_ok(const uint8_t *frame, uint16_t payload_len)
{
    uint16_t body = (uint16_t)(ARM_PROTO_HEADER_SIZE + payload_len);
    return (uint8_t)(arm_proto_crc16(frame, body) == get_u16(&frame[body]));
}

/* ------------------------------------------------------------ payloads */

uint16_t arm_proto_encode_state(uint8_t *dst, const ArmStatePayload_t *s)
{
    uint16_t o = 0;

    put_u32(&dst[o], s->timestamp_ms);  o += 4U;
    dst[o++] = s->arm_mode;
    dst[o++] = s->system_state;
    put_u16(&dst[o], s->fault_code);    o += 2U;
    dst[o++] = s->flags;
    dst[o++] = s->feedback_valid;
    put_u16(&dst[o], 0U);               o += 2U;    /* reserved */

    for (uint8_t i = 0; i < ARM_PROTO_JOINT_NUM; i++, o += 2U)
    {
        put_u16(&dst[o], s->feedback_age_ms[i]);
    }
    for (uint8_t i = 0; i < ARM_PROTO_JOINT_NUM; i++, o += 4U)
    {
        put_u32(&dst[o], (uint32_t)s->position_urad[i]);
    }
    for (uint8_t i = 0; i < ARM_PROTO_JOINT_NUM; i++, o += 4U)
    {
        put_u32(&dst[o], (uint32_t)s->velocity_urad_s[i]);
    }
    for (uint8_t i = 0; i < ARM_PROTO_JOINT_NUM; i++, o += 4U)
    {
        put_u32(&dst[o], (uint32_t)s->torque_mNm[i]);
    }
    for (uint8_t i = 0; i < ARM_PROTO_JOINT_NUM; i++, o += 4U)
    {
        put_u32(&dst[o], (uint32_t)s->target_position_urad[i]);
    }

    return o;   /* ARM_STATE_PAYLOAD_SIZE */
}

void arm_proto_decode_position_cmd(const uint8_t *src, ArmPositionCommand_t *cmd)
{
    uint16_t o = 0;

    cmd->command_timestamp_ms = get_u32(&src[o]);  o += 4U;
    cmd->valid_time_ms        = get_u16(&src[o]);  o += 2U;
    cmd->enable               = src[o];            o += 2U;  /* + reserved */

    for (uint8_t i = 0; i < ARM_PROTO_JOINT_NUM; i++, o += 4U)
    {
        cmd->position_urad[i] = (int32_t)get_u32(&src[o]);
    }
    for (uint8_t i = 0; i < ARM_PROTO_JOINT_NUM; i++, o += 4U)
    {
        cmd->velocity_limit_urad_s[i] = (int32_t)get_u32(&src[o]);
    }
}

uint16_t arm_proto_encode_ack(uint8_t *dst, uint8_t acked_type, uint8_t acked_seq,
                              uint8_t result)
{
    dst[0] = acked_type;
    dst[1] = acked_seq;
    dst[2] = result;
    dst[3] = 0U;
    return ARM_ACK_PAYLOAD_SIZE;
}

uint16_t arm_proto_encode_fault(uint8_t *dst, uint16_t fault_code, uint8_t system_state)
{
    put_u16(&dst[0], fault_code);
    dst[2] = system_state;
    dst[3] = 0U;
    return ARM_FAULT_PAYLOAD_SIZE;
}

/* ---------------------------------------------------------------- units */

static int32_t float_to_i32_sat(float v)
{
    if (!(v == v))                  /* NaN */
    {
        return 0;
    }
    if (v >= 2147483520.0f)         /* largest float below INT32_MAX */
    {
        return INT32_MAX;
    }
    if (v <= -2147483648.0f)
    {
        return INT32_MIN;
    }
    return (int32_t)(v >= 0.0f ? v + 0.5f : v - 0.5f);
}

int32_t arm_proto_rad_to_urad(float rad)
{
    return float_to_i32_sat(rad * ARM_URAD_PER_RAD);
}

int32_t arm_proto_nm_to_mnm(float nm)
{
    return float_to_i32_sat(nm * ARM_MNM_PER_NM);
}

float arm_proto_urad_to_rad(int32_t urad)
{
    return (float)urad / ARM_URAD_PER_RAD;
}
