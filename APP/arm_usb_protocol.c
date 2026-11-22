/**
  ******************************************************************************
  * @file    arm_usb_protocol.c
  * @brief   Frame building and payload codecs for the arm USB link.
  ******************************************************************************
  */
/*
 * 编解码实现：所有多字节字段逐字节写出/读出，一律小端（低字节在前），
 * 不依赖编译器对齐、填充或主机字节序，因此 C 结构体布局永远不上线。
 * 帧格式见 arm_usb_protocol.h：头部 10 B + 载荷 N + CRC 2 B，CRC 覆盖 [0, 10+N)。
 * 本文件无全局状态，全部函数可重入；调用上下文见各函数注释。
 */

#include "arm_usb_protocol.h"

/* ------------------------------------------------------- byte helpers */

/* 小端写 u16：先低字节、后高字节。 */
static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v);  // 低字节在前
    p[1] = (uint8_t)(v >> 8);
}

/* 小端写 u32：依次写出 b0..b3（b0 = 最低字节）。 */
static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v);  // b0 = 最低字节
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

/* 小端读 u16：与 put_u16 对称。 */
static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

/* 小端读 u32：与 put_u32 对称。 */
static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* ---------------------------------------------------------------- crc */

/* CRC16-CCITT-FALSE：初值 0xFFFF、多项式 0x1021、不反射输入/输出。
   参数：data/len 参与校验的字节数；返回 16 位校验值。
   逐位实现约 8×len 次循环，单帧最多 172 B，100 Hz 下开销可忽略。
   上下文：任意（无状态），任务上下文与自测代码都会调用。 */
uint16_t arm_proto_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;  // 初值 0xFFFF（CCITT-FALSE 约定）

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;  // 先异或到高字节，再左移 8 次
        for (uint8_t bit = 0; bit < 8U; bit++)
        {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);  // 最高位为 1 才异或多项式，左移溢出由 uint16_t 自然截断
        }
    }

    return crc;  // 对 123456789 应得 0x29B1（CCITT-FALSE 标准校验值）
}

/* -------------------------------------------------------------- frame */

/* 组一帧到 dst。
   参数：dst 目标缓冲；capacity 其容量；type/sequence/timestamp_ms 填头部；
        payload/payload_len 载荷（payload_len 为 0 时 payload 可为 NULL）。
   返回：整帧长度（12 + payload_len）；参数非法或容量不足返回 0。
   上下文：任务上下文（send_tx 调用），不做并发保护。 */
uint16_t arm_proto_build_frame(uint8_t *dst, uint16_t capacity, uint8_t type,
                               uint8_t sequence, uint32_t timestamp_ms,
                               const uint8_t *payload, uint16_t payload_len)
{
    uint16_t frame_len = (uint16_t)(ARM_PROTO_OVERHEAD + payload_len);  // 总长 = 10 B 头 + N + 2 B CRC

    /* 参数非法或容量不足：返回 0，调用方据此跳过这一帧。 */
    if (dst == 0 || payload_len > ARM_PROTO_MAX_PAYLOAD || capacity < frame_len ||
        (payload_len > 0U && payload == 0))
    {
        return 0U;
    }

    dst[0] = ARM_PROTO_SOF;  // 偏移 0：SOF 0xA5
    dst[1] = ARM_PROTO_VERSION;  // 偏移 1：版本号
    dst[2] = type;  // 偏移 2：消息类型
    dst[3] = sequence;  // 偏移 3：序号（本端自增，8 位回绕）
    put_u16(&dst[4], payload_len);  // 偏移 4：payload_len，小端
    put_u32(&dst[6], timestamp_ms);  // 偏移 6：发送方毫秒时戳

    for (uint16_t i = 0; i < payload_len; i++)
    {
        dst[ARM_PROTO_HEADER_SIZE + i] = payload[i];  // 偏移 10 起：载荷原样拷贝，不做对齐
    }

    /* CRC 放在载荷之后（偏移 10+N），覆盖头部与载荷，低字节在前。 */
    put_u16(&dst[ARM_PROTO_HEADER_SIZE + payload_len],
            arm_proto_crc16(dst, (uint16_t)(ARM_PROTO_HEADER_SIZE + payload_len)));

    return frame_len;
}

/* 解出 10 B 头部到 hdr。不校验 SOF 与 CRC，也不检查 payload_len 是否合法，
   这些由调用方（parse_rx）负责。 */
void arm_proto_parse_header(const uint8_t *src, ArmFrameHeader_t *hdr)
{
    hdr->sof          = src[0];  // 偏移 0：SOF
    hdr->version      = src[1];  // 偏移 1：版本
    hdr->type         = src[2];  // 偏移 2：消息类型
    hdr->sequence     = src[3];  // 偏移 3：序号
    hdr->payload_len  = get_u16(&src[4]);  // 偏移 4：payload_len（小端）
    hdr->timestamp_ms = get_u32(&src[6]);  // 偏移 6：发送方时戳
}

/* 校验整帧 CRC。参数：frame 整帧起点；payload_len 取自帧头。
   返回 1 = 通过。调用前必须确认整帧已在缓冲里。 */
uint8_t arm_proto_frame_crc_ok(const uint8_t *frame, uint16_t payload_len)
{
    uint16_t body = (uint16_t)(ARM_PROTO_HEADER_SIZE + payload_len);  // 参与校验的长度 = 10 + N（头部 + 载荷）
    return (uint8_t)(arm_proto_crc16(frame, body) == get_u16(&frame[body]));  // CRC 紧跟载荷之后，同样低字节在前
}

/* ------------------------------------------------------------ payloads */

/* 编码 STATE 载荷（138 B）。
   参数：dst 输出缓冲（至少 ARM_STATE_PAYLOAD_SIZE 字节）；s 输入快照。
   返回：写入长度，恒为 ARM_STATE_PAYLOAD_SIZE。
   上下文：任务上下文；s 由 fill_state 在临界区里填好，本函数只读。 */
uint16_t arm_proto_encode_state(uint8_t *dst, const ArmStatePayload_t *s)
{
    uint16_t o = 0;

    put_u32(&dst[o], s->timestamp_ms);  o += 4U;  // 偏移 0..3：时戳
    dst[o++] = s->arm_mode;  // 偏移 4：运动模式
    dst[o++] = s->system_state;  // 偏移 5：系统状态
    put_u16(&dst[o], s->fault_code);    o += 2U;  // 偏移 6..7：故障码
    dst[o++] = s->flags;  // 偏移 8：状态位图
    dst[o++] = s->feedback_valid;  // 偏移 9：每关节反馈有效掩码
    /* 偏移 10..11：保留字段（填 0），让后面的 age 数组对齐到 12。 */
    put_u16(&dst[o], 0U);               o += 2U;    /* reserved */

    /* 偏移 12..25：7 × u16 反馈年龄（ms），0xFFFF = 从未收到。 */
    for (uint8_t i = 0; i < ARM_PROTO_JOINT_NUM; i++, o += 2U)
    {
        put_u16(&dst[o], s->feedback_age_ms[i]);
    }
    /* 偏移 26..53：7 × i32 位置。负数按补码位型写出，上位机按 int32 解释。 */
    for (uint8_t i = 0; i < ARM_PROTO_JOINT_NUM; i++, o += 4U)
    {
        put_u32(&dst[o], (uint32_t)s->position_urad[i]);
    }
    /* 偏移 54..81：7 × i32 速度（µrad/s）。 */
    for (uint8_t i = 0; i < ARM_PROTO_JOINT_NUM; i++, o += 4U)
    {
        put_u32(&dst[o], (uint32_t)s->velocity_urad_s[i]);
    }
    /* 偏移 82..109：7 × i32 力矩（mN·m）。 */
    for (uint8_t i = 0; i < ARM_PROTO_JOINT_NUM; i++, o += 4U)
    {
        put_u32(&dst[o], (uint32_t)s->torque_mNm[i]);
    }
    /* 偏移 110..137：7 × i32 目标位置（µrad）。 */
    for (uint8_t i = 0; i < ARM_PROTO_JOINT_NUM; i++, o += 4U)
    {
        put_u32(&dst[o], (uint32_t)s->target_position_urad[i]);
    }

    /* 返回写入长度，恒等于 ARM_STATE_PAYLOAD_SIZE（138）。 */
    return o;   /* ARM_STATE_PAYLOAD_SIZE */
}

/* 解码 POSITION_CMD 载荷（64 B）。
   参数：src 载荷起点；cmd 输出结构体。
   不校验数值范围与使能合法性 —— 越限需要整条指令一起判，放在业务层做。 */
void arm_proto_decode_position_cmd(const uint8_t *src, ArmPositionCommand_t *cmd)
{
    uint16_t o = 0;

    cmd->command_timestamp_ms = get_u32(&src[o]);  o += 4U;  // 偏移 0..3：主机时戳
    cmd->valid_time_ms        = get_u16(&src[o]);  o += 2U;  // 偏移 4..5：有效期（ms）
    /* 偏移 6：使能；偏移 7：保留字节，一起跳过。 */
     /*（下面这行通过 o += 2U 同时越过 enable 与 reserved）*/
    cmd->enable               = src[o];            o += 2U;  /* + reserved */

    /* 偏移 8..35：7 × i32 位置（µrad）。 */
    for (uint8_t i = 0; i < ARM_PROTO_JOINT_NUM; i++, o += 4U)
    {
        cmd->position_urad[i] = (int32_t)get_u32(&src[o]);
    }
    /* 偏移 36..63：7 × i32 速度上限（µrad/s），当前保留但已参与校验。 */
    for (uint8_t i = 0; i < ARM_PROTO_JOINT_NUM; i++, o += 4U)
    {
        cmd->velocity_limit_urad_s[i] = (int32_t)get_u32(&src[o]);
    }
}

/* 编码 ACK 载荷（4 B）：被应答类型、被应答序号、结果、保留字节。 */
uint16_t arm_proto_encode_ack(uint8_t *dst, uint8_t acked_type, uint8_t acked_seq,
                              uint8_t result)
{
    dst[0] = acked_type;  // 被应答的消息类型
    dst[1] = acked_seq;  // 被应答帧的序号
    dst[2] = result;  // ArmAckResult_t
    dst[3] = 0U;
    return ARM_ACK_PAYLOAD_SIZE;
}

/* 编码 FAULT 载荷（4 B）：故障位 u16、系统状态 u8、保留。 */
uint16_t arm_proto_encode_fault(uint8_t *dst, uint16_t fault_code, uint8_t system_state)
{
    put_u16(&dst[0], fault_code);
    dst[2] = system_state;
    dst[3] = 0U;
    return ARM_FAULT_PAYLOAD_SIZE;
}

/* ---------------------------------------------------------------- units */

/* float -> int32 饱和转换：NaN 归 0，越界钳位，避免未定义行为
   （x86 与 Cortex-M7 对溢出转换的处理并不相同）。
   参数 v 已是定点单位的浮点值；返回四舍五入后的整数。 */
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
    if (v <= -2147483648.0f)  // 下溢同样饱和到 INT32_MIN
    {
        return INT32_MIN;
    }
    return (int32_t)(v >= 0.0f ? v + 0.5f : v - 0.5f);  // 四舍五入到最近整数（µrad 精度下 0.5 足够）
}

/* rad -> µrad（×1e6），饱和；用于状态帧上报。 */
int32_t arm_proto_rad_to_urad(float rad)
{
    return float_to_i32_sat(rad * ARM_URAD_PER_RAD);
}

/* N·m -> mN·m（×1000），饱和。 */
int32_t arm_proto_nm_to_mnm(float nm)
{
    return float_to_i32_sat(nm * ARM_MNM_PER_NM);
}

/* µrad -> rad（÷1e6）；上位机下发的目标在这里换成控制用的弧度。 */
float arm_proto_urad_to_rad(int32_t urad)
{
    return (float)urad / ARM_URAD_PER_RAD;
}
