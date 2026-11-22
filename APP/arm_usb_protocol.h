/**
  ******************************************************************************
  * @file    arm_usb_protocol.h
  * @brief   Wire format of the arm <-> ROS2 USB CDC link, version 2.
  *
  * Every field is little endian and written byte by byte by the helpers in
  * arm_usb_protocol.c, so the C struct layout never reaches the wire.
  *
  *   off  size  field
  *    0    1    sof          0xA5
  *    1    1    version      ARM_PROTO_VERSION
  *    2    1    type         ArmMsgType_t
  *    3    1    sequence     per sender, wraps
  *    4    2    payload_len
  *    6    4    timestamp_ms sender tick
  *   10    N    payload
  * 10+N    2    crc16        CRC16-CCITT (poly 0x1021, init 0xFFFF) over
  *                           bytes [0, 10+N), low byte first
  *
  * Angles are int32 micro-radians, angular speed int32 micro-rad/s,
  * torque int32 milli-newton-metres.
  *
  * Comments stay ASCII: the project mixes GBK and UTF-8 sources.
  ******************************************************************************
  */
/*
 * 中文说明（本模块 = 上位机 ROS2/MoveIt2 与机械臂固件之间的 USB CDC 线协议）
 * 链路：USB 全速 CDC 虚拟串口，固件侧 100 Hz 发 STATE 帧。
 *
 * 帧结构（共 12 + N 字节，全部小端）：
 *   偏移  0 : SOF = 0xA5         解析器逐字节搜索它做重同步
 *   偏移  1 : version = 0x02     版本不符直接拒绝，不做版本协商
 *   偏移  2 : type               ArmMsgType_t
 *   偏移  3 : sequence           每个发送方独立自增，8 位回绕
 *   偏移  4 : payload_len u16    <= ARM_PROTO_MAX_PAYLOAD(160)
 *   偏移  6 : timestamp_ms u32   发送方本地毫秒时基，两端不对钟
 *   偏移 10 : payload            长度 N
 *   偏移10+N: crc16              覆盖 [0, 10+N) 全部字节，低字节在前
 *
 * 单位：角度 µrad、角速度 µrad/s、力矩 mN·m。定点在协议层完成，C 结构体
 * 布局永远不上线，全部由 arm_usb_protocol.c 逐字节搬运。
 */

#ifndef __ARM_USB_PROTOCOL_H
#define __ARM_USB_PROTOCOL_H

#include <stdint.h>

#define ARM_PROTO_SOF               0xA5U  // 帧起始字节；载荷里偶然出现 0xA5 时靠长度 + CRC 校验剔除
#define ARM_PROTO_VERSION           0x02U  // 协议版本；版本不符回 ARM_ACK_BAD_VERSION
#define ARM_PROTO_HEADER_SIZE       10U  // 10 B = 1 + 1 + 1 + 1 + 2 + 4
#define ARM_PROTO_CRC_SIZE          2U  // 2 B，低字节在前
#define ARM_PROTO_OVERHEAD          (ARM_PROTO_HEADER_SIZE + ARM_PROTO_CRC_SIZE)  // 12 B：不含 payload 的固定开销
#define ARM_PROTO_MAX_PAYLOAD       160U  // 160 B：STATE 的 138 B 与最长诊断页的 160 B 都在此之内
#define ARM_PROTO_MAX_FRAME         (ARM_PROTO_OVERHEAD + ARM_PROTO_MAX_PAYLOAD)  // 172 B：单帧最大长度，收发缓冲按它分配

#define ARM_PROTO_JOINT_NUM         7U  // 7 个关节：J1..J6 + 夹爪，顺序与线上数组一致

/* 消息类型与方向：
 *   0x01 STATE        MCU -> PC，每个任务周期一帧（100 Hz）
 *   0x02 POSITION_CMD PC  -> MCU，7 关节目标位置（µrad）+ 时戳 + 有效期
 *   0x03 ENABLE       PC  -> MCU，4 B，payload[0] = 1 使能 / 0 撤销
 *   0x04 STOP         PC  -> MCU，0 B，断使能并丢目标
 *   0x05 HEARTBEAT    PC  -> MCU，0 B，只为刷新链路活跃时间
 *   0x06 ACK          MCU -> PC，4 B，应答类型 + 序号 + 结果
 *   0x07 FAULT        MCU -> PC，4 B，故障位变化时主动上报
 *   0x08 DIAGNOSTIC   MCU -> PC，可选，500 ms 一帧、两页交替
 * 未知类型回 ARM_ACK_UNKNOWN_TYPE，但链路保持。
 */
typedef enum
{
    ARM_MSG_STATE        = 0x01,    /* MCU -> PC, every task period        */
    ARM_MSG_POSITION_CMD = 0x02,    /* PC -> MCU                           */
    ARM_MSG_ENABLE       = 0x03,    /* PC -> MCU, payload enable 1 / 0     */
    ARM_MSG_STOP         = 0x04,    /* PC -> MCU, disable and drop command */
    ARM_MSG_HEARTBEAT    = 0x05,    /* PC -> MCU, keeps the link alive     */
    ARM_MSG_ACK          = 0x06,    /* MCU -> PC                           */
    ARM_MSG_DIAGNOSTIC   = 0x08,    /* MCU -> PC, optional paged health data */
    ARM_MSG_FAULT        = 0x07     /* MCU -> PC, sent when faults change  */
} ArmMsgType_t;

/* STATE.system_state */
/* 对外的系统状态：由使能与指令新鲜度推导，不是独立的内部状态机。
 *   IDLE    PC 控制未使能
 *   ENABLED 已使能，但还没有新鲜的 POSITION_CMD
 *   RUNNING 已使能且持有未过期指令
 *   FAULT   有严重故障位迫使断开
 */
typedef enum
{
    ARM_SYS_IDLE    = 0x00,         /* PC control disabled                 */
    ARM_SYS_ENABLED = 0x01,         /* enabled, no fresh command           */
    ARM_SYS_RUNNING = 0x02,         /* enabled with a fresh command        */
    ARM_SYS_FAULT   = 0x03          /* a latched fault forced a disable    */
} ArmSystemState_t;

/* STATE.flags */
/* flags 是「当前存在」的位图（每帧重新计算），fault_code 是锁存的历史；
   两者都在 STATE 帧里上报，FAULT 帧再额外携带一份。 */
#define ARM_STATE_FLAG_ARM_READY    0x01U     /* Arm_L_Init() has finished    */

#define ARM_STATE_FLAG_TX_DROP 0x02U  // CAN 发送队列出现丢弃
#define ARM_STATE_FLAG_CAN_FAULT 0x04U  // CAN 控制器报总线错误
#define ARM_STATE_FLAG_CAN_RX_OVERFLOW 0x08U  // CAN 接收溢出
#define ARM_STATE_FLAG_DM_FAILED 0x10U  // 电机使能失败（DM 驱动）
#define ARM_FAULT_CAN_BUS 0x0010U  // 与 ARM_FAULT_* 共用同一故障位域，放在一起方便组合
#define ARM_FAULT_DM_ENABLE 0x0020U  // 同上：电机使能失败对应的故障位

/* STATE.fault_code / FAULT.fault_code bits. Latched until the next ENABLE. */
/* 故障位只在 USB 任务里锁存，成功 ENABLE 时整体清零；
   每一位都对应 Arm_USB_task.c 文件头「失效处理」里的一条路径。 */
#define ARM_FAULT_LINK_TIMEOUT      0x0001U   /* no valid frame while enabled */
#define ARM_FAULT_USB_DISCONNECT    0x0002U   /* USB session ended / suspended */
#define ARM_FAULT_RX_OVERFLOW       0x0004U   /* RX ring refused data         */
// 100 ms 看门狗：某个关节在 PC 控制期间停止反馈
#define ARM_FAULT_FEEDBACK_LOST     0x0008U   /* a joint stopped feeding back
                                                 while PC control was enabled */

/* ACK.result */
/* ACK.result 取值（ROS2 侧据此决定重发还是报警）：
 *   0x00 OK            成功
 *   0x01 BAD_LENGTH    payload_len 与该类型不符（如 POSITION_CMD 不是 64 B）
 *   0x02 BAD_VERSION   版本号不是 ARM_PROTO_VERSION
 *   0x03 NOT_ENABLED   本会话尚未 ENABLE，或 ENABLE 属于旧会话
 *   0x04 OUT_OF_RANGE  任一关节越限 —— 整条指令拒绝，不做单关节限幅
 *   0x05 UNKNOWN_TYPE  未知类型；链路保持
 *   0x06 BAD_VALUE     数值非法（例如 velocity_limit_urad_s 为负）
 *   0x07 STALE         时戳不新于上一条，或排队时间超过 valid_time_ms
 *   0x08 NOT_READY     ENABLE 被拒：Arm_L_Init 未完成或有关节反馈过期
 *   0x09 NO_SESSION    解析期间 USB 会话代号已变（复位/挂起插入）
 *   0x0A WRONG_MODE    ENABLE 被拒：遥控开关不在 PC 模式（arm_mode != 10）
 */
typedef enum
{
    ARM_ACK_OK           = 0x00,
    ARM_ACK_BAD_LENGTH   = 0x01,
    ARM_ACK_BAD_VERSION  = 0x02,
    ARM_ACK_NOT_ENABLED  = 0x03,
    ARM_ACK_OUT_OF_RANGE = 0x04,
    ARM_ACK_UNKNOWN_TYPE = 0x05,
    ARM_ACK_BAD_VALUE    = 0x06,
    ARM_ACK_STALE        = 0x07,    /* not newer than the last command, or
                                       queued longer than its valid time   */
    ARM_ACK_NOT_READY    = 0x08,    /* ENABLE refused: init not done or a
                                       joint has no fresh feedback          */
    ARM_ACK_NO_SESSION   = 0x09,    /* USB session changed while parsing   */
    ARM_ACK_WRONG_MODE   = 0x0A     /* ENABLE refused: the RC switch is not
                                       in PC mode (arm_mode != 10)         */
} ArmAckResult_t;

/* 载荷布局（小端、连续、无填充）：
 *
 *  STATE 138 B：
 *    off   0  u32 timestamp_ms        固件本地毫秒时基
 *    off   4  u8  arm_mode            ArmData.ArmMode 原值，10 = PC 控制
 *    off   5  u8  system_state        ArmSystemState_t
 *    off   6  u16 fault_code          锁存故障位
 *    off   8  u8  flags               当前存在的状态位图
 *    off   9  u8  feedback_valid       bit i = 关节 i 反馈有效
 *    off  10  u16 reserved
 *    off  12  u16 feedback_age_ms[7]  0xFFFF = 从未收到
 *    off  26  i32 position_urad[7]
 *    off  54  i32 velocity_urad_s[7]
 *    off  82  i32 torque_mNm[7]
 *    off 110  i32 target_position_urad[7]
 *
 *  POSITION_CMD 64 B：
 *    off   0  u32 command_timestamp_ms  主机单调毫秒，必须严格递增
 *    off   4  u16 valid_time_ms         固件夹到 10..300 ms
 *    off   6  u8  enable                0 = 保持当前姿态
 *    off   7  u8  reserved
 *    off   8  i32 position_urad[7]
 *    off  36  i32 velocity_limit_urad_s[7]  负值非法，0 = 固件默认
 *
 *  ENABLE 4 B：enable + 3 B 保留；STOP / HEARTBEAT 0 B；
 *  ACK 4 B：被应答类型、被应答序号、结果、保留；
 *  FAULT 4 B：fault_code u16、system_state u8、保留。
 */
/* ---------------------------------------------------------- payload sizes
 * STATE (138 B)
 *   timestamp_ms u32, arm_mode u8, system_state u8, fault_code u16,
 *   flags u8, feedback_valid u8 (bit i = joint i fresh), reserved u16,
 *   feedback_age_ms u16[7]  (0xFFFF = never received),
 *   position_urad i32[7], velocity_urad_s i32[7], torque_mNm i32[7]
 *     (one motor feedback snapshot),
 *   target_position_urad i32[7]  (ArmData.Jn.send_set)
 *
 * POSITION_CMD (64 B)
 *   command_timestamp_ms u32  host monotonic ms, strictly increasing,
 *   valid_time_ms u16, enable u8, reserved u8,
 *   position_urad i32[7], velocity_limit_urad_s i32[7]: signed little-endian; negative is invalid,
 *   0 = firmware default, positive = cruise limit capped by firmware.
 *   Limit reductions brake within the firmware acceleration limit.
 *
 * ENABLE 4 B: enable u8, reserved u8[3]       STOP / HEARTBEAT: 0 B
 * ACK    4 B: acked_type, acked_sequence, result, reserved
 * FAULT  4 B: fault_code u16, system_state u8, reserved
 */
#define ARM_STATE_PAYLOAD_SIZE      138U  // 138 B = 4+1+1+2+1+1+2 + 7×2 + 7×4×4
#define ARM_POS_CMD_PAYLOAD_SIZE    64U  // 64 B = 4+2+1+1 + 7×4 + 7×4
#define ARM_ENABLE_PAYLOAD_SIZE     4U  // 4 B：enable + 3 B 保留
#define ARM_ACK_PAYLOAD_SIZE        4U  // 4 B：类型、序号、结果、保留
#define ARM_FAULT_PAYLOAD_SIZE      4U  // 4 B：故障码、系统状态、保留

#define ARM_FEEDBACK_AGE_NEVER      0xFFFFU  // 年龄字段的哨兵值；0xFFFE 是最大可用值（约 65 s）

/* Unit conversion */
#define ARM_URAD_PER_RAD            1000000.0f  // 1 rad = 1e6 µrad；32 位定点可表示约 ±2147 rad
#define ARM_MNM_PER_NM              1000.0f  // 1 N·m = 1000 mN·m

/* Decoded payloads (application side, not wire layout) */
/* 下面是解码后的应用侧结构体：字段顺序与线序一致，但类型已展开成 int32/uint16，
   上位机（ROS2 侧）按同样的约定生成对应结构体。 */
typedef struct
{
    uint32_t timestamp_ms;  // 固件本地毫秒时基（与会话无关，持续递增）
    uint8_t  arm_mode;  // ArmData.ArmMode 原值
    uint8_t  system_state;  // ArmSystemState_t 取值
    uint16_t fault_code;  // 锁存故障位
    uint8_t  flags;  // 当前存在的状态位图
    uint8_t  feedback_valid;  // bit i = 关节 i 的反馈在 100 ms 内
    uint16_t feedback_age_ms[ARM_PROTO_JOINT_NUM];  // 每关节反馈年龄（ms），0xFFFF = 从未收到
    int32_t  position_urad[ARM_PROTO_JOINT_NUM];  // 位置（µrad）；上位机除以 1e6 得 rad
    int32_t  velocity_urad_s[ARM_PROTO_JOINT_NUM];  // 速度（µrad/s）
    int32_t  torque_mNm[ARM_PROTO_JOINT_NUM];  // 力矩（mN·m）
    int32_t  target_position_urad[ARM_PROTO_JOINT_NUM];  // 控制器输出的目标位置（µrad）
} ArmStatePayload_t;

/* 上位机下发的一条 POSITION_CMD，对应 MoveIt 轨迹上的一个采样点。 */
typedef struct
{
    uint32_t command_timestamp_ms;  // 主机单调毫秒时戳，同一会话内必须严格递增
    uint16_t valid_time_ms;  // 指令有效期（ms），固件夹到 ARM_USB_CMD_MIN_VALID_MS..ARM_USB_CMD_TIMEOUT_MS
    uint8_t  enable;  // 0 = 保持当前姿态（撤销目标），1 = 跟踪 position_urad
    int32_t  position_urad[ARM_PROTO_JOINT_NUM];  // 7 关节目标位置（µrad）
    int32_t  velocity_limit_urad_s[ARM_PROTO_JOINT_NUM];  // 每关节速度上限（µrad/s）：0 = 固件默认，正数取与固件上限的较小值
} ArmPositionCommand_t;

/* 解析后的帧头（10 B 的解码结果）。sof 由调用方先行校验，parse_header 不再检查。 */
typedef struct
{
    uint8_t  sof;
    uint8_t  version;
    uint8_t  type;
    uint8_t  sequence;
    uint16_t payload_len;  // 解析器先检查 <= ARM_PROTO_MAX_PAYLOAD，防伪造长度
    uint32_t timestamp_ms;
} ArmFrameHeader_t;

/* CRC16-CCITT-FALSE, check value for "123456789" is 0x29B1. */
/* CRC16-CCITT-FALSE：多项式 0x1021、初值 0xFFFF、不反射；
   对字符串 123456789 的校验值应为 0x29B1。 */
uint16_t arm_proto_crc16(const uint8_t *data, uint16_t len);

/* Build one frame into dst. Returns the frame length, 0 on bad arguments. */
/* 组帧到 dst。参数 capacity 为目标缓冲容量；返回整帧长度，参数非法或放不下时返回 0。 */
uint16_t arm_proto_build_frame(uint8_t *dst, uint16_t capacity, uint8_t type,
                               uint8_t sequence, uint32_t timestamp_ms,
                               const uint8_t *payload, uint16_t payload_len);

/* Header helpers: the parse function does not check sof / crc. */
/* 注意：parse_header 只解字段，不校验 SOF/CRC；crc_ok 要求整帧已经到齐。 */
void     arm_proto_parse_header(const uint8_t *src, ArmFrameHeader_t *hdr);
uint8_t  arm_proto_frame_crc_ok(const uint8_t *frame, uint16_t payload_len);

/* Payload codecs. encode returns the payload length. */
/* 载荷编解码：encode 系列返回写入的载荷长度；decode 假定调用方已校验长度。 */
uint16_t arm_proto_encode_state(uint8_t *dst, const ArmStatePayload_t *s);
void     arm_proto_decode_position_cmd(const uint8_t *src, ArmPositionCommand_t *cmd);
uint16_t arm_proto_encode_ack(uint8_t *dst, uint8_t acked_type, uint8_t acked_seq,
                              uint8_t result);
uint16_t arm_proto_encode_fault(uint8_t *dst, uint16_t fault_code, uint8_t system_state);

/* Saturating float -> fixed point (NaN -> 0). */
/* 单位换算：float 与定点互转，饱和处理（NaN 归 0）。 */
int32_t  arm_proto_rad_to_urad(float rad);
int32_t  arm_proto_nm_to_mnm(float nm);
float    arm_proto_urad_to_rad(int32_t urad);

#endif /* __ARM_USB_PROTOCOL_H */
