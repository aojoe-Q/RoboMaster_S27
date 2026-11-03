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

#ifndef __ARM_USB_PROTOCOL_H
#define __ARM_USB_PROTOCOL_H

#include <stdint.h>

#define ARM_PROTO_SOF               0xA5U
#define ARM_PROTO_VERSION           0x02U
#define ARM_PROTO_HEADER_SIZE       10U
#define ARM_PROTO_CRC_SIZE          2U
#define ARM_PROTO_OVERHEAD          (ARM_PROTO_HEADER_SIZE + ARM_PROTO_CRC_SIZE)
#define ARM_PROTO_MAX_PAYLOAD       160U
#define ARM_PROTO_MAX_FRAME         (ARM_PROTO_OVERHEAD + ARM_PROTO_MAX_PAYLOAD)

#define ARM_PROTO_JOINT_NUM         7U

typedef enum
{
    ARM_MSG_STATE        = 0x01,    /* MCU -> PC, every task period        */
    ARM_MSG_POSITION_CMD = 0x02,    /* PC -> MCU                           */
    ARM_MSG_ENABLE       = 0x03,    /* PC -> MCU, payload enable 1 / 0     */
    ARM_MSG_STOP         = 0x04,    /* PC -> MCU, disable and drop command */
    ARM_MSG_HEARTBEAT    = 0x05,    /* PC -> MCU, keeps the link alive     */
    ARM_MSG_ACK          = 0x06,    /* MCU -> PC                           */
    ARM_MSG_FAULT        = 0x07     /* MCU -> PC, sent when faults change  */
} ArmMsgType_t;

/* STATE.system_state */
typedef enum
{
    ARM_SYS_IDLE    = 0x00,         /* PC control disabled                 */
    ARM_SYS_ENABLED = 0x01,         /* enabled, no fresh command           */
    ARM_SYS_RUNNING = 0x02,         /* enabled with a fresh command        */
    ARM_SYS_FAULT   = 0x03          /* a latched fault forced a disable    */
} ArmSystemState_t;

/* STATE.flags */
#define ARM_STATE_FLAG_ARM_READY    0x01U     /* Arm_L_Init() has finished    */

/* STATE.fault_code / FAULT.fault_code bits. Latched until the next ENABLE. */
#define ARM_FAULT_LINK_TIMEOUT      0x0001U   /* no valid frame while enabled */
#define ARM_FAULT_USB_DISCONNECT    0x0002U   /* USB session ended / suspended */
#define ARM_FAULT_RX_OVERFLOW       0x0004U   /* RX ring refused data         */
#define ARM_FAULT_FEEDBACK_LOST     0x0008U   /* a joint stopped feeding back
                                                 while PC control was enabled */

/* ACK.result */
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
 *   position_urad i32[7], velocity_limit_urad_s i32[7] (reserved, 0)
 *
 * ENABLE 4 B: enable u8, reserved u8[3]       STOP / HEARTBEAT: 0 B
 * ACK    4 B: acked_type, acked_sequence, result, reserved
 * FAULT  4 B: fault_code u16, system_state u8, reserved
 */
#define ARM_STATE_PAYLOAD_SIZE      138U
#define ARM_POS_CMD_PAYLOAD_SIZE    64U
#define ARM_ENABLE_PAYLOAD_SIZE     4U
#define ARM_ACK_PAYLOAD_SIZE        4U
#define ARM_FAULT_PAYLOAD_SIZE      4U

#define ARM_FEEDBACK_AGE_NEVER      0xFFFFU

/* Unit conversion */
#define ARM_URAD_PER_RAD            1000000.0f
#define ARM_MNM_PER_NM              1000.0f

/* Decoded payloads (application side, not wire layout) */
typedef struct
{
    uint32_t timestamp_ms;
    uint8_t  arm_mode;
    uint8_t  system_state;
    uint16_t fault_code;
    uint8_t  flags;
    uint8_t  feedback_valid;
    uint16_t feedback_age_ms[ARM_PROTO_JOINT_NUM];
    int32_t  position_urad[ARM_PROTO_JOINT_NUM];
    int32_t  velocity_urad_s[ARM_PROTO_JOINT_NUM];
    int32_t  torque_mNm[ARM_PROTO_JOINT_NUM];
    int32_t  target_position_urad[ARM_PROTO_JOINT_NUM];
} ArmStatePayload_t;

typedef struct
{
    uint32_t command_timestamp_ms;
    uint16_t valid_time_ms;
    uint8_t  enable;
    int32_t  position_urad[ARM_PROTO_JOINT_NUM];
    int32_t  velocity_limit_urad_s[ARM_PROTO_JOINT_NUM];
} ArmPositionCommand_t;

typedef struct
{
    uint8_t  sof;
    uint8_t  version;
    uint8_t  type;
    uint8_t  sequence;
    uint16_t payload_len;
    uint32_t timestamp_ms;
} ArmFrameHeader_t;

/* CRC16-CCITT-FALSE, check value for "123456789" is 0x29B1. */
uint16_t arm_proto_crc16(const uint8_t *data, uint16_t len);

/* Build one frame into dst. Returns the frame length, 0 on bad arguments. */
uint16_t arm_proto_build_frame(uint8_t *dst, uint16_t capacity, uint8_t type,
                               uint8_t sequence, uint32_t timestamp_ms,
                               const uint8_t *payload, uint16_t payload_len);

/* Header helpers: the parse function does not check sof / crc. */
void     arm_proto_parse_header(const uint8_t *src, ArmFrameHeader_t *hdr);
uint8_t  arm_proto_frame_crc_ok(const uint8_t *frame, uint16_t payload_len);

/* Payload codecs. encode returns the payload length. */
uint16_t arm_proto_encode_state(uint8_t *dst, const ArmStatePayload_t *s);
void     arm_proto_decode_position_cmd(const uint8_t *src, ArmPositionCommand_t *cmd);
uint16_t arm_proto_encode_ack(uint8_t *dst, uint8_t acked_type, uint8_t acked_seq,
                              uint8_t result);
uint16_t arm_proto_encode_fault(uint8_t *dst, uint16_t fault_code, uint8_t system_state);

/* Saturating float -> fixed point (NaN -> 0). */
int32_t  arm_proto_rad_to_urad(float rad);
int32_t  arm_proto_nm_to_mnm(float nm);
float    arm_proto_urad_to_rad(int32_t urad);

#endif /* __ARM_USB_PROTOCOL_H */
