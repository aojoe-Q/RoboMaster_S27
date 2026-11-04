/**
  ******************************************************************************
  * @file    Arm_USB_task.c
  * @brief   USB CDC link between the arm and ROS2 / MoveIt2.
  ******************************************************************************
  */

#include "Arm_USB_task.h"
#include "ring_buffer.h"
#include "MiniPC.h"
#include "Arm_L_task.h"
#include "can_bsp.h"
#include "multi_turn_restore.h"
#include "FreeRTOS.h"
#include "task.h"

USB_Stats_t usb_stats;

/* ------------------------------------------------------- session state
 *
 * The USB interrupt bumps session_gen at every CDC Init / DeInit and sets
 * session_open accordingly. Bytes are tagged implicitly: the ring is flushed
 * by the task whenever it sees a new generation, and the RX push ignores
 * bytes while no session is open. A frame is only acted upon when the
 * generation is unchanged from before parsing to after the state update,
 * checked inside the same critical section that writes the state.
 */

static RingBuffer_t rx_ring;

static volatile uint32_t session_gen;       /* written by the USB IRQ   */
static volatile uint8_t  session_open;      /* written by the USB IRQ   */
static volatile uint16_t session_rx_start;  /* ring head at session start */
static uint32_t          task_gen;          /* generation the task saw  */

typedef struct
{
    uint8_t  enabled;
    uint8_t  command_valid;
    uint16_t fault_code;
    uint32_t gen;                   /* session the enable belongs to */
    uint32_t last_rx_ms;            /* any valid frame               */
    uint32_t cmd_expire_ms;         /* POSITION_CMD freshness        */
    uint8_t  has_last_cmd;
    uint32_t last_cmd_host_ms;      /* POSITION_CMD.command_timestamp_ms */
    uint32_t host_offset_ms;        /* mcu_ms - host_ms, minimum seen */
    uint32_t offset_creep_ms;       /* last drift adjustment          */
    float    target_rad[ARM_PROTO_JOINT_NUM];
} PcControl_t;

static PcControl_t pc;              /* guarded by a critical section */

typedef struct
{
    uint8_t type;
    uint8_t seq;
    uint8_t result;
} PendingAck_t;

static PendingAck_t ack_queue[ARM_USB_ACK_QUEUE_LEN];
static uint8_t      ack_count;
static uint8_t      fault_pending;

static uint8_t  tx_seq;
static uint8_t  tx_buf[(ARM_USB_ACK_QUEUE_LEN + 2U) * ARM_PROTO_MAX_FRAME];
static uint8_t  rx_frame[ARM_PROTO_MAX_FRAME];

static uint32_t rx_seen_dropped;
static uint8_t  rx_stalled;
static uint32_t rx_stall_since_ms;

/* Joint limits from Arm_L_task.h, same order as the wire arrays. */
static const float joint_max[ARM_PROTO_JOINT_NUM] = {
    J1_Max_position, J2_Max_position, J3_Max_position, J4_Max_position,
    J5_Max_position, J6_Max_position, Grip_Max_position
};
static const float joint_min[ARM_PROTO_JOINT_NUM] = {
    J1_Min_position, J2_Min_position, J3_Min_position, J4_Min_position,
    J5_Min_position, J6_Min_position, Grip_Min_position
};

/* ------------------------------------------------------------- helpers */

static uint8_t time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (uint8_t)((int32_t)(now_ms - deadline_ms) >= 0);
}

static uint8_t system_state(void)
{
    if (pc.enabled)
    {
        return pc.command_valid ? ARM_SYS_RUNNING : ARM_SYS_ENABLED;
    }
    return (pc.fault_code & (ARM_FAULT_LINK_TIMEOUT | ARM_FAULT_USB_DISCONNECT |
                             ARM_FAULT_FEEDBACK_LOST))
           ? ARM_SYS_FAULT : ARM_SYS_IDLE;
}

static void queue_ack(uint8_t type, uint8_t seq, uint8_t result)
{
    if (ack_count >= ARM_USB_ACK_QUEUE_LEN)
    {
        for (uint8_t i = 1; i < ARM_USB_ACK_QUEUE_LEN; i++)
        {
            ack_queue[i - 1U] = ack_queue[i];
        }
        ack_count--;
        usb_stats.tx_ack_dropped++;
    }
    ack_queue[ack_count].type   = type;
    ack_queue[ack_count].seq    = seq;
    ack_queue[ack_count].result = result;
    ack_count++;
}

static void set_fault(uint16_t bits)
{
    if ((pc.fault_code & bits) != bits)
    {
        pc.fault_code |= bits;
        fault_pending = 1U;
    }
}

/* Caller holds the critical section. */
static void disable_locked(void)
{
    pc.enabled       = 0U;
    pc.command_valid = 0U;
}

/* Bit i set when joint i has fed back within ARM_USB_FEEDBACK_TIMEOUT_MS.
   Ages are clamped to 0xFFFE; ARM_FEEDBACK_AGE_NEVER means no feedback yet.
   Caller holds the critical section. */
static uint8_t feedback_mask_locked(uint32_t now_ms, uint16_t age_out[ARM_PROTO_JOINT_NUM])
{
    uint8_t mask = 0U;

    for (uint8_t i = 0; i < ARM_PROTO_JOINT_NUM; i++)
    {
        uint16_t age = ARM_FEEDBACK_AGE_NEVER;

        if (joint_L_motor[i].update_count != 0U)
        {
            int32_t d = (int32_t)(now_ms - joint_L_motor[i].update_tick);
            uint32_t a = d < 0 ? 0U : (uint32_t)d;      /* feedback after now_ms was sampled */
            age = (uint16_t)(a > 0xFFFEU ? 0xFFFEU : a);
            if (a <= ARM_USB_FEEDBACK_TIMEOUT_MS)
            {
                mask |= (uint8_t)(1U << i);
            }
        }
        if (age_out != 0)
        {
            age_out[i] = age;
        }
    }
    return mask;
}

#define ALL_JOINTS_MASK  ((uint8_t)((1U << ARM_PROTO_JOINT_NUM) - 1U))

/* -------------------------------------------------------- frame handlers
 * Each handler decodes and validates without the lock, then commits under
 * the lock only if the USB session is still the one the bytes came from.
 */

static uint8_t handle_position_cmd(const ArmFrameHeader_t *hdr, const uint8_t *payload,
                                   uint32_t now_ms, uint32_t gen)
{
    ArmPositionCommand_t cmd;
    float target[ARM_PROTO_JOINT_NUM];
    uint8_t result = ARM_ACK_OK;

    if (hdr->payload_len != ARM_POS_CMD_PAYLOAD_SIZE)
    {
        return ARM_ACK_BAD_LENGTH;
    }

    arm_proto_decode_position_cmd(payload, &cmd);

    /* Reject the whole command if one joint is outside its limits:
       clamping one joint of a trajectory point distorts the path. */
    for (uint8_t i = 0; i < ARM_PROTO_JOINT_NUM; i++)
    {
        target[i] = arm_proto_urad_to_rad(cmd.position_urad[i]);
        /* PC 目标是逻辑坐标 P = p + b，joint_min/max 是 raw 物理限位，
           先减 b 再比。 */
        if ((target[i] - Joint_Boot_Offset[i]) > joint_max[i] ||
            (target[i] - Joint_Boot_Offset[i]) < joint_min[i])
        {
            return ARM_ACK_OUT_OF_RANGE;
        }
    }

    uint32_t valid_ms = cmd.valid_time_ms;
    if (valid_ms == 0U || valid_ms > ARM_USB_CMD_TIMEOUT_MS)
    {
        valid_ms = ARM_USB_CMD_TIMEOUT_MS;
    }
    if (valid_ms < ARM_USB_CMD_MIN_VALID_MS)
    {
        valid_ms = ARM_USB_CMD_MIN_VALID_MS;
    }

    taskENTER_CRITICAL();
    if (gen != session_gen)
    {
        result = ARM_ACK_NO_SESSION;
    }
    else if (!pc.enabled || pc.gen != gen)
    {
        result = ARM_ACK_NOT_ENABLED;
    }
    else if (pc.has_last_cmd &&
             (int32_t)(cmd.command_timestamp_ms - pc.last_cmd_host_ms) <= 0)
    {
        /* Host timestamps are monotonic per session: older or equal means a
           resend or an out-of-date frame, never a new trajectory point. */
        result = ARM_ACK_STALE;
    }
    else
    {
        /* Queueing delay: (mcu - host) grows when a frame waited in a buffer.
           The minimum seen is the delay of the fastest frame; measure the
           extra delay against it, which needs no clock synchronisation. */
        uint32_t offset = now_ms - cmd.command_timestamp_ms;
        if (!pc.has_last_cmd || (int32_t)(offset - pc.host_offset_ms) < 0)
        {
            pc.host_offset_ms = offset;
            pc.offset_creep_ms = now_ms;
        }
        else if (offset != pc.host_offset_ms &&
                 time_reached(now_ms, pc.offset_creep_ms + ARM_USB_OFFSET_CREEP_MS))
        {
            /* Crystal drift between PC and MCU (~100 ppm = 0.1 ms/s) would
               slowly turn every command stale. Let the baseline follow by
               1 ms per second; a real backlog grows far faster than that. */
            pc.host_offset_ms++;
            pc.offset_creep_ms = now_ms;
        }
        uint32_t queued_ms = offset - pc.host_offset_ms;

        pc.last_cmd_host_ms = cmd.command_timestamp_ms;
        pc.has_last_cmd     = 1U;

        if (queued_ms >= valid_ms)
        {
            result = ARM_ACK_STALE;
        }
        else if (!cmd.enable)
        {
            pc.command_valid = 0U;  /* host asks to hold: drop the target */
        }
        else
        {
            for (uint8_t i = 0; i < ARM_PROTO_JOINT_NUM; i++)
            {
                pc.target_rad[i] = target[i];
            }
            /* Valid time counts from the host send time, as far as the
               offset estimate can tell. */
            pc.cmd_expire_ms = now_ms + (valid_ms - queued_ms);
            pc.command_valid = 1U;
        }
    }
    taskEXIT_CRITICAL();

    return result;
}

static uint8_t handle_enable(const ArmFrameHeader_t *hdr, const uint8_t *payload,
                             uint32_t now_ms, uint32_t gen)
{
    uint8_t result = ARM_ACK_OK;
    uint8_t enabled_now = 0U;

    if (hdr->payload_len != ARM_ENABLE_PAYLOAD_SIZE)
    {
        return ARM_ACK_BAD_LENGTH;
    }
    if (payload[0] > 1U)
    {
        return ARM_ACK_BAD_VALUE;
    }

    taskENTER_CRITICAL();
    if (gen != session_gen || !session_open)
    {
        result = ARM_ACK_NO_SESSION;
    }
    else if (payload[0] == 0U)
    {
        disable_locked();
    }
    else if (ArmData.ArmMode != ARM_MODE_PC_CONTROL)
    {
        /* Arm_L_Task disarms on leaving PC mode; refusing ENABLE outside it
           closes the other path to a stale enable. */
        result = ARM_ACK_WRONG_MODE;
    }
    else if (!Arm_L_Ready ||
             feedback_mask_locked(now_ms, 0) != ALL_JOINTS_MASK)
    {
        result = ARM_ACK_NOT_READY;
    }
    else
    {
        pc.enabled       = 1U;
        pc.command_valid = 0U;      /* a fresh POSITION_CMD is required */
        pc.gen           = gen;
        pc.has_last_cmd  = 0U;      /* new host timeline */
        pc.fault_code    = 0U;      /* ENABLE acknowledges the old faults */
        enabled_now      = 1U;
    }
    taskEXIT_CRITICAL();

    if (enabled_now)
    {
        fault_pending = 1U;         /* report the cleared state */
    }
    return result;
}

/* Host self-test only: lets the test end the USB session at the point an
   interrupt could, between validating a frame and committing it. */
#ifdef ARM_USB_SELFTEST
void arm_usb_selftest_before_commit(uint8_t type);
#define BEFORE_COMMIT(type)  arm_usb_selftest_before_commit(type)
#else
#define BEFORE_COMMIT(type)  ((void)0)
#endif

static void handle_frame(const uint8_t *frame, const ArmFrameHeader_t *hdr,
                         uint32_t now_ms, uint32_t gen)
{
    const uint8_t *payload = &frame[ARM_PROTO_HEADER_SIZE];
    uint8_t result;
    uint8_t ack_always = 1U;

    BEFORE_COMMIT(hdr->type);

    if (hdr->version != ARM_PROTO_VERSION)
    {
        result = ARM_ACK_BAD_VERSION;
    }
    else
    {
        switch (hdr->type)
        {
            case ARM_MSG_POSITION_CMD:
                result = handle_position_cmd(hdr, payload, now_ms, gen);
                ack_always = 0U;    /* 100 Hz stream: only errors are acked */
                break;

            case ARM_MSG_ENABLE:
                result = handle_enable(hdr, payload, now_ms, gen);
                break;

            case ARM_MSG_STOP:
                taskENTER_CRITICAL();
                disable_locked();
                taskEXIT_CRITICAL();
                result = ARM_ACK_OK;
                break;

            case ARM_MSG_HEARTBEAT:
                result = ARM_ACK_OK;
                ack_always = 0U;
                break;

            default:
                result = ARM_ACK_UNKNOWN_TYPE;
                break;
        }
    }

    /* Only a frame of the live session keeps the link alive. */
    if (result != ARM_ACK_NO_SESSION && gen == session_gen)
    {
        pc.last_rx_ms = now_ms;
    }

    if (result != ARM_ACK_OK)
    {
        usb_stats.rx_rejected++;
        if (result == ARM_ACK_STALE)
        {
            usb_stats.rx_stale++;
        }
    }
    if (result == ARM_ACK_NO_SESSION)
    {
        return;                     /* nobody left to answer */
    }
    if (ack_always || result != ARM_ACK_OK)
    {
        queue_ack(hdr->type, hdr->sequence, result);
    }
}

/* ------------------------------------------------------------- RX path */

static void parse_rx(uint32_t now_ms, uint32_t gen)
{
    ArmFrameHeader_t hdr = {0};

    for (;;)
    {
        if (gen != session_gen)
        {
            return;                 /* session changed: next step flushes */
        }

        uint16_t avail = ring_buffer_available(&rx_ring);

        if (avail == 0U)
        {
            rx_stalled = 0U;
            return;
        }

        /* Hunt for SOF one byte at a time. */
        ring_buffer_peek(&rx_ring, rx_frame, 1U);
        if (rx_frame[0] != ARM_PROTO_SOF)
        {
            ring_buffer_skip(&rx_ring, 1U);
            usb_stats.rx_resync_bytes++;
            rx_stalled = 0U;
            continue;
        }

        uint16_t frame_len = ARM_PROTO_HEADER_SIZE;
        if (avail >= ARM_PROTO_HEADER_SIZE)
        {
            ring_buffer_peek(&rx_ring, rx_frame, ARM_PROTO_HEADER_SIZE);
            arm_proto_parse_header(rx_frame, &hdr);

            if (hdr.payload_len > ARM_PROTO_MAX_PAYLOAD)
            {
                ring_buffer_skip(&rx_ring, 1U);     /* false SOF */
                usb_stats.rx_resync_bytes++;
                rx_stalled = 0U;
                continue;
            }
            frame_len = (uint16_t)(ARM_PROTO_OVERHEAD + hdr.payload_len);
        }

        if (avail < frame_len)
        {
            /* Wait for the rest, but not forever: a false SOF with a big
               length would otherwise hide the real frames behind it. */
            if (!rx_stalled)
            {
                rx_stalled = 1U;
                rx_stall_since_ms = now_ms;
            }
            else if (time_reached(now_ms, rx_stall_since_ms + ARM_USB_RX_STALL_MS))
            {
                ring_buffer_skip(&rx_ring, 1U);
                usb_stats.rx_resync_bytes++;
                rx_stalled = 0U;
                continue;
            }
            return;
        }

        rx_stalled = 0U;
        ring_buffer_peek(&rx_ring, rx_frame, frame_len);

        if (!arm_proto_frame_crc_ok(rx_frame, hdr.payload_len))
        {
            ring_buffer_skip(&rx_ring, 1U);
            usb_stats.rx_crc_errors++;
            continue;
        }

        ring_buffer_skip(&rx_ring, frame_len);
        usb_stats.rx_frames++;
        handle_frame(rx_frame, &hdr, now_ms, gen);
    }
}

/* -------------------------------------------------------- safety checks */

/* Runs first: a new session generation discards everything older. */
static uint32_t check_session(void)
{
    uint32_t gen;
    uint8_t  open;
    uint16_t start;
    uint8_t  dropped = 0U;

    /* gen, open and session_rx_start are read together: the USB IRQ writes
       them together and is masked here. */
    taskENTER_CRITICAL();
    gen   = session_gen;
    open  = session_open;
    start = session_rx_start;
    if (pc.enabled && (pc.gen != gen || !open || !MiniPC_Is_Connected()))
    {
        /* Ended, replaced or suspended (suspend keeps the CDC class but
           leaves CONFIGURED). Re-enabling needs a new ENABLE. */
        disable_locked();
        dropped = 1U;
    }
    taskEXIT_CRITICAL();

    if (dropped)
    {
        set_fault(ARM_FAULT_USB_DISCONNECT);
    }

    if (gen != task_gen)
    {
        /* Drop the bytes of earlier sessions, keep what the new session has
           already sent: session_rx_start is the ring head captured when the
           session opened, so everything before it is old. If the session
           also ended again, the push side refused new bytes, so the ring
           holds nothing worth keeping. */
        if (open)
        {
            /* If the parser already consumed past `start` (a new session
               opened mid-parse; those bytes were refused as NO_SESSION),
               this is a no-op and the host retries ENABLE. */
            ring_buffer_discard_until(&rx_ring, start);
        }
        else
        {
            ring_buffer_flush(&rx_ring);
        }
        rx_stalled = 0U;
        ack_count  = 0U;            /* answers to the old session */
        task_gen   = gen;
    }
    return gen;
}

/* Runs after parse_rx so frames of this period count as link activity. */
static void check_timeouts(uint32_t now_ms)
{
    uint8_t timed_out = 0U;
    uint8_t feedback_lost = 0U;

    taskENTER_CRITICAL();
    if (pc.enabled && time_reached(now_ms, pc.last_rx_ms + ARM_USB_LINK_TIMEOUT_MS))
    {
        disable_locked();
        timed_out = 1U;
    }
    /* Runtime CAN watchdog: a joint silent for ARM_USB_FEEDBACK_TIMEOUT_MS
       disables PC control, so the old target cannot resume by itself when
       the feedback comes back. Arm_L_Task holds the position meanwhile. */
    if (pc.enabled && feedback_mask_locked(now_ms, 0) != ALL_JOINTS_MASK)
    {
        disable_locked();
        feedback_lost = 1U;
    }
    if (pc.command_valid && time_reached(now_ms, pc.cmd_expire_ms))
    {
        pc.command_valid = 0U;
    }
    taskEXIT_CRITICAL();

    if (timed_out)
    {
        set_fault(ARM_FAULT_LINK_TIMEOUT);
    }
    if (feedback_lost)
    {
        set_fault(ARM_FAULT_FEEDBACK_LOST);
    }

    uint32_t dropped = rx_ring.dropped;
    if (dropped != rx_seen_dropped)
    {
        usb_stats.rx_overflow_bytes += dropped - rx_seen_dropped;
        rx_seen_dropped = dropped;
        set_fault(ARM_FAULT_RX_OVERFLOW);
    }
}

/* ------------------------------------------------------------- TX path */

static void fill_state(ArmStatePayload_t *s, uint32_t now_ms)
{
    const DM_contorl_t *joint[ARM_PROTO_JOINT_NUM] = {
        &ArmData.J1, &ArmData.J2, &ArmData.J3, &ArmData.J4,
        &ArmData.J5, &ArmData.J6, &ArmData.Grip
    };
    float pos[ARM_PROTO_JOINT_NUM], vel[ARM_PROTO_JOINT_NUM];
    float tor[ARM_PROTO_JOINT_NUM], set[ARM_PROTO_JOINT_NUM];
    uint8_t mode, ready, mask;

    /* One critical section: position / velocity / torque all come from the
       same CAN feedback sample of each motor (written by the FDCAN RX
       interrupt), the target from the controller output (Arm_L_Task). */
    taskENTER_CRITICAL();
    mode  = ArmData.ArmMode;
    ready = Arm_L_Ready;
    mask  = feedback_mask_locked(now_ms, s->feedback_age_ms);
    for (uint8_t i = 0; i < ARM_PROTO_JOINT_NUM; i++)
    {
        /* Logical position P = p + b: same coordinates as the PC targets
           and the joint limits. */
        pos[i] = joint_L_motor[i].position + Joint_Boot_Offset[i];
        vel[i] = joint_L_motor[i].velocity;
        tor[i] = joint_L_motor[i].torque;
        set[i] = joint[i]->send_set;
    }
    taskEXIT_CRITICAL();

    s->timestamp_ms   = now_ms;
    s->arm_mode       = mode;
    s->system_state   = system_state();
    s->fault_code     = pc.fault_code;
    s->flags          = ready ? ARM_STATE_FLAG_ARM_READY : 0U;
    s->feedback_valid = mask;
    for (uint8_t i = 0; i < ARM_PROTO_JOINT_NUM; i++)
    {
        s->position_urad[i]        = arm_proto_rad_to_urad(pos[i]);
        s->velocity_urad_s[i]      = arm_proto_rad_to_urad(vel[i]);
        s->torque_mNm[i]           = arm_proto_nm_to_mnm(tor[i]);
        s->target_position_urad[i] = arm_proto_rad_to_urad(set[i]);
    }
}

static void send_tx(uint32_t now_ms)
{
    uint8_t payload[ARM_PROTO_MAX_PAYLOAD];
    ArmStatePayload_t state;
    uint16_t len = 0;
    uint16_t n;

    if (!session_open || !MiniPC_Is_Connected())
    {
        ack_count = 0U;             /* nobody to answer */
        return;
    }
    if (MiniPC_Tx_Busy())
    {
        usb_stats.tx_busy++;        /* tx_buf is still being sent */
        return;
    }

    /* ACK and FAULT first, then the STATE frame, in one CDC transfer. */
    for (uint8_t i = 0; i < ack_count; i++)
    {
        n = arm_proto_encode_ack(payload, ack_queue[i].type, ack_queue[i].seq,
                                 ack_queue[i].result);
        len += arm_proto_build_frame(&tx_buf[len], (uint16_t)(sizeof(tx_buf) - len),
                                     ARM_MSG_ACK, tx_seq++, now_ms, payload, n);
    }

    if (fault_pending)
    {
        n = arm_proto_encode_fault(payload, pc.fault_code, system_state());
        len += arm_proto_build_frame(&tx_buf[len], (uint16_t)(sizeof(tx_buf) - len),
                                     ARM_MSG_FAULT, tx_seq++, now_ms, payload, n);
    }

    fill_state(&state, now_ms);
    n = arm_proto_encode_state(payload, &state);
    len += arm_proto_build_frame(&tx_buf[len], (uint16_t)(sizeof(tx_buf) - len),
                                 ARM_MSG_STATE, tx_seq++, now_ms, payload, n);

    /* MiniPC_Transmit_Info re-checks the class pointer with interrupts
       masked, so a reset between the checks above and here is harmless. */
    if (MiniPC_Transmit_Info(tx_buf, len) == MINIPC_TX_OK)
    {
        usb_stats.tx_frames += (uint32_t)ack_count + fault_pending + 1U;
        ack_count     = 0U;
        fault_pending = 0U;
    }
    else
    {
        usb_stats.tx_busy++;        /* keep ACK / FAULT for the next period */
    }
}

/* ---------------------------------------------------------- public API */

void Arm_USB_OnSessionStart(void)
{
    session_rx_start = rx_ring.head;    /* same IRQ as RxPush: no race */
    session_gen++;
    session_open = 1U;
}

void Arm_USB_OnSessionEnd(void)
{
    session_gen++;
    session_open = 0U;
}

void Arm_USB_RxPush(const uint8_t *data, uint32_t len)
{
    if (!session_open)
    {
        return;                     /* no session: nothing may be queued */
    }
    if (len > 0xFFFFU)
    {
        rx_ring.dropped += len;
        return;
    }
    (void)ring_buffer_write_bytes(&rx_ring, data, (uint16_t)len);
}

void Arm_USB_Init(void)
{
    taskENTER_CRITICAL();
    disable_locked();
    pc.fault_code   = 0U;
    pc.has_last_cmd = 0U;
    pc.gen          = 0U;
    task_gen        = session_gen;
    taskEXIT_CRITICAL();

    ack_count       = 0U;
    fault_pending   = 0U;
    tx_seq          = 0U;
    rx_stalled      = 0U;
    rx_seen_dropped = rx_ring.dropped;
    ring_buffer_flush(&rx_ring);
}

void Arm_USB_Step(uint32_t now_ms)
{
    uint32_t gen = check_session();
    parse_rx(now_ms, gen);
    /* Again: a USB reset during parse_rx must be applied (disable, fault,
       drop old bytes) before this period's STATE reports the state. */
    (void)check_session();
    check_timeouts(now_ms);
    send_tx(now_ms);
}

void Arm_USB_PC_Disarm(void)
{
    taskENTER_CRITICAL();
    disable_locked();
    taskEXIT_CRITICAL();
}

uint8_t Arm_USB_PC_Enabled(void)
{
    uint8_t on;

    taskENTER_CRITICAL();
    on = (uint8_t)(pc.enabled && session_open && pc.gen == session_gen &&
                   MiniPC_Is_Connected());
    taskEXIT_CRITICAL();
    return on;
}

uint8_t Arm_USB_Get_PC_Target(float target_rad[ARM_PROTO_JOINT_NUM])
{
    uint8_t ok;
    uint32_t now_ms = (uint32_t)xTaskGetTickCount();

    if (target_rad == 0)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    /* Re-check everything here: the USB task runs only every 10 ms, and a
       USB reset may have ended the session since. Also check connection and
       feedback validity (runtime CAN watchdog). */
    ok = (uint8_t)(pc.enabled && pc.command_valid &&
                   session_open && pc.gen == session_gen &&
                   MiniPC_Is_Connected() &&
                   feedback_mask_locked(now_ms, 0) == ALL_JOINTS_MASK &&
                   !time_reached(now_ms, pc.cmd_expire_ms));
    if (ok)
    {
        for (uint8_t i = 0; i < ARM_PROTO_JOINT_NUM; i++)
        {
            target_rad[i] = pc.target_rad[i];
        }
    }
    taskEXIT_CRITICAL();

    return ok;
}

void Arm_USB_Task(void const *argument)
{
    (void)argument;

    Arm_USB_Init();
    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        Arm_USB_Step((uint32_t)xTaskGetTickCount());
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ARM_USB_TASK_PERIOD_MS));
    }
}
