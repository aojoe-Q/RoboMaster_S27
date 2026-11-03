/* Host test of APP/Arm_USB_task.c + arm_usb_protocol.c + ring_buffer.c.
 *
 * The firmware sources are compiled unchanged against the stubs in stub/.
 * This program fakes the clock, the USB session callbacks and the CAN
 * feedback, then:
 *   - runs the C checks below,
 *   - writes dev_state.bin (STATE frames) for run_selftest.py to decode,
 *   - replays host_frames.bin (built by Python) and writes cross_out.bin.
 *
 * Usage: usb_selftest <work_dir>
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "Arm_L_task.h"
#include "can_bsp.h"
#include "task.h"
#include "MiniPC.h"
#include "Arm_USB_task.h"
#include "arm_usb_protocol.h"
#include "ring_buffer.h"

/* ------------------------------------------------------------ fakes */

ArmData_t ArmData;
volatile uint8_t Arm_L_Ready = 1;
joint_motor_measure_t joint_L_motor[7];

static uint32_t fake_ms;
static bool     fake_connected = true;
static bool     fake_busy;
static uint8_t  sent[1U << 21];   /* drift test alone sends ~150 KB */
static uint32_t sent_len;

TickType_t xTaskGetTickCount(void) { return fake_ms; }
void vTaskDelayUntil(TickType_t *prev, TickType_t inc) { *prev += inc; }

bool MiniPC_Is_Connected(void) { return fake_connected; }
bool MiniPC_Tx_Busy(void) { return fake_busy; }

uint8_t MiniPC_Transmit_Info(const uint8_t *buf, uint16_t len)
{
    if (!fake_connected || sent_len + len > sizeof(sent))
    {
        return 2U;
    }
    memcpy(&sent[sent_len], buf, len);
    sent_len += len;
    return MINIPC_TX_OK;
}

/* USB plug cycle as the ST stack does it: DeInit, then Init. */
static void usb_unplug(void) { fake_connected = false; Arm_USB_OnSessionEnd(); }
static void usb_plug(void)   { fake_connected = true;  Arm_USB_OnSessionStart(); }

/* Firmware hook (ARM_USB_SELFTEST): replug once, in the middle of parsing,
   right before a frame of this type is handled. */
static uint8_t replug_before_type;

void arm_usb_selftest_before_commit(uint8_t type)
{
    if (replug_before_type != 0U && type == replug_before_type)
    {
        replug_before_type = 0U;
        usb_unplug();
        usb_plug();
    }
}

/* Every joint fed back at fake_ms (as the CAN RX interrupt would). */
static void feedback_now(void)
{
    for (int i = 0; i < 7; i++)
    {
        joint_L_motor[i].update_tick = fake_ms;
        joint_L_motor[i].update_count++;
    }
}

/* ---------------------------------------------------------- helpers */

static int failures;

#define CHECK(cond, ...)                                              \
    do {                                                              \
        if (!(cond)) {                                                \
            failures++;                                               \
            printf("FAIL line %d: ", __LINE__);                       \
            printf(__VA_ARGS__);                                      \
            printf("\n");                                             \
        }                                                             \
    } while (0)

typedef struct
{
    int      frames, states, faults, acks;
    uint8_t  ack_type[16], ack_seq[16], ack_result[16];
    uint8_t  sys_state;     /* of the last STATE */
    uint16_t fault_code;    /* of the last STATE */
    uint8_t  flags;
    uint8_t  fb_valid;
    uint16_t fb_age[7];
    int32_t  torque[7];
    uint16_t fault_frame_code;
    int      bad;
} Output;

static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static int32_t  rd32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
                     ((uint32_t)p[3] << 24));
}

static Output collect(uint32_t from)
{
    Output o;
    memset(&o, 0, sizeof(o));

    uint32_t pos = from;
    while (pos < sent_len)
    {
        ArmFrameHeader_t h;
        if (sent_len - pos < ARM_PROTO_OVERHEAD || sent[pos] != ARM_PROTO_SOF)
        {
            o.bad++;
            break;
        }
        arm_proto_parse_header(&sent[pos], &h);
        if (!arm_proto_frame_crc_ok(&sent[pos], h.payload_len))
        {
            o.bad++;
            break;
        }
        const uint8_t *p = &sent[pos + ARM_PROTO_HEADER_SIZE];
        o.frames++;
        if (h.type == ARM_MSG_STATE)
        {
            o.states++;
            o.sys_state  = p[5];
            o.fault_code = rd16(&p[6]);
            o.flags      = p[8];
            o.fb_valid   = p[9];
            for (int i = 0; i < 7; i++)
            {
                o.fb_age[i] = rd16(&p[12 + 2 * i]);
                o.torque[i] = rd32(&p[12 + 14 + 28 + 28 + 4 * i]);
            }
        }
        else if (h.type == ARM_MSG_ACK && o.acks < 16)
        {
            o.ack_type[o.acks]   = p[0];
            o.ack_seq[o.acks]    = p[1];
            o.ack_result[o.acks] = p[2];
            o.acks++;
        }
        else if (h.type == ARM_MSG_FAULT)
        {
            o.faults++;
            o.fault_frame_code = rd16(p);
        }
        pos += ARM_PROTO_OVERHEAD + h.payload_len;
    }
    return o;
}

/* Advance the clock, keep the motors alive, run one task period. */
static Output step(uint32_t dt_ms)
{
    uint32_t mark = sent_len;
    fake_ms += dt_ms;
    feedback_now();
    Arm_USB_Step(fake_ms);
    return collect(mark);
}

/* Same, but the motors stay silent. */
static Output step_no_feedback(uint32_t dt_ms)
{
    uint32_t mark = sent_len;
    fake_ms += dt_ms;
    Arm_USB_Step(fake_ms);
    return collect(mark);
}

static uint8_t  host_seq;
static uint32_t host_off = 50000U;  /* host monotonic clock = fake_ms + host_off */
static uint32_t host_last_ts;

/* Host timestamp "now", strictly increasing as HostEncoder guarantees. */
static uint32_t host_now(void)
{
    uint32_t ts = fake_ms + host_off;
    if ((int32_t)(ts - host_last_ts) <= 0)
    {
        ts = host_last_ts + 1U;
    }
    host_last_ts = ts;
    return ts;
}

static uint16_t host_frame(uint8_t *dst, uint8_t type, const uint8_t *payload, uint16_t len)
{
    return arm_proto_build_frame(dst, ARM_PROTO_MAX_FRAME, type, host_seq++, fake_ms + host_off,
                                 payload, len);
}

static void put32(uint8_t *p, int32_t v)
{
    uint32_t u = (uint32_t)v;
    p[0] = (uint8_t)u; p[1] = (uint8_t)(u >> 8); p[2] = (uint8_t)(u >> 16); p[3] = (uint8_t)(u >> 24);
}

/* POSITION_CMD with an explicit host timestamp. */
static uint16_t host_pos_cmd_at(uint8_t *dst, const float rad[7], uint16_t valid_ms,
                                uint8_t enable, uint32_t ts)
{
    uint8_t pl[ARM_POS_CMD_PAYLOAD_SIZE];
    memset(pl, 0, sizeof(pl));
    put32(&pl[0], (int32_t)ts);
    pl[4] = (uint8_t)valid_ms;
    pl[5] = (uint8_t)(valid_ms >> 8);
    pl[6] = enable;
    for (int i = 0; i < 7; i++)
    {
        put32(&pl[8 + 4 * i], (int32_t)lroundf(rad[i] * 1e6f));
    }
    return host_frame(dst, ARM_MSG_POSITION_CMD, pl, sizeof(pl));
}

static void push(const uint8_t *data, uint32_t len) { Arm_USB_RxPush(data, len); }

static void send_simple(uint8_t type, const uint8_t *payload, uint16_t len)
{
    uint8_t f[ARM_PROTO_MAX_FRAME];
    push(f, host_frame(f, type, payload, len));
}

static void send_enable(uint8_t on)
{
    uint8_t pl[4] = { on, 0, 0, 0 };
    send_simple(ARM_MSG_ENABLE, pl, 4);
}

static void send_heartbeat(void) { send_simple(ARM_MSG_HEARTBEAT, 0, 0); }

/* Normal stream: stamped now, processed at the next step. */
static void send_pos(const float rad[7], uint16_t valid_ms)
{
    uint8_t f[ARM_PROTO_MAX_FRAME];
    push(f, host_pos_cmd_at(f, rad, valid_ms, 1, host_now()));
}

static bool same(const float a[7], const float b[7])
{
    for (int i = 0; i < 7; i++)
    {
        if (fabsf(a[i] - b[i]) > 2e-6f)
        {
            return false;
        }
    }
    return true;
}

/* Fresh session, enabled, host timeline aligned with fake_ms. */
static void fresh_enabled_session(void)
{
    usb_unplug();
    (void)step(10);
    usb_plug();
    (void)step(10);
    send_enable(1);
    (void)step(10);
}

static void fill_arm(void)
{
    DM_contorl_t *j[7] = { &ArmData.J1, &ArmData.J2, &ArmData.J3, &ArmData.J4,
                           &ArmData.J5, &ArmData.J6, &ArmData.Grip };
    ArmData.ArmMode = 8;    /* ARM_RESET */
    for (int i = 0; i < 7; i++)
    {
        j[i]->position = 99.0f;                     /* must NOT be sent */
        j[i]->send_set = -0.05f * (float)(i + 1);
        joint_L_motor[i].position = 0.1f * (float)(i + 1) - 0.25f;
        joint_L_motor[i].velocity = 1.5f - 0.5f * (float)i;
        joint_L_motor[i].torque   = 0.25f * (float)i - 0.8f;
    }
    joint_L_motor[6].torque = -48.5f;               /* beyond int16 mNm */
}

static bool write_file(const char *dir, const char *name, const uint8_t *data, uint32_t len)
{
    char path[512];
    snprintf(path, sizeof(path), "%s/%s", dir, name);
    FILE *f = fopen(path, "wb");
    if (!f)
    {
        return false;
    }
    bool ok = fwrite(data, 1, len, f) == len;
    fclose(f);
    return ok;
}

/* --------------------------------------------------------------- tests */

int main(int argc, char **argv)
{
    const char *dir = argc > 1 ? argv[1] : ".";
    const float ok_pos[7]  = { 0.10f, 0.50f, 0.60f, -1.0f, 0.20f, -0.30f, 0.35f };
    const float ok_pos2[7] = { -0.20f, 1.00f, 1.20f, 2.0f, -0.50f, 1.00f, 0.10f };
    uint8_t frame[ARM_PROTO_MAX_FRAME];
    uint16_t flen;
    float got[7];
    Output o;

    CHECK(arm_proto_crc16((const uint8_t *)"123456789", 9) == 0x29B1U, "crc check value");

    fill_arm();
    fake_ms = 1000;
    Arm_USB_Init();
    usb_plug();

    /* 1. No session yet seen by the task: one STATE, IDLE, feedback valid. */
    uint32_t state_mark = sent_len;
    o = step(10);
    CHECK(o.frames == 1 && o.states == 1 && o.bad == 0, "one STATE per period, got %d", o.frames);
    CHECK(o.sys_state == ARM_SYS_IDLE, "IDLE after reset, got %d", o.sys_state);
    CHECK(o.fb_valid == 0x7F && o.flags == ARM_STATE_FLAG_ARM_READY, "feedback / ready flags");
    CHECK(o.torque[6] == -48500, "torque int32, got %d", (int)o.torque[6]);
    (void)step(10);
    if (!write_file(dir, "dev_state.bin", &sent[state_mark], sent_len - state_mark))
    {
        printf("FAIL cannot write dev_state.bin\n");
        return 1;
    }
    CHECK(!Arm_USB_Get_PC_Target(got), "no PC target while idle");

    /* 1b. ENABLE outside PC mode (RC switch elsewhere) is refused. */
    send_enable(1);
    o = step(10);
    CHECK(o.acks == 1 && o.ack_result[0] == ARM_ACK_WRONG_MODE && !Arm_USB_PC_Enabled(),
          "ENABLE refused while arm_mode != PC_CONTROL");
    ArmData.ArmMode = ARM_MODE_PC_CONTROL;          /* both DT7 switches up */

    /* 2. POSITION_CMD before ENABLE is refused. */
    send_pos(ok_pos, 100);
    o = step(10);
    CHECK(o.acks == 1 && o.ack_result[0] == ARM_ACK_NOT_ENABLED, "NOT_ENABLED ack");

    /* 3. ENABLE refused while Arm_L_Init is running, or a joint is silent. */
    Arm_L_Ready = 0;
    send_enable(1);
    o = step(10);
    CHECK(o.acks == 1 && o.ack_result[0] == ARM_ACK_NOT_READY && o.flags == 0, "NOT_READY: init");
    Arm_L_Ready = 1;
    joint_L_motor[3].update_tick = fake_ms - 150U;
    send_enable(1);
    o = step_no_feedback(10);
    CHECK(o.acks == 1 && o.ack_result[0] == ARM_ACK_NOT_READY &&
          o.fb_valid == (0x7F & ~(1U << 3)) && o.fb_age[3] >= 150U, "NOT_READY: J4 silent");

    /* 4. ENABLE ok -> FAULT(0) report, ENABLED. */
    send_enable(1);
    o = step(10);
    CHECK(o.acks == 1 && o.ack_type[0] == ARM_MSG_ENABLE && o.ack_result[0] == ARM_ACK_OK, "ENABLE ack");
    CHECK(o.faults == 1 && o.fault_frame_code == 0, "FAULT frame reports cleared faults");
    CHECK(o.sys_state == ARM_SYS_ENABLED, "ENABLED, got %d", o.sys_state);

    /* 5. Valid POSITION_CMD: no ACK on success, RUNNING, target readable. */
    send_pos(ok_pos, 100);
    o = step(10);
    CHECK(o.acks == 0 && o.sys_state == ARM_SYS_RUNNING, "accepted, RUNNING");
    CHECK(Arm_USB_Get_PC_Target(got) && same(got, ok_pos), "target matches the command");

    /* 6. Replayed old command (older timestamp, new sequence) is STALE. */
    uint32_t old_ts = host_last_ts;
    send_pos(ok_pos2, 100);
    (void)step(10);
    flen = host_pos_cmd_at(frame, ok_pos, 100, 1, old_ts);
    push(frame, flen);
    o = step(10);
    CHECK(o.acks == 1 && o.ack_result[0] == ARM_ACK_STALE, "replayed command STALE");
    CHECK(Arm_USB_Get_PC_Target(got) && same(got, ok_pos2), "replay did not overwrite");
    flen = host_pos_cmd_at(frame, ok_pos, 100, 1, host_last_ts);   /* equal ts */
    push(frame, flen);
    o = step(10);
    CHECK(o.acks == 1 && o.ack_result[0] == ARM_ACK_STALE, "equal timestamp STALE");

    /* 7. Expiry counts from the send time: a command queued 60 ms in a buffer
          with valid 100 ms lives 40 ms more; one queued 120 ms is STALE. */
    send_pos(ok_pos, 100);
    o = step(0);                                    /* zero delay: baseline */
    fake_ms += 10U;
    flen = host_pos_cmd_at(frame, ok_pos2, 100, 1, host_now());
    fake_ms += 60U;                                 /* it sat in a buffer */
    push(frame, flen);
    o = step(0);
    CHECK(o.acks == 0 && Arm_USB_Get_PC_Target(got) && same(got, ok_pos2), "queued 60 ms accepted");
    fake_ms += 30U; feedback_now();
    CHECK(Arm_USB_Get_PC_Target(got), "still valid 30 ms later");
    fake_ms += 20U; feedback_now();
    CHECK(!Arm_USB_Get_PC_Target(got), "expired 40 ms after arrival (100 - 60)");
    flen = host_pos_cmd_at(frame, ok_pos, 100, 1, host_now());
    fake_ms += 130U;
    send_heartbeat();
    push(frame, flen);
    o = step(0);
    CHECK(o.acks == 1 && o.ack_result[0] == ARM_ACK_STALE && usb_stats.rx_stale >= 3,
          "queued past its valid time STALE");

    /* 8. Clock drift: host clock 200 ppm slow. Over 10 min that is 120 ms,
          more than the 50 ms valid time: without drift tracking every
          command would end up STALE. */
    fresh_enabled_session();
    uint32_t stale_before = usb_stats.rx_stale;
    double drift_host = (double)host_now();
    for (int i = 0; i < 60000; i++)                 /* 10 min at 100 Hz */
    {
        drift_host += 10.0 * (1.0 - 200e-6);
        flen = host_pos_cmd_at(frame, ok_pos, 50, 1, (uint32_t)drift_host);
        push(frame, flen);
        uint32_t mark = sent_len;
        (void)step(10);
        sent_len = mark;                            /* do not keep 60000 STATE frames */
    }
    CHECK(usb_stats.rx_stale == stale_before && Arm_USB_Get_PC_Target(got),
          "200 ppm drift over 10 min: no stale, %u", (unsigned)(usb_stats.rx_stale - stale_before));
    host_off = (uint32_t)drift_host - fake_ms + 1U;     /* continue from the drifted clock */

    /* 9. Out of range rejects the whole command, old target kept. */
    float bad[7];
    memcpy(bad, ok_pos2, sizeof(bad));
    bad[1] = J2_Max_position + 0.01f;
    send_pos(bad, 300);
    o = step(10);
    CHECK(o.acks == 1 && o.ack_result[0] == ARM_ACK_OUT_OF_RANGE, "OUT_OF_RANGE ack");
    CHECK(Arm_USB_Get_PC_Target(got) && same(got, ok_pos), "previous target kept");

    /* 10. Bad CRC, junk, byte-wise delivery, stalled false header. */
    flen = host_pos_cmd_at(frame, ok_pos2, 300, 1, host_now());
    frame[20] ^= 0x01U;
    push(frame, flen);
    o = step(10);
    CHECK(o.acks == 0 && usb_stats.rx_crc_errors >= 1, "bad CRC counted, no ACK");
    for (int i = 0; i < 3; i++)
    {
        send_heartbeat();
        (void)step(25);
    }
    const uint8_t junk[] = { 0x00, 0xA5, 0xFF, 0xA5, 0x02, 0x02, 0x00, 0x7F, 0x00, 0x13,
                             0xA5, 0x02, 0x05, 0x00, 0x40, 0x00, 0x11, 0x22, 0x33, 0x44 };
    push(junk, sizeof(junk));
    flen = host_pos_cmd_at(frame, ok_pos2, 300, 1, host_now());
    for (uint16_t i = 0; i < flen; i++)
    {
        push(&frame[i], 1);
        if (i % 7U == 0U)
        {
            (void)step(1);
        }
    }
    o = step(25);
    CHECK(o.bad == 0 && Arm_USB_Get_PC_Target(got) && same(got, ok_pos2),
          "recovered after junk and byte-wise transfer");

    /* 11. Validation ACKs. */
    uint8_t two[2] = { 1, 0 };
    send_simple(ARM_MSG_ENABLE, two, 2);
    uint8_t enable_bad[4] = { 7, 0, 0, 0 };
    send_simple(ARM_MSG_ENABLE, enable_bad, 4);
    send_simple(0x55, 0, 0);
    flen = host_frame(frame, ARM_MSG_HEARTBEAT, 0, 0);
    frame[1] = 0x09;
    uint16_t crc = arm_proto_crc16(frame, ARM_PROTO_HEADER_SIZE);
    frame[10] = (uint8_t)crc;
    frame[11] = (uint8_t)(crc >> 8);
    push(frame, flen);
    o = step(10);
    CHECK(o.acks == 4 &&
          o.ack_result[0] == ARM_ACK_BAD_LENGTH && o.ack_result[1] == ARM_ACK_BAD_VALUE &&
          o.ack_result[2] == ARM_ACK_UNKNOWN_TYPE && o.ack_result[3] == ARM_ACK_BAD_VERSION,
          "validation ACKs, got %d", o.acks);
    CHECK(Arm_USB_PC_Enabled(), "rejected ENABLE frames do not change the state");

    /* 12. CDC busy keeps ACKs; overflow keeps the newest. */
    fake_busy = true;
    for (int i = 0; i < (int)ARM_USB_ACK_QUEUE_LEN + 2; i++)
    {
        send_simple(0x55, 0, 0);
    }
    uint8_t last_seq = (uint8_t)(host_seq - 1U);
    o = step(10);
    CHECK(o.frames == 0 && usb_stats.tx_busy >= 1, "nothing sent while busy");
    fake_busy = false;
    o = step(10);
    CHECK(o.acks == (int)ARM_USB_ACK_QUEUE_LEN && o.ack_seq[ARM_USB_ACK_QUEUE_LEN - 1U] == last_seq,
          "ACK queue drains, oldest dropped");

    /* 13. Link timeout. */
    o = step(280);
    CHECK(Arm_USB_PC_Enabled(), "still enabled before the link timeout");
    o = step(10);
    CHECK(!Arm_USB_PC_Enabled() && o.sys_state == ARM_SYS_FAULT &&
          (o.fault_code & ARM_FAULT_LINK_TIMEOUT) && o.faults == 1, "link timeout -> FAULT");

    /* 14. STOP. */
    fresh_enabled_session();
    send_pos(ok_pos, 300);
    (void)step(10);
    send_simple(ARM_MSG_STOP, 0, 0);
    o = step(10);
    CHECK(o.acks == 1 && o.ack_type[0] == ARM_MSG_STOP && !Arm_USB_PC_Enabled() &&
          !Arm_USB_Get_PC_Target(got) && o.sys_state == ARM_SYS_IDLE, "STOP -> IDLE");

    /* 15. Review issue 1: ENABLE + POSITION_CMD still queued when the
          session ends must not re-enable within the same task step. */
    fresh_enabled_session();
    send_pos(ok_pos, 300);
    (void)step(10);
    CHECK(Arm_USB_Get_PC_Target(got), "running before the unplug");
    send_enable(1);
    send_pos(ok_pos2, 300);                         /* queued, not parsed yet */
    usb_unplug();
    CHECK(!Arm_USB_Get_PC_Target(got), "target unavailable right after DeInit, before the task runs");
    uint32_t rx_before_stale = usb_stats.rx_frames;
    o = step(0);
    CHECK(!Arm_USB_PC_Enabled() && !Arm_USB_Get_PC_Target(got) && o.frames == 0 &&
          usb_stats.rx_frames == rx_before_stale,
          "stale ENABLE/POSITION not parsed after session end, enabled %d", Arm_USB_PC_Enabled());
    uint32_t rx_before_unplugged = usb_stats.rx_frames;
    send_enable(1);                                 /* bytes while unplugged */
    o = step(10);
    CHECK(usb_stats.rx_frames == rx_before_unplugged && !Arm_USB_PC_Enabled(),
          "bytes received with no session are refused");

    /* 16. Same, with a fast replug before the task runs: old bytes dropped,
          the new session's first ENABLE kept. */
    usb_plug();
    o = step(10);
    CHECK(o.states == 1 && (o.fault_code & ARM_FAULT_USB_DISCONNECT) &&
          o.sys_state == ARM_SYS_FAULT, "USB_DISCONNECT reported after reconnect");
    send_enable(1);
    send_pos(ok_pos2, 300);
    usb_unplug();
    usb_plug();                                     /* both before the task step */
    send_enable(1);                                 /* new session, first frame */
    o = step(10);
    CHECK(o.acks == 1 && o.ack_type[0] == ARM_MSG_ENABLE && o.ack_result[0] == ARM_ACK_OK &&
          Arm_USB_PC_Enabled() && !Arm_USB_Get_PC_Target(got),
          "replug: old ENABLE/POSITION dropped, new ENABLE kept (acks %d)", o.acks);

    /* 17. Session ends between the task parsing a frame and committing it:
          simulated by ending it while the task has already sampled gen. */
    send_pos(ok_pos, 300);
    (void)step(10);
    CHECK(Arm_USB_Get_PC_Target(got), "running in the new session");
    usb_unplug();
    usb_plug();
    CHECK(!Arm_USB_Get_PC_Target(got), "target dropped by a session change alone");
    o = step(10);
    CHECK(!Arm_USB_PC_Enabled() && (o.fault_code & ARM_FAULT_USB_DISCONNECT),
          "reconnect requires a new ENABLE");

    /* 17b. The session ends (and a new one opens) while the task is inside
           parse_rx, between validating a frame and committing it. Neither
           this frame nor the ones behind it may take effect. */
    usb_plug();
    (void)step(10);
    send_enable(1);
    (void)step(10);
    CHECK(Arm_USB_PC_Enabled(), "enabled before the mid-parse test");
    send_pos(ok_pos2, 300);
    send_enable(1);
    send_pos(ok_pos, 300);
    replug_before_type = ARM_MSG_POSITION_CMD;
    o = step(10);
    CHECK(replug_before_type == 0, "hook fired");
    CHECK(!Arm_USB_Get_PC_Target(got) && !Arm_USB_PC_Enabled(),
          "frames parsed across a mid-parse replug are not committed (enabled %d)",
          Arm_USB_PC_Enabled());
    o = step(10);
    CHECK(!Arm_USB_PC_Enabled() && usb_stats.rx_frames > 0, "old session bytes discarded");

    /* 17c. Same, ending the session just before an ENABLE is committed. */
    send_enable(1);
    (void)step(10);
    CHECK(Arm_USB_PC_Enabled(), "re-enabled in the replugged session");
    send_simple(ARM_MSG_STOP, 0, 0);
    (void)step(10);
    send_enable(1);
    replug_before_type = ARM_MSG_ENABLE;
    o = step(10);
    CHECK(!Arm_USB_PC_Enabled() && o.acks == 0, "ENABLE of a dead session not committed or acked");

    /* 18. Feedback loss while running: STATE marks the joint stale. */
    send_enable(1);
    (void)step(10);
    send_pos(ok_pos, 300);
    (void)step(10);
    for (int i = 0; i < 6; i++)
    {
        joint_L_motor[i].update_tick = fake_ms + 110U;
    }
    fake_ms += 110U;
    send_heartbeat();
    o = step_no_feedback(0);
    CHECK(o.fb_valid == 0x3F && o.fb_age[6] == 110U, "grip stale after 110 ms, mask 0x%02x age %u",
          o.fb_valid, o.fb_age[6]);

    /* 18b. Leaving PC mode disarms (Arm_L_Task calls Arm_USB_PC_Disarm);
           coming back needs a new ENABLE, the old command never applies. */
    fresh_enabled_session();
    send_pos(ok_pos, 300);
    (void)step(10);
    CHECK(Arm_USB_Get_PC_Target(got), "running before the switch moves");
    ArmData.ArmMode = 6;                            /* TAKE_WATER, switch moved */
    Arm_USB_PC_Disarm();
    CHECK(!Arm_USB_PC_Enabled() && !Arm_USB_Get_PC_Target(got), "disarmed on mode exit");
    send_pos(ok_pos2, 300);                         /* host keeps streaming */
    o = step(10);
    CHECK(o.acks == 1 && o.ack_result[0] == ARM_ACK_NOT_ENABLED, "commands refused after exit");
    send_enable(1);
    o = step(10);
    CHECK(o.acks == 1 && o.ack_result[0] == ARM_ACK_WRONG_MODE, "re-ENABLE refused outside PC mode");
    ArmData.ArmMode = ARM_MODE_PC_CONTROL;          /* switch back up */
    o = step(10);
    CHECK(!Arm_USB_PC_Enabled() && !Arm_USB_Get_PC_Target(got),
          "returning to PC mode does not resume the old command");
    send_enable(1);
    (void)step(10);
    CHECK(Arm_USB_PC_Enabled() && !Arm_USB_Get_PC_Target(got),
          "new ENABLE ok, still no target until a fresh POSITION_CMD");

    /* 19. RX ring overflow refused whole and latched. */
    static uint8_t fill[RING_BUFFER_SIZE];
    memset(fill, 0, sizeof(fill));
    push(fill, sizeof(fill));
    push(fill, 100);
    o = step(10);
    CHECK(usb_stats.rx_overflow_bytes == 100 && (o.fault_code & ARM_FAULT_RX_OVERFLOW),
          "overflow counted and latched");

    /* 20. Replay frames built by Python in a fresh session. */
    char path[512];
    snprintf(path, sizeof(path), "%s/host_frames.bin", dir);
    FILE *hf = fopen(path, "rb");
    if (hf)
    {
        static uint8_t host[8192];
        size_t n = fread(host, 1, sizeof(host), hf);
        fclose(hf);

        usb_unplug();
        (void)step(10);
        usb_plug();
        (void)step(10);
        uint32_t mark = sent_len;
        for (size_t i = 0; i < n; i += 5)
        {
            push(&host[i], (uint32_t)((n - i) < 5 ? (n - i) : 5));
            (void)step(1);
        }
        (void)step(10);
        bool fresh = Arm_USB_Get_PC_Target(got);
        (void)step(10);
        write_file(dir, "cross_out.bin", &sent[mark], sent_len - mark);

        snprintf(path, sizeof(path), "%s/cross_target.txt", dir);
        FILE *tf = fopen(path, "w");
        if (tf)
        {
            fprintf(tf, "%d", fresh ? 1 : 0);
            for (int i = 0; i < 7; i++)
            {
                fprintf(tf, " %.7f", fresh ? got[i] : 0.0f);
            }
            fprintf(tf, "\n");
            fclose(tf);
        }
    }

    printf("C checks: %s (%d failure%s)\n", failures ? "FAILED" : "OK", failures,
           failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
