/* Host self-test of the power-loss log (APP/multi_turn_store.c), the restore
 * math (APP/multi_turn_restore.c) and the USART1 image-transmission stream
 * parser (Components/Device/Src/Image_Transmission.c).
 *
 * Fake Flash: 2 x 128 KiB, 0xFF after erase, a Flashword can only be
 * programmed once, power cuts can be injected before / inside any program,
 * a torn Flashword reads back as an ECC double error.
 * Built and run by mt_selftest.py; nothing here runs on the STM32.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "multi_turn_store.h"
#include "multi_turn_restore.h"
#include "Image_Transmission.h"
#include "CRC.h"
#include "can_bsp.h"
#include "referee_usart_task.h"
#include "task.h"
#include "usart.h"

static int failures;
#define CHECK(c) do { if (!(c)) { failures++; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); } } while (0)

/* ------------------------------------------------------- fake system */

static uint32_t fake_ms;
uint32_t   HAL_GetTick(void)        { return fake_ms; }
TickType_t xTaskGetTickCount(void)  { return fake_ms; }
void       osDelay(uint32_t ms)     { fake_ms += ms; }
UART_HandleTypeDef huart1;

joint_motor_measure_t joint_L_motor[7];
master_raw_t Master_Raw;

uint8_t APP_Joint_Fresh_Mask(uint32_t now_tick)
{
    uint8_t mask = 0;
    for (int i = 0; i < 7; i++)
    {
        int32_t age = (int32_t)(now_tick - joint_L_motor[i].update_tick);
        if (joint_L_motor[i].update_count && age <= (int32_t)JOINT_FEEDBACK_TIMEOUT_MS &&
            joint_L_motor[i].err < JOINT_ERR_FAULT_MIN)
        {
            mask |= (uint8_t)(1U << i);
        }
    }
    return mask;
}

static void feed_all(const float raw[7], float vel)
{
    for (int i = 0; i < 7; i++)
    {
        joint_L_motor[i].position = raw[i];
        joint_L_motor[i].velocity = vel;
        joint_L_motor[i].update_tick = fake_ms;
        joint_L_motor[i].update_count++;
        joint_L_motor[i].err = 1;
    }
}

/* -------------------------------------------------------- fake Flash */

#define FLASH_BYTES (2U * MT_SECTOR_BYTES)
#define NWORDS      (FLASH_BYTES / MT_FLASHWORD_BYTES)
static uint8_t  flash[FLASH_BYTES];
static uint8_t  programmed[NWORDS];
static uint8_t  ecc_bad[NWORDS];
static uint16_t fake_kib = 1024;
static int      program_budget = -1;     /* <0: unlimited; 0: power cut now */
static int      cut_inside;              /* cut leaves a torn Flashword */
static int      powered = 1;
static int      program_calls, erase_calls;

static void flash_reset(void)
{
    memset(flash, 0xFF, sizeof(flash));
    memset(programmed, 0, sizeof(programmed));
    memset(ecc_bad, 0, sizeof(ecc_bad));
    program_budget = -1;
    powered = 1;
}

uint8_t MT_Port_Read(uint32_t addr, uint8_t *dst, uint32_t len)
{
    if (addr < MT_STORE_A_BASE || addr + len > MT_STORE_A_BASE + FLASH_BYTES)
    {
        return 0;
    }
    uint32_t off = addr - MT_STORE_A_BASE;
    for (uint32_t w = off / 32U; w <= (off + len - 1U) / 32U; w++)
    {
        if (ecc_bad[w])
        {
            return 0;
        }
    }
    memcpy(dst, &flash[off], len);
    return 1;
}

uint8_t MT_Port_Program(uint32_t addr, const uint32_t words[8])
{
    uint32_t off = addr - MT_STORE_A_BASE;
    program_calls++;
    if (!powered || (off % 32U) || off >= FLASH_BYTES || programmed[off / 32U])
    {
        return 0;
    }
    if (program_budget == 0)
    {
        powered = 0;                    /* power is gone from here on */
        if (cut_inside)
        {
            memcpy(&flash[off], words, 16);
            programmed[off / 32U] = 1;
            ecc_bad[off / 32U] = 1;
        }
        return 0;
    }
    if (program_budget > 0)
    {
        program_budget--;
    }
    memcpy(&flash[off], words, 32);
    programmed[off / 32U] = 1;
    return 1;
}

uint8_t MT_Port_Erase(uint8_t sector)
{
    uint32_t off = (sector == MT_STORE_A_SECTOR) ? 0U : MT_SECTOR_BYTES;
    erase_calls++;
    if (!powered)
    {
        return 0;
    }
    memset(&flash[off], 0xFF, MT_SECTOR_BYTES);
    memset(&programmed[off / 32U], 0, MT_SECTOR_BYTES / 32U);
    memset(&ecc_bad[off / 32U], 0, MT_SECTOR_BYTES / 32U);
    return 1;
}

uint8_t MT_Port_Flash_KiB(uint16_t *kib) { *kib = fake_kib; return 1; }
void    MT_Store_MPU_Config(void) { }

static void reboot(void)
{
    powered = 1;
    program_budget = -1;
    MT_Store_Boot_Maintenance();
}

static MT_Record_t make_rec(float base)
{
    MT_Record_t r;
    memset(&r, 0, sizeof(r));
    r.config_id = 0x12345678U;
    r.flags = MT_FLAG_SLAVE_POS_VALID;
    r.slave_valid_mask = 0x7F;
    for (int i = 0; i < 7; i++)
    {
        r.slave_logical_pos[i] = base + 0.1f * (float)i;
    }
    r.master_anchor[0] = -1.25;
    r.master_wire_int[1] = -123456;
    return r;
}

/* --------------------------------------------------------------- store */

static void test_crc_and_codec(void)
{
    uint8_t body[MT_BODY_BYTES];
    MT_Record_t a = make_rec(1.0f), b;

    CHECK(MT_Crc32((const uint8_t *)"123456789", 9) == 0xCBF43926UL);
    a.seq = 77;
    a.master_payload[29] = 0xA5;
    MT_Record_Encode(&a, body);
    CHECK(body[0] == 0x4C && body[3] == 0x4D);              /* "MTRL" little endian */
    CHECK(body[208] == 0xFF && body[251] == 0xFF);          /* reserved */
    CHECK(MT_Record_Decode(body, &b));
    CHECK(b.seq == 77 && b.master_wire_int[1] == -123456 && b.master_anchor[0] == -1.25);
    CHECK(b.slave_logical_pos[6] == a.slave_logical_pos[6] && b.master_payload[29] == 0xA5);
    body[40] ^= 1U;
    CHECK(!MT_Record_Decode(body, &b));
}

static void test_store_basic(void)
{
    MT_Record_t r = make_rec(0.5f), got;

    flash_reset();
    fake_kib = 0xFFFF;
    erase_calls = program_calls = 0;
    reboot();
    CHECK(MT_Store_Status.state == MT_STORE_DISABLED);
    CHECK(!MT_Store_Append(&r) && erase_calls == 0 && program_calls == 0);
    fake_kib = 0;
    reboot();
    CHECK(MT_Store_Status.state == MT_STORE_DISABLED);
    fake_kib = 1024;

    reboot();
    CHECK(MT_Store_Status.state == MT_STORE_OK && !MT_Store_Status.has_record && erase_calls == 0);
    CHECK(MT_Store_Append(&r) && r.seq == 1);
    r = make_rec(0.7f);
    CHECK(MT_Store_Append(&r) && r.seq == 2);
    reboot();
    CHECK(MT_Store_Get_Latest(&got) && got.seq == 2 && got.slave_logical_pos[0] == 0.7f);
    CHECK(MT_Store_Status.next_slot == 2 && MT_Store_Status.active == 0);
}

static void test_store_power_cut(void)
{
    MT_Record_t r, got;

    for (int cut = 0; cut <= 8; cut++)
    {
        for (cut_inside = 0; cut_inside <= 1; cut_inside++)
        {
            flash_reset();
            reboot();
            r = make_rec(1.0f);
            CHECK(MT_Store_Append(&r));
            r = make_rec(2.0f);
            program_budget = cut;                       /* cut before / inside Flashword `cut` */
            CHECK(!MT_Store_Append(&r));
            reboot();
            CHECK(MT_Store_Status.state == MT_STORE_OK);
            CHECK(MT_Store_Get_Latest(&got) && got.seq == 1 && got.slave_logical_pos[0] == 1.0f);
            /* the torn slot is skipped, the next append lands after it */
            r = make_rec(3.0f);
            CHECK(MT_Store_Append(&r) && r.seq == 2);
            reboot();
            CHECK(MT_Store_Get_Latest(&got) && got.seq == 2 && got.slave_logical_pos[0] == 3.0f);
        }
    }
    cut_inside = 0;
}

static void test_store_rotate_and_full(void)
{
    MT_Record_t r, got;
    uint32_t n;

    flash_reset();
    reboot();
    for (n = 0; n < MT_SLOTS_PER_SECTOR; n++)
    {
        r = make_rec((float)n * 0.001f);
        CHECK(MT_Store_Append(&r));
    }
    CHECK(MT_Store_Status.active == 0);
    /* sector B is still blank: run time moves on without erasing */
    erase_calls = 0;
    r = make_rec(9.0f);
    CHECK(MT_Store_Append(&r) && MT_Store_Status.active == 1 && erase_calls == 0);
    for (n = 1; n < MT_SLOTS_PER_SECTOR; n++)
    {
        r = make_rec(5.0f);
        CHECK(MT_Store_Append(&r));
    }
    /* both full: stop, never erase at run time */
    r = make_rec(6.0f);
    CHECK(!MT_Store_Append(&r) && MT_Store_Status.state == MT_STORE_FULL && erase_calls == 0);

    /* boot rotation: erase A (older), copy latest, B untouched */
    reboot();
    CHECK(MT_Store_Status.state == MT_STORE_OK && MT_Store_Status.active == 0 && erase_calls == 1);
    CHECK(MT_Store_Get_Latest(&got) && got.slave_logical_pos[0] == 5.0f && got.seq == 2U * MT_SLOTS_PER_SECTOR + 1U);
    CHECK(flash[MT_SECTOR_BYTES + 288] != 0xFF);           /* B kept */

    /* power cut during the rotation copy: latest still in B */
    flash_reset();
    reboot();
    for (n = 0; n < MT_SLOTS_PER_SECTOR - 10U; n++)
    {
        r = make_rec(7.0f);
        CHECK(MT_Store_Append(&r));
    }
    program_budget = 3;
    MT_Store_Boot_Maintenance();
    CHECK(MT_Store_Status.state == MT_STORE_FAULT);
    CHECK(MT_Store_Get_Latest(&got) && got.seq == MT_SLOTS_PER_SECTOR - 10U);   /* A still holds it */
    /* next boot: B (torn copy, older) is erased and the copy is redone */
    erase_calls = 0;
    reboot();
    CHECK(erase_calls == 1);
    CHECK(MT_Store_Get_Latest(&got) && got.slave_logical_pos[0] == 7.0f && got.seq == MT_SLOTS_PER_SECTOR - 9U);
    CHECK(MT_Store_Status.state == MT_STORE_OK && MT_Store_Status.active == 1);
}

static void test_store_seq_wrap(void)
{
    MT_Record_t r, got;

    flash_reset();
    reboot();
    r = make_rec(1.0f);
    CHECK(MT_Store_Append(&r));
    /* fake history: A holds 0xFFFFFFFE, B the wrapped seq 3 written later */
    flash_reset();
    reboot();
    r = make_rec(1.0f);
    MT_Store_Append(&r);                         /* seq 1 in A slot 0 */
    {
        uint8_t body[MT_BODY_BYTES];
        uint32_t w[8];
        MT_Record_t a = make_rec(1.0f), b = make_rec(2.0f);
        uint8_t commit[32];
        flash_reset();
        a.seq = 0xFFFFFFFEU;
        b.seq = 3U;
        for (int k = 0; k < 2; k++)
        {
            MT_Record_t *x = k ? &b : &a;
            uint32_t base = MT_STORE_A_BASE + (k ? MT_SECTOR_BYTES : 0U);
            MT_Record_Encode(x, body);
            memset(commit, 0xFF, 32);
            uint32_t v[4] = { MT_COMMIT_MAGIC, x->seq, 0, ~MT_COMMIT_MAGIC };
            memcpy(&v[2], &body[252], 4);
            memcpy(commit, v, 16);
            for (int f = 0; f < 8; f++) { memcpy(w, &body[32 * f], 32); MT_Port_Program(base + 32U * f, w); }
            memcpy(w, commit, 32);
            MT_Port_Program(base + 256U, w);
        }
    }
    reboot();
    CHECK(MT_Store_Get_Latest(&got) && got.seq == 3U && got.slave_logical_pos[0] == 2.0f);
    CHECK(MT_Store_Status.active == 1);
}

/* ------------------------------------------------------------- restore */

static void test_restore_math(void)
{
    float saved[7] = { 0.5f, 1.0f, 1.0f, 4.0f, 0.2f, -0.3f, 0.6f };
    float raw[7], b[7];

    for (int i = 0; i < 7; i++) { raw[i] = saved[i]; }
    raw[3] = 4.0f - 6.28318531f;                 /* J4 came back one turn lower */
    CHECK(MT_Restore_Compute(saved, raw, b));
    CHECK(fabsf(b[3] - 6.28318531f) < 1e-4f && b[0] == 0.0f);
    CHECK(fabsf(raw[3] + b[3] - saved[3]) < 1e-4f);

    raw[3] = -4.0f + 6.28318531f; saved[3] = -4.0f;    /* negative direction */
    CHECK(MT_Restore_Compute(saved, raw, b) && fabsf(b[3] + 6.28318531f) < 1e-4f);

    raw[3] = saved[3] + 0.6f;                    /* residual above MT_RESTORE_RESIDUAL_MAX */
    CHECK(!MT_Restore_Compute(saved, raw, b));
    raw[3] = saved[3] + 3.14159265f;             /* half-period ambiguity */
    CHECK(!MT_Restore_Compute(saved, raw, b));
    raw[3] = saved[3];
    raw[6] = NAN;
    CHECK(!MT_Restore_Compute(saved, raw, b));
    raw[6] = saved[6];
    saved[2] = 2.6f; raw[2] = 2.6f;              /* J3 beyond its limit + margin */
    CHECK(!MT_Restore_Compute(saved, raw, b));
}

static void test_restore_boot_and_save(void)
{
    float raw[7] = { 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.3f };

    flash_reset();
    reboot();
    fake_ms = 1000;
    feed_all(raw, 0.0f);
    MT_Restore_Boot();
#if MT_RESTORE_ENABLE
    CHECK(MT_Restore_State == MT_RESTORE_NEED_REFERENCE);        /* no record yet */
    CHECK(!MT_Restore_Output_Allowed());
#else
    MT_Record_t got;
    uint32_t w0;

    CHECK(MT_Restore_State == MT_RESTORE_DISABLED && MT_Restore_Output_Allowed());
    for (int i = 0; i < 7; i++) { CHECK(Joint_Boot_Offset[i] == 0.0f); }

    /* save trigger: still for 500 ms in the save window, 1 s apart, real move */
    w0 = MT_Store_Status.writes;
    for (int k = 0; k < 200; k++) { fake_ms += 8; feed_all(raw, 0.0f); MT_Restore_Process(1); }
    CHECK(MT_Store_Status.writes == w0 + 1U);
    CHECK(MT_Store_Get_Latest(&got) && got.config_id == MT_Config_Id());
    CHECK(got.calibration_state == MT_CAL_LEGACY && got.slave_logical_pos[6] == 0.3f);
    for (int k = 0; k < 400; k++) { fake_ms += 8; feed_all(raw, 0.0f); MT_Restore_Process(1); }
    CHECK(MT_Store_Status.writes == w0 + 1U);                    /* no move, no write */
    raw[1] += 0.2f;
    for (int k = 0; k < 400; k++) { fake_ms += 8; feed_all(raw, 0.5f); MT_Restore_Process(1); }
    CHECK(MT_Store_Status.writes == w0 + 1U);                    /* moving */
    for (int k = 0; k < 400; k++) { fake_ms += 8; feed_all(raw, 0.0f); MT_Restore_Process(0); }
    CHECK(MT_Store_Status.writes == w0 + 1U);                    /* outside the window */
    for (int k = 0; k < 400; k++) { fake_ms += 8; feed_all(raw, 0.0f); MT_Restore_Process(1); }
    CHECK(MT_Store_Status.writes == w0 + 2U);
#endif
}

#if MT_RESTORE_ENABLE
static void test_restore_enabled(void)
{
    float saved[7] = { 0.1f, 0.2f, 0.3f, 2.0f, 0.5f, 0.6f, 0.3f };
    float raw[7];
    MT_Record_t r;

    flash_reset();
    reboot();
    memset(&r, 0, sizeof(r));
    r.config_id = MT_Config_Id();
    r.flags = MT_FLAG_SLAVE_POS_VALID;
    r.slave_valid_mask = 0x7F;
    memcpy(r.slave_logical_pos, saved, sizeof(saved));
    CHECK(MT_Store_Append(&r));
    reboot();

    memcpy(raw, saved, sizeof(raw));
    raw[3] = 2.0f - 6.28318531f;
    fake_ms += 1000;
    feed_all(raw, 0.0f);
    MT_Restore_Boot();
    CHECK(MT_Restore_State == MT_RESTORE_READY && fabsf(Joint_Boot_Offset[3] - 6.28318531f) < 1e-4f);
    CHECK(fabsf(JOINT_LOGICAL_POSITION(3) - 2.0f) < 1e-4f);

    /* one joint stops feeding back: b is no longer trusted */
    fake_ms += 8;
    feed_all(raw, 0.0f);
    joint_L_motor[2].update_tick = fake_ms - 500U;
    MT_Restore_Process(1);
    CHECK(MT_Restore_State == MT_RESTORE_FAULT && !MT_Restore_Output_Allowed());

    /* record from another configuration: refuse */
    r.config_id ^= 1U;
    CHECK(MT_Store_Append(&r));
    reboot();
    fake_ms += 8;
    feed_all(raw, 0.0f);
    MT_Restore_Boot();
    CHECK(MT_Restore_State == MT_RESTORE_NEED_REFERENCE);
    for (int i = 0; i < 7; i++) { CHECK(Joint_Boot_Offset[i] == 0.0f); }
}
#endif

/* --------------------------------------------------- stream parser */

static uint16_t make_frame(uint8_t *f, uint16_t cmd, const uint8_t *payload, uint16_t n, uint8_t seq)
{
    f[0] = 0xA5; f[1] = (uint8_t)n; f[2] = (uint8_t)(n >> 8); f[3] = seq;
    Append_CRC8_Check_Sum(f, 5);
    f[5] = (uint8_t)cmd; f[6] = (uint8_t)(cmd >> 8);
    memcpy(&f[7], payload, n);
    Append_CRC16_Check_Sum(f, (uint32_t)(n + 9U));
    return (uint16_t)(n + 9U);
}

static void feed(const uint8_t *s, uint32_t len, uint32_t chunk)
{
    static uint8_t block[IMAGE_TRANS_RX_BUF_LEN];
    for (uint32_t off = 0; off < len; off += chunk)
    {
        uint32_t n = (len - off < chunk) ? (len - off) : chunk;
        memcpy(block, &s[off], n);
        Image_Transmission_Info_Update(block, (uint16_t)n);
    }
}

static void test_parser(void)
{
    uint8_t p30[30], p12[12], f[64], s[600], got[30];
    uint16_t n;
    uint32_t len, c0, tick;

    for (int i = 0; i < 30; i++) { p30[i] = (uint8_t)(i * 7 + 1); }
    memset(p12, 0, sizeof(p12));
    p12[0] = 0x34; p12[1] = 0x12; p12[8] = 0x00; p12[9] = 0x08;    /* mouse_x 0x1234, key Z */

    fake_ms = 5000;
    n = make_frame(f, 0x0302, p30, 30, 1);
    feed(f, n, 78);
    CHECK(Image_Transmission_Get_Custom(got, &tick) == 1U && memcmp(got, p30, 30) == 0 && tick == 5000);

    /* split over blocks of 1..38 bytes */
    for (uint32_t chunk = 1; chunk < 39; chunk++)
    {
        c0 = Image_Transmission_Info.custom_rx_count;
        p30[0] = (uint8_t)chunk;
        n = make_frame(f, 0x0302, p30, 30, 2);
        feed(f, n, chunk);
        CHECK(Image_Transmission_Info.custom_rx_count == c0 + 1U);
        Image_Transmission_Get_Custom(got, 0);
        CHECK(got[0] == (uint8_t)chunk);
    }

    /* noise + 0x0304 + 0x0302 glued together in one block */
    len = 0;
    s[len++] = 0x00; s[len++] = 0xA5; s[len++] = 0x13; s[len++] = 0xA9;
    len += make_frame(&s[len], 0x0304, p12, 12, 3);
    len += make_frame(&s[len], 0x0302, p30, 30, 4);
    c0 = Image_Transmission_Info.custom_rx_count;
    uint32_t r0 = Image_Transmission_Info.remote_rx_count;
    feed(s, len, 78);
    CHECK(Image_Transmission_Info.custom_rx_count == c0 + 1U);
    CHECK(Image_Transmission_Info.remote_rx_count == r0 + 1U);
    CHECK(Image_Transmission_Info.remote_control.mouse_x == 0x1234);
    CHECK(Image_Transmission_Info.remote_control.Key.Set.Z == 1);

    /* right length, CRC ok, wrong cmd_id: not 0x0302 */
    c0 = Image_Transmission_Info.custom_rx_count;
    n = make_frame(f, 0x0301, p30, 30, 5);
    feed(f, n, 78);
    CHECK(Image_Transmission_Info.custom_rx_count == c0);

    /* bad CRC16, and a header announcing 120 bytes: rejected, then recovers */
    n = make_frame(f, 0x0302, p30, 30, 6);
    f[20] ^= 0x55;
    feed(f, n, 78);
    CHECK(Image_Transmission_Info.custom_rx_count == c0);
    {
        uint8_t big[5] = { 0xA5, 120, 0, 7, 0 };
        Append_CRC8_Check_Sum(big, 5);
        feed(big, 5, 5);
    }
    n = make_frame(f, 0x0302, p30, 30, 8);
    feed(f, n, 78);
    CHECK(Image_Transmission_Info.custom_rx_count == c0 + 1U);

    /* oversized Size is clamped, random noise never breaks the stream */
    {
        static uint8_t junk[IMAGE_TRANS_RX_BUF_LEN * 3];
        uint32_t seed = 12345;
        for (uint32_t i = 0; i < sizeof(junk); i++) { seed = seed * 1103515245U + 12345U; junk[i] = (uint8_t)(seed >> 16); }
        Image_Transmission_Info_Update(junk, (uint16_t)sizeof(junk));
        for (int k = 0; k < 2000; k++)
        {
            seed = seed * 1103515245U + 12345U;
            feed(&junk[seed % 100U], 1U + (seed >> 8) % 78U, 78);
        }
        c0 = Image_Transmission_Info.custom_rx_count;
        n = make_frame(f, 0x0302, p30, 30, 9);
        feed(f, n, 78);
        feed(f, n, 78);
        CHECK(Image_Transmission_Info.custom_rx_count >= c0 + 1U);
    }
}

int main(void)
{
    test_crc_and_codec();
    test_store_basic();
    test_store_power_cut();
    test_store_rotate_and_full();
    test_store_seq_wrap();
    test_restore_math();
    test_restore_boot_and_save();
#if MT_RESTORE_ENABLE
    test_restore_enabled();
#endif
    test_parser();
    printf("MT_RESTORE_ENABLE=%d: %d failure(s)\n", MT_RESTORE_ENABLE, failures);
    return failures ? 1 : 0;
}
