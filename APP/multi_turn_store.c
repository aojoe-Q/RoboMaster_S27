/**
  ******************************************************************************
  * @file    multi_turn_store.c
  * @brief   Power-loss log in Flash Bank 1 Sector 6 / 7 (see the header).
  *
  * The Flash access is in the MT_Port_* functions at the end of the file;
  * a host test builds this file with MT_STORE_HOST_TEST and its own port.
  ******************************************************************************
  */

#include "multi_turn_store.h"
#include <string.h>

_Static_assert(MT_SLOTS_PER_SECTOR == 455U, "288-byte slots per 128 KiB sector");
_Static_assert((MT_RECORD_BYTES % MT_FLASHWORD_BYTES) == 0U, "record is whole Flashwords");
_Static_assert((MT_BODY_BYTES % MT_FLASHWORD_BYTES) == 0U, "body is whole Flashwords");
_Static_assert((MT_STORE_A_BASE % MT_SECTOR_BYTES) == 0U, "log A on a sector boundary");
_Static_assert(MT_STORE_B_BASE == MT_STORE_A_BASE + MT_SECTOR_BYTES, "log B follows log A");

/* Flash access, one implementation per build. Return 1 on success. */
uint8_t MT_Port_Read(uint32_t addr, uint8_t *dst, uint32_t len);   /* 0: ECC / bus error */
uint8_t MT_Port_Program(uint32_t addr, const uint32_t words[8]);  /* one Flashword       */
uint8_t MT_Port_Erase(uint8_t sector);
uint8_t MT_Port_Flash_KiB(uint16_t *kib);                          /* raw 0x1FF1E880      */

MT_Store_Status_t MT_Store_Status;

static MT_Record_t latest;
static uint16_t    next_slot[2];
static uint8_t     sector_blank[2];

static const uint32_t sector_base[2]   = { MT_STORE_A_BASE, MT_STORE_B_BASE };
static const uint8_t  sector_number[2] = { MT_STORE_A_SECTOR, MT_STORE_B_SECTOR };

/* Source buffers stay valid and aligned for the whole HAL program call. */
static uint8_t  slot_buf[MT_RECORD_BYTES] __attribute__((aligned(32)));
static uint8_t  read_buf[MT_RECORD_BYTES] __attribute__((aligned(32)));
static uint32_t word_buf[8]               __attribute__((aligned(32)));

/* ----------------------------------------------------------------- CRC */

uint32_t MT_Crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;

    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8U; b++)
        {
            crc = (crc & 1UL) ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
        }
    }
    return crc ^ 0xFFFFFFFFUL;
}

/* -------------------------------------------------- little-endian body */

static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;         p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static void put_f32(uint8_t *p, float v)  { uint32_t u; memcpy(&u, &v, 4); put_u32(p, u); }
static void put_f64(uint8_t *p, double v)
{
    uint64_t u;
    memcpy(&u, &v, 8);
    put_u32(p, (uint32_t)u);
    put_u32(p + 4, (uint32_t)(u >> 32));
}
static uint16_t get_u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static float  get_f32(const uint8_t *p) { uint32_t u = get_u32(p); float v; memcpy(&v, &u, 4); return v; }
static double get_f64(const uint8_t *p)
{
    uint64_t u = (uint64_t)get_u32(p) | ((uint64_t)get_u32(p + 4) << 32);
    double v;
    memcpy(&v, &u, 8);
    return v;
}

void MT_Record_Encode(const MT_Record_t *rec, uint8_t body[MT_BODY_BYTES])
{
    uint8_t i;

    memset(body, 0xFF, MT_BODY_BYTES);              /* reserved bytes are 0xFF */
    put_u32(&body[0],  MT_RECORD_MAGIC);
    put_u16(&body[4],  MT_FORMAT_VERSION);
    put_u16(&body[6],  (uint16_t)MT_BODY_BYTES);
    put_u32(&body[8],  rec->seq);
    put_u32(&body[12], rec->config_id);
    put_u32(&body[16], rec->flags);
    put_u32(&body[20], rec->slave_valid_mask);
    put_u32(&body[24], rec->master_valid_mask);
    put_u32(&body[28], rec->source_protocol_id);
    for (i = 0; i < MT_JOINT_NUM; i++)
    {
        put_f32(&body[32 + 4U * i], rec->slave_logical_pos[i]);
        put_f32(&body[60 + 4U * i], rec->slave_anchor[i]);
    }
    for (i = 0; i < 4U; i++)
    {
        put_f32(&body[88 + 4U * i], rec->master_wire_float[i]);
    }
    put_u32(&body[104], (uint32_t)rec->master_wire_int[0]);
    put_u32(&body[108], (uint32_t)rec->master_wire_int[1]);
    memcpy(&body[112], rec->master_payload, MT_MASTER_PAYLOAD_LEN);
    for (i = 0; i < 6U; i++)
    {
        put_f64(&body[144 + 8U * i], rec->master_anchor[i]);
    }
    put_u32(&body[192], rec->mapped_axis_mask);
    put_u32(&body[196], rec->source_session);
    put_u32(&body[200], rec->master_session_flags);
    put_u32(&body[204], rec->calibration_state);
    put_u32(&body[252], MT_Crc32(body, 252U));
}

uint8_t MT_Record_Decode(const uint8_t body[MT_BODY_BYTES], MT_Record_t *rec)
{
    uint8_t i;

    if ((get_u32(&body[0]) != MT_RECORD_MAGIC) || (get_u16(&body[4]) != MT_FORMAT_VERSION) ||
        (get_u16(&body[6]) != MT_BODY_BYTES)   || (get_u32(&body[252]) != MT_Crc32(body, 252U)))
    {
        return 0U;
    }
    rec->seq                = get_u32(&body[8]);
    rec->config_id          = get_u32(&body[12]);
    rec->flags              = get_u32(&body[16]);
    rec->slave_valid_mask   = get_u32(&body[20]);
    rec->master_valid_mask  = get_u32(&body[24]);
    rec->source_protocol_id = get_u32(&body[28]);
    for (i = 0; i < MT_JOINT_NUM; i++)
    {
        rec->slave_logical_pos[i] = get_f32(&body[32 + 4U * i]);
        rec->slave_anchor[i]      = get_f32(&body[60 + 4U * i]);
    }
    for (i = 0; i < 4U; i++)
    {
        rec->master_wire_float[i] = get_f32(&body[88 + 4U * i]);
    }
    rec->master_wire_int[0] = (int32_t)get_u32(&body[104]);
    rec->master_wire_int[1] = (int32_t)get_u32(&body[108]);
    memcpy(rec->master_payload, &body[112], MT_MASTER_PAYLOAD_LEN);
    for (i = 0; i < 6U; i++)
    {
        rec->master_anchor[i] = get_f64(&body[144 + 8U * i]);
    }
    rec->mapped_axis_mask     = get_u32(&body[192]);
    rec->source_session       = get_u32(&body[196]);
    rec->master_session_flags = get_u32(&body[200]);
    rec->calibration_state    = get_u32(&body[204]);
    return 1U;
}

static void encode_commit(uint32_t seq, uint32_t body_crc, uint8_t commit[MT_FLASHWORD_BYTES])
{
    memset(commit, 0xFF, MT_FLASHWORD_BYTES);
    put_u32(&commit[0],  MT_COMMIT_MAGIC);
    put_u32(&commit[4],  seq);
    put_u32(&commit[8],  body_crc);
    put_u32(&commit[12], ~MT_COMMIT_MAGIC);
}

/* --------------------------------------------------------------- slots */

typedef enum { SLOT_BLANK, SLOT_VALID, SLOT_BAD } slot_kind_t;

static uint8_t seq_newer(uint32_t a, uint32_t b)
{
    return (uint8_t)((int32_t)(a - b) > 0);
}

static slot_kind_t read_slot(uint32_t addr, MT_Record_t *rec)
{
    uint8_t commit[MT_FLASHWORD_BYTES];
    uint8_t blank = 1U;

    if (!MT_Port_Read(addr, read_buf, MT_RECORD_BYTES))
    {
        return SLOT_BAD;                    /* ECC double error: never reused */
    }
    for (uint32_t i = 0; i < MT_RECORD_BYTES; i++)
    {
        if (read_buf[i] != 0xFFU)
        {
            blank = 0U;
            break;
        }
    }
    if (blank)
    {
        return SLOT_BLANK;
    }
    if (!MT_Record_Decode(read_buf, rec))
    {
        return SLOT_BAD;
    }
    encode_commit(rec->seq, get_u32(&read_buf[252]), commit);
    if (memcmp(commit, &read_buf[MT_BODY_BYTES], MT_FLASHWORD_BYTES) != 0)
    {
        return SLOT_BAD;                    /* body written, commit missing or torn */
    }
    return SLOT_VALID;
}

/* Fills next_slot / sector_blank of one sector; updates latest. */
static void scan_sector(uint8_t s)
{
    MT_Record_t rec;
    int32_t last_used = -1;

    for (uint16_t slot = 0; slot < MT_SLOTS_PER_SECTOR; slot++)
    {
        slot_kind_t k = read_slot(sector_base[s] + (uint32_t)slot * MT_RECORD_BYTES, &rec);

        if (k == SLOT_BLANK)
        {
            continue;
        }
        last_used = slot;
        if (k == SLOT_BAD)
        {
            MT_Store_Status.bad_slots++;
            continue;
        }
        if (!MT_Store_Status.has_record || seq_newer(rec.seq, latest.seq))
        {
            latest = rec;
            MT_Store_Status.has_record = 1U;
            MT_Store_Status.active = s;
        }
    }
    next_slot[s]    = (uint16_t)(last_used + 1);
    sector_blank[s] = (uint8_t)(last_used < 0);
}

static uint8_t erase_sector(uint8_t s)
{
    MT_Store_Status.erases++;
    if (!MT_Port_Erase(sector_number[s]))
    {
        return 0U;
    }
    for (uint32_t off = 0; off < MT_SECTOR_BYTES; off += MT_RECORD_BYTES)
    {
        uint32_t len = MT_SECTOR_BYTES - off;
        if (len > MT_RECORD_BYTES)
        {
            len = MT_RECORD_BYTES;
        }
        if (!MT_Port_Read(sector_base[s] + off, read_buf, len))
        {
            return 0U;
        }
        for (uint32_t i = 0; i < len; i++)
        {
            if (read_buf[i] != 0xFFU)
            {
                return 0U;
            }
        }
    }
    next_slot[s]    = 0U;
    sector_blank[s] = 1U;
    return 1U;
}

/* Write rec into slot next_slot[s] of sector s. 1 = committed and verified. */
static uint8_t write_slot(uint8_t s, const MT_Record_t *rec)
{
    uint32_t addr = sector_base[s] + (uint32_t)next_slot[s] * MT_RECORD_BYTES;
    MT_Record_t check;

    if (read_slot(addr, &check) != SLOT_BLANK)
    {
        next_slot[s]++;                     /* someone else's bytes: skip, do not program */
        MT_Store_Status.bad_slots++;
        return 0U;
    }

    MT_Record_Encode(rec, slot_buf);
    encode_commit(rec->seq, get_u32(&slot_buf[252]), &slot_buf[MT_BODY_BYTES]);

    /* The slot counts as used from the first programmed Flashword on. */
    next_slot[s]++;
    sector_blank[s] = 0U;

    for (uint32_t off = 0; off < MT_BODY_BYTES; off += MT_FLASHWORD_BYTES)
    {
        memcpy(word_buf, &slot_buf[off], MT_FLASHWORD_BYTES);
        if (!MT_Port_Program(addr + off, word_buf))
        {
            return 0U;
        }
    }
    if (!MT_Port_Read(addr, read_buf, MT_BODY_BYTES) ||
        (memcmp(read_buf, slot_buf, MT_BODY_BYTES) != 0))
    {
        return 0U;
    }
    memcpy(word_buf, &slot_buf[MT_BODY_BYTES], MT_FLASHWORD_BYTES);
    if (!MT_Port_Program(addr + MT_BODY_BYTES, word_buf))
    {
        return 0U;
    }
    if (read_slot(addr, &check) != SLOT_VALID)
    {
        return 0U;
    }
    latest = check;
    MT_Store_Status.has_record = 1U;
    MT_Store_Status.active     = s;
    MT_Store_Status.next_slot  = next_slot[s];
    MT_Store_Status.writes++;
    return 1U;
}

/* ---------------------------------------------------------------- API */

void MT_Store_Boot_Maintenance(void)
{
    uint16_t kib = 0U;
    uint8_t  s;

    memset(&MT_Store_Status, 0, sizeof(MT_Store_Status));
    if (!MT_Port_Flash_KiB(&kib) || (kib != MT_FLASH_SIZE_KIB))
    {
        /* Includes 0x0000 / 0xFFFF and read errors: never fall back to a
           default size, never erase or program. */
        MT_Store_Status.raw_flash_kib = kib;
        MT_Store_Status.state = MT_STORE_DISABLED;
        return;
    }
    MT_Store_Status.raw_flash_kib = kib;

    scan_sector(0U);
    scan_sector(1U);

    if (MT_Store_Status.has_record)
    {
        s = MT_Store_Status.active;
    }
    else
    {
        /* No committed record: nothing to keep. Start in a blank sector,
           or erase A if both hold leftovers. */
        s = sector_blank[0] ? 0U : (sector_blank[1] ? 1U : 0U);
        if (!sector_blank[s] && !erase_sector(s))
        {
            MT_Store_Status.state = MT_STORE_FAULT;
            return;
        }
        MT_Store_Status.active = s;
    }

    if (MT_Store_Status.has_record &&
        ((MT_SLOTS_PER_SECTOR - next_slot[s]) < MT_ROTATE_MARGIN_SLOTS))
    {
        /* Rotate: the other sector only holds older records. The active
           sector keeps the latest record until the copy is committed. */
        uint8_t o = (uint8_t)(1U - s);
        MT_Record_t copy = latest;

        if (!sector_blank[o] && !erase_sector(o))
        {
            MT_Store_Status.state = MT_STORE_FAULT;
            return;
        }
        copy.seq = latest.seq + 1U;
        if (!write_slot(o, &copy))
        {
            MT_Store_Status.write_errors++;
            MT_Store_Status.state = MT_STORE_FAULT;
            return;
        }
        s = o;
    }

    MT_Store_Status.active    = s;
    MT_Store_Status.next_slot = next_slot[s];
    MT_Store_Status.state     = MT_STORE_OK;
}

uint8_t MT_Store_Get_Latest(MT_Record_t *rec)
{
    if (!MT_Store_Status.has_record)
    {
        return 0U;
    }
    *rec = latest;
    return 1U;
}

uint8_t MT_Store_Append(MT_Record_t *rec)
{
    uint8_t s = MT_Store_Status.active;

    if (MT_Store_Status.state != MT_STORE_OK)
    {
        return 0U;
    }
    if (next_slot[s] >= MT_SLOTS_PER_SECTOR)
    {
        uint8_t o = (uint8_t)(1U - s);
        if (!sector_blank[o])
        {
            MT_Store_Status.state = MT_STORE_FULL;   /* erase only at boot */
            return 0U;
        }
        s = o;
    }
    rec->seq = MT_Store_Status.has_record ? (latest.seq + 1U) : 1U;
    if (!write_slot(s, rec))
    {
        MT_Store_Status.write_errors++;
        if (MT_Store_Status.write_errors >= 3U)
        {
            MT_Store_Status.state = MT_STORE_FAULT;  /* do not keep burning slots */
        }
        return 0U;
    }
    return 1U;
}

/* ------------------------------------------------------- target port */
#ifndef MT_STORE_HOST_TEST

#include "main.h"

void MT_Store_MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    /* MPU_Config() disables the MPU, adds Region 0 and enables it again; a
       region set here is kept. Region 0 (AXI SRAM) is not touched. */
    HAL_MPU_Disable();

    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER1;
    MPU_InitStruct.BaseAddress      = MT_STORE_A_BASE;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_256KB;
    MPU_InitStruct.SubRegionDisable = 0x0;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL1;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
}

/* Reads with FAULTMASK set and CCR.BFHFNMIGN: a bus error raised by a
   Flash double ECC error is ignored instead of entering BusFault/HardFault,
   then reported through FLASH SR1.DBECCERR / CFSR. The log region is not
   cacheable (MT_Store_MPU_Config), so the read reaches the Flash. */
uint8_t MT_Port_Read(uint32_t addr, uint8_t *dst, uint32_t len)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t  ok = 1U;

    __disable_irq();
    __HAL_FLASH_CLEAR_FLAG_BANK1(FLASH_FLAG_SNECCERR_BANK1 | FLASH_FLAG_DBECCERR_BANK1);
    SCB->CFSR = SCB_CFSR_BUSFAULTSR_Msk;
    SCB->CCR |= SCB_CCR_BFHFNMIGN_Msk;
    __DSB();
    __ISB();
    __set_FAULTMASK(1U);

    for (uint32_t i = 0; i + 4U <= len; i += 4U)
    {
        uint32_t w = *(volatile const uint32_t *)(addr + i);
        memcpy(&dst[i], &w, 4);
    }

    __DSB();
    __set_FAULTMASK(0U);
    SCB->CCR &= ~SCB_CCR_BFHFNMIGN_Msk;
    __DSB();
    __ISB();

    if (__HAL_FLASH_GET_FLAG_BANK1(FLASH_FLAG_DBECCERR_BANK1) ||
        ((SCB->CFSR & SCB_CFSR_BUSFAULTSR_Msk) != 0U))
    {
        ok = 0U;
        MT_Store_Status.ecc_double++;
    }
    if (__HAL_FLASH_GET_FLAG_BANK1(FLASH_FLAG_SNECCERR_BANK1))
    {
        MT_Store_Status.ecc_single++;       /* corrected: data is good */
    }
    __HAL_FLASH_CLEAR_FLAG_BANK1(FLASH_FLAG_SNECCERR_BANK1 | FLASH_FLAG_DBECCERR_BANK1);
    SCB->CFSR = SCB_CFSR_BUSFAULTSR_Msk;
    __set_PRIMASK(primask);
    return ok;
}

uint8_t MT_Port_Program(uint32_t addr, const uint32_t words[8])
{
    HAL_StatusTypeDef st;

    if ((addr < MT_STORE_A_BASE) || (addr >= MT_STORE_B_BASE + MT_SECTOR_BYTES) ||
        ((addr % MT_FLASHWORD_BYTES) != 0U))
    {
        return 0U;
    }
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG_BANK1(FLASH_FLAG_ALL_ERRORS_BANK1);
    /* Third argument: address of the 8-word source buffer. */
    st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr, (uint32_t)words);
    HAL_FLASH_Lock();
    return (uint8_t)(st == HAL_OK);
}

uint8_t MT_Port_Erase(uint8_t sector)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sector_error = 0U;
    HAL_StatusTypeDef st;

    if ((sector != MT_STORE_A_SECTOR) && (sector != MT_STORE_B_SECTOR))
    {
        return 0U;
    }
    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Banks        = FLASH_BANK_1;
    erase.Sector       = sector;
    erase.NbSectors    = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG_BANK1(FLASH_FLAG_ALL_ERRORS_BANK1);
    st = HAL_FLASHEx_Erase(&erase, &sector_error);
    HAL_FLASH_Lock();
    return (uint8_t)((st == HAL_OK) && (sector_error == 0xFFFFFFFFUL));
}

uint8_t MT_Port_Flash_KiB(uint16_t *kib)
{
    uint8_t raw[4];

    /* Not FLASH_SIZE: that macro turns 0x0000 / 0xFFFF into 1 MiB. */
    if (!MT_Port_Read(FLASH_SIZE_DATA_REGISTER, raw, 4U))
    {
        return 0U;
    }
    *kib = (uint16_t)(raw[0] | ((uint16_t)raw[1] << 8));
    return 1U;
}

#endif /* MT_STORE_HOST_TEST */
