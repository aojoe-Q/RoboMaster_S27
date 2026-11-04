/**
  ******************************************************************************
  * @file    multi_turn_store.h
  * @brief   Power-loss log of the seven joint logical positions, STM32H723VG
  *          internal Flash, Bank 1 Sector 6 / 7.
  *
  * Layout (the scatter file limits the image to 0x08000000..0x080BFFFF):
  *   log A  Sector 6  0x080C0000..0x080DFFFF
  *   log B  Sector 7  0x080E0000..0x080FFFFF
  *
  * One record = 256-byte body (8 Flashwords) + 32-byte commit Flashword,
  * 288 bytes, 455 slots per sector. The body is written and verified first,
  * then the commit block; a slot without a valid commit is skipped. Every
  * Flashword is programmed once; a used slot is never rewritten.
  *
  * Erase happens only in MT_Store_Boot_Maintenance(), before Arm_L_Init
  * enables the motors. At run time a full log stops appending
  * (MT_STORE_FULL); the next boot rotates to the other sector.
  ******************************************************************************
  */

#ifndef __MULTI_TURN_STORE_H
#define __MULTI_TURN_STORE_H

#include <stdint.h>

#define MT_JOINT_NUM            7U
#define MT_MASTER_PAYLOAD_LEN   30U

#define MT_FLASH_SIZE_KIB       1024U           /* STM32H723VG, read raw at 0x1FF1E880 */
#define MT_STORE_A_BASE         0x080C0000UL
#define MT_STORE_B_BASE         0x080E0000UL
#define MT_STORE_A_SECTOR       6U              /* FLASH_SECTOR_6 */
#define MT_STORE_B_SECTOR       7U              /* FLASH_SECTOR_7 */
#define MT_SECTOR_BYTES         0x00020000UL
#define MT_FLASHWORD_BYTES      32U
#define MT_BODY_BYTES           256U
#define MT_RECORD_BYTES         288U
#define MT_SLOTS_PER_SECTOR     (MT_SECTOR_BYTES / MT_RECORD_BYTES)    /* 455 */
#define MT_ROTATE_MARGIN_SLOTS  32U             /* boot rotates when fewer slots are left */

#define MT_RECORD_MAGIC         0x4D54524CUL    /* "MTRL" */
#define MT_COMMIT_MAGIC         0x434D4954UL    /* "CMIT" */
#define MT_FORMAT_VERSION       0x0007U         /* plan V8 section 6.2 layout */

/* flags */
#define MT_FLAG_SLAVE_POS_VALID   (1UL << 0)    /* slave_logical_pos holds P = p + b */
#define MT_FLAG_RESTORE_ENABLED   (1UL << 1)    /* firmware applied b when saving   */
#define MT_FLAG_SLAVE_ANCHOR      (1UL << 2)    /* slave_anchor valid (alignment)   */
#define MT_FLAG_MASTER_PAYLOAD    (1UL << 3)    /* master_payload is a received frame */

/* calibration_state, written as found when the record was saved */
#define MT_CAL_LEGACY           1U              /* restore disabled, b = 0          */
#define MT_CAL_RESTORED         2U              /* b computed from a matching record */

typedef struct
{
    uint32_t seq;
    uint32_t config_id;
    uint32_t flags;
    uint32_t slave_valid_mask;                  /* low 7 bits */
    uint32_t master_valid_mask;                 /* low 6 bits; 0 until the protocol is confirmed */
    uint32_t source_protocol_id;
    float    slave_logical_pos[MT_JOINT_NUM];
    float    slave_anchor[MT_JOINT_NUM];
    float    master_wire_float[4];
    int32_t  master_wire_int[2];
    uint8_t  master_payload[MT_MASTER_PAYLOAD_LEN];
    double   master_anchor[6];
    uint32_t mapped_axis_mask;
    uint32_t source_session;
    uint32_t master_session_flags;
    uint32_t calibration_state;
} MT_Record_t;

/* MT_Store_Status_t.state */
typedef enum
{
    MT_STORE_UNINIT = 0,
    MT_STORE_OK,
    MT_STORE_FULL,          /* both sectors used: no append until next boot      */
    MT_STORE_DISABLED,      /* raw capacity / layout check failed: never touched */
    MT_STORE_FAULT          /* program / verify / erase failed: no more writes   */
} MT_Store_State_t;

typedef struct
{
    MT_Store_State_t state;
    uint8_t  has_record;        /* a committed record was found or written       */
    uint8_t  active;            /* 0 = A, 1 = B                                  */
    uint16_t next_slot;         /* append position in the active sector          */
    uint16_t raw_flash_kib;     /* value read at 0x1FF1E880                      */
    uint32_t writes;
    uint32_t write_errors;
    uint32_t erases;
    uint32_t ecc_single;        /* corrected single-bit errors while reading     */
    uint32_t ecc_double;        /* slots rejected for a double-bit / bus error   */
    uint32_t bad_slots;         /* used slots without a valid commit             */
} MT_Store_Status_t;

extern MT_Store_Status_t MT_Store_Status;

/* Region 1 = 0x080C0000, 256 KiB, Normal, not cacheable, not bufferable,
   execute never. Called from main() USER CODE BEGIN 1, before MPU_Config()
   enables the MPU and before the D-Cache is enabled. */
void MT_Store_MPU_Config(void);

/* Check the raw Flash capacity, scan both sectors, and rotate when the active
   sector is nearly full. May erase one sector (blocks the CPU for up to a few
   seconds on this single-bank part): call only while no motor is enabled. */
void MT_Store_Boot_Maintenance(void);

/* Latest committed record. Returns 1 and fills rec, 0 if none. */
uint8_t MT_Store_Get_Latest(MT_Record_t *rec);

/* Append one record (seq is assigned here). Returns 1 on a verified commit.
   Programs 9 Flashwords; never erases. Task context, one caller. */
uint8_t MT_Store_Append(MT_Record_t *rec);

/* CRC-32/ISO-HDLC (poly 0x04C11DB7 reflected, init/xorout 0xFFFFFFFF),
   check value "123456789" -> 0xCBF43926. */
uint32_t MT_Crc32(const uint8_t *data, uint32_t len);

/* Little-endian body serialisation (no struct memcpy). Exposed for tests. */
void    MT_Record_Encode(const MT_Record_t *rec, uint8_t body[MT_BODY_BYTES]);
uint8_t MT_Record_Decode(const uint8_t body[MT_BODY_BYTES], MT_Record_t *rec);

#endif /* __MULTI_TURN_STORE_H */
