#ifndef MULTI_TURN_STORE_H
#define MULTI_TURN_STORE_H
#include <stdint.h>
#include <stdbool.h>
#define MT_SECTOR_SIZE 131072U
#define MT_RECORD_SIZE 288U
#define MT_SLOTS 455U
#define MT_FORMAT 8U
typedef enum { MT_OK,MT_EMPTY,MT_FULL,MT_INCOMPATIBLE,MT_IO,MT_INVALID,MT_LOCKED } MT_Result;
typedef struct {
    uint32_t seq,config_id,flags,slave_valid_mask,master_valid_mask,source_protocol_id;
    float slave_logical_pos[7],slave_anchor[7],master_wire_float[4];
    int32_t master_wire_int[2];
    uint8_t master_payload[30];
    double master_anchor[6];
    uint32_t mapped_axis_mask,source_session,master_session_flags,calibration_state;
} MT_Record;
typedef struct {
    void *ctx;
    bool (*read)(void *,uint32_t,uint8_t *,uint32_t);
    bool (*program)(void *,uint32_t,const uint8_t *); /* 恰好一个 32 字节 Flashword */
    bool (*erase)(void *,unsigned);
    bool (*maintenance)(void *);
} MT_Backend;
typedef struct {
    MT_Backend io;
    MT_Record latest;
    uint32_t config_id;
    unsigned next[2],active;
    bool found,scanned;
} MT_Store;
uint32_t MT_CRC32(const uint8_t *data,uint32_t size);
void MT_Encode(const MT_Record *r,uint8_t out[MT_RECORD_SIZE]);
bool MT_Decode(const uint8_t in[MT_RECORD_SIZE],MT_Record *r);
MT_Result MT_Scan(MT_Store *s,const MT_Backend *io,uint32_t config);
MT_Result MT_Append(MT_Store *s,const MT_Record *r,bool allow_erase);
/* 硬件入口：只能在明确停机后的维护窗口调用，绝不在启动时自动调用。 */
MT_Result MT_H7_Open(MT_Store *s,uint32_t config);
/* 调试器维护邮箱，由 Arm 任务消费一次：
 * 1=扫描，2=追加诊断记录（不擦除），3=追加并在需要时换区擦除。
 * 诊断记录的所有基准位/主手有效位都为 0。 */
extern volatile uint32_t mt_maintenance_request;
extern volatile MT_Result mt_maintenance_result;
extern MT_Record mt_maintenance_latest;
void MT_Maintenance_Service(void);
extern volatile uint32_t mt_flash_fault,mt_flash_fault_address,mt_flash_read_active;
void MT_H7_FaultCapture(void);
#endif
