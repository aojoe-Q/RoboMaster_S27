#include "multi_turn_store.h"
#include "arm_output.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include "master_link.h"
#define LOG_BASE 0x080C0000UL
#define LOG_SIZE 0x00040000UL
volatile uint32_t mt_flash_fault,mt_flash_fault_address,mt_flash_read_active;
extern const uint8_t image_load_limit[] __asm("Load$$LR$$LR_IROM1$$Limit");
static bool layout(void)
{
    uint32_t old,rbar,rasr;
    /* 绝不用 FLASH_SIZE 宏：读失败时它会静默退回默认值。 */
    if(*(volatile const uint16_t *)0x1FF1E880UL!=1024U ||
       (DBGMCU->IDCODE&0xfffU)!=0x483U || (uintptr_t)image_load_limit>LOG_BASE) return false;
    old=MPU->RNR; MPU->RNR=1; rbar=MPU->RBAR; rasr=MPU->RASR; MPU->RNR=old;
    return (MPU->CTRL&MPU_CTRL_ENABLE_Msk) && (rbar&MPU_RBAR_ADDR_Msk)==LOG_BASE &&
           (rasr&MPU_RASR_ENABLE_Msk) && ((rasr&MPU_RASR_SIZE_Msk)>>MPU_RASR_SIZE_Pos)==17U &&
           ((rasr&MPU_RASR_TEX_Msk)>>MPU_RASR_TEX_Pos)==1U &&
           !(rasr&(MPU_RASR_C_Msk|MPU_RASR_B_Msk|MPU_RASR_SRD_Msk)) &&
           (rasr&MPU_RASR_XN_Msk) && ((rasr&MPU_RASR_AP_Msk)>>MPU_RASR_AP_Pos)==3U;
}
static bool maintenance(void *ctx)
{
    (void)ctx;
    return !mt_flash_fault && __get_IPSR()==0 && layout() && ArmOutput_Stopped();
}
void MT_H7_FaultCapture(void)
{
    /* 只允许从致命异常处理中调用。禁止跳过出错指令，
       也禁止从事不相关的异常里返回；原有的死循环保留。 */
    if(mt_flash_read_active) {
        mt_flash_fault=FLASH->SR1 | 0x80000000U;
        mt_flash_fault_address=FLASH->ECC_FA1;
        ArmOutput_StorageFault();
    }
}
static bool read_flash(void *ctx,uint32_t offset,uint8_t *out,uint32_t n)
{
    uint32_t i,errors; const volatile uint8_t *src;
    (void)ctx;
    if(mt_flash_fault || offset>LOG_SIZE || n>LOG_SIZE-offset || !layout()) return false;
    errors=FLASH->SR1&(FLASH_SR_SNECCERR|FLASH_SR_DBECCERR);
    if(errors) { mt_flash_fault=errors; ArmOutput_StorageFault(); return false; }
    mt_flash_read_active=1; __DSB();
    src=(const volatile uint8_t *)(LOG_BASE+offset);
    for(i=0;i<n;i++) out[i]=src[i];
    __DSB();
    errors=FLASH->SR1&(FLASH_SR_SNECCERR|FLASH_SR_DBECCERR);
    mt_flash_read_active=0;
    if(errors) {
        mt_flash_fault=errors; mt_flash_fault_address=FLASH->ECC_FA1;
        ArmOutput_StorageFault(); return false;
    }
    return true;
}
static bool program(void *ctx,uint32_t offset,const uint8_t *data)
{
    __attribute__((aligned(32))) uint32_t word[8];
    HAL_StatusTypeDef result;
    if((offset&31U)||offset>LOG_SIZE-32 || !maintenance(ctx)) return false;
    memcpy(word,data,32);
    if(HAL_FLASH_Unlock()!=HAL_OK) return false;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);
    result=HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,LOG_BASE+offset,(uint32_t)word);
    HAL_FLASH_Lock(); __DSB(); __ISB();
    return result==HAL_OK;
}
static bool erase(void *ctx,unsigned sector)
{
    FLASH_EraseInitTypeDef e={0}; uint32_t error;
    HAL_StatusTypeDef result;
    if(sector>1 || !maintenance(ctx)) return false;
    e.TypeErase=FLASH_TYPEERASE_SECTORS; e.Banks=FLASH_BANK_1;
    e.Sector=6+sector; e.NbSectors=1;
    e.VoltageRange=FLASH_VOLTAGE_RANGE_3;
    if(HAL_FLASH_Unlock()!=HAL_OK) return false;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);
    result=HAL_FLASHEx_Erase(&e,&error);
    HAL_FLASH_Lock(); __DSB(); __ISB();
    return result==HAL_OK;
}
/* 打开硬件日志：先校验分区布局与 MPU 配置，再扫描已有记录。 */
MT_Result MT_H7_Open(MT_Store *s,uint32_t config)
{
    MT_Backend io={0,read_flash,program,erase,maintenance};
    if(!maintenance(0)) return MT_LOCKED;
    return MT_Scan(s,&io,config);
}

volatile uint32_t mt_maintenance_request;
volatile MT_Result mt_maintenance_result=MT_LOCKED;
MT_Record mt_maintenance_latest;
static MT_Store maintenance_store;
/* 维护窗口服务：只在无故障、任务上下文且输出已停时执行一次请求。 */
void MT_Maintenance_Service(void)
{
    MasterRawSnapshot master;
    MT_Record record={0};
    uint32_t request=mt_maintenance_request;
    if(!request) return;
    mt_maintenance_request=0; /* 一次请求只尝试一次，输入一直保持也不重试。 */
    if(request>3) { mt_maintenance_result=MT_INVALID; return; }
    /* config 0 仅用于诊断，绝不用于恢复/标定。 */
    mt_maintenance_result=MT_H7_Open(&maintenance_store,0);
    if(mt_maintenance_result!=MT_OK && mt_maintenance_result!=MT_EMPTY) return;
    if(request==1) {
        if(maintenance_store.found) mt_maintenance_latest=maintenance_store.latest;
        else memset(&mt_maintenance_latest,0,sizeof(mt_maintenance_latest));
        return;
    }
    if(MasterLink_Read(&master) && (uint32_t)(HAL_GetTick()-master.rx_tick)<100)
        memcpy(record.master_payload,master.payload,30);
    /* 不编造任何主手类型或从手逻辑位置。 */
    mt_maintenance_result=MT_Append(&maintenance_store,&record,request==3);
    if(mt_maintenance_result==MT_OK) mt_maintenance_latest=maintenance_store.latest;
}
