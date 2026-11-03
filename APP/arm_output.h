#ifndef ARM_OUTPUT_H
#define ARM_OUTPUT_H
#include "arm_gate.h"
#include "arm_control.h"
typedef enum { ARM_NEED_REFERENCE=1, ARM_MAINTENANCE=2, ARM_STORAGE_FAULT=3 } ArmOutputReason;
extern volatile ArmOutputReason arm_output_reason;
/* 在实测周期、限位和拨杆物理左右之前，不提供解锁或重建基准的接口。 */
void ArmOutput_Publish(void);
void ArmOutput_Send(unsigned axis);
void ArmOutput_Feedback(unsigned axis,float raw,uint8_t state,uint32_t tick);
bool ArmOutput_Stopped(void);
bool ArmOutput_IsLocked(void);
void ArmOutput_StorageFault(void);
#endif
