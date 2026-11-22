/* user_lib.h：机械臂斜坡辅助函数的声明与数据结构。
 * 这是 S27_ARM_H7 user_lib 的裁剪版，只保留 Arm_L_task.c 用到的
 * ramp 接口；函数语义见 user_lib.c。 */
#ifndef _USER_LIB_H
#define _USER_LIB_H

/*
 * Trimmed copy of the S27_ARM_H7 user_lib: only the ramp helpers used by
 * Arm_L_task.c are required here. The original also contained OLS helpers and
 * a duplicate set of integer typedefs, which are intentionally left out.
 */

#include <stdint.h>
#include "struct_typedef.h"

/* ramp_function_source_t：斜坡发生器状态。
 * input 为每拍增量基准（与 frame_period 相乘），out 为当前输出，
 * max_value 为本周期目标；frame_period 本工程取 1；min_value 未使用。 */
typedef struct
{
    float input;
    float out;
    float min_value;
    float max_value;
    float frame_period;
} ramp_function_source_t;

/* 斜坡接口：初始化 + 每周期推进一步；详细参数说明见 user_lib.c。 */
void arm_data_ramp_init(ramp_function_source_t *ramp_source_type,
                        fp32 frame_period, fp32 now);
void arm_ramp_calc(ramp_function_source_t *ramp_source_type,
                   fp32 input, fp32 max);

#endif /* _USER_LIB_H */
