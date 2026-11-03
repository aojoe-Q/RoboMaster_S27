#ifndef _USER_LIB_H
#define _USER_LIB_H

/*
 * Trimmed copy of the S27_ARM_H7 user_lib: only the ramp helpers used by
 * Arm_L_task.c are required here. The original also contained OLS helpers and
 * a duplicate set of integer typedefs, which are intentionally left out.
 */

#include <stdint.h>
#include "struct_typedef.h"

typedef struct
{
    float input;
    float out;
    float min_value;
    float max_value;
    float frame_period;
} ramp_function_source_t;

void arm_data_ramp_init(ramp_function_source_t *ramp_source_type,
                        fp32 frame_period, fp32 now);
void arm_ramp_calc(ramp_function_source_t *ramp_source_type,
                   fp32 input, fp32 max);

#endif /* _USER_LIB_H */
