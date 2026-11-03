#include "user_lib.h"

void arm_data_ramp_init(ramp_function_source_t *ramp_source_type,
                        fp32 frame_period, fp32 now)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->input        = 0.0f;
    ramp_source_type->out          = now;
}

void arm_ramp_calc(ramp_function_source_t *ramp_source_type,
                   fp32 input, fp32 max)
{
    ramp_source_type->max_value = max;
    ramp_source_type->input     = input;

    if (ramp_source_type->max_value > ramp_source_type->out)
    {
        ramp_source_type->out +=
            ramp_source_type->input * ramp_source_type->frame_period;
        if (ramp_source_type->out > ramp_source_type->max_value)
        {
            ramp_source_type->out = ramp_source_type->max_value;
        }
    }
    else if (ramp_source_type->max_value < ramp_source_type->out)
    {
        ramp_source_type->out -=
            ramp_source_type->input * ramp_source_type->frame_period;
        if (ramp_source_type->out < ramp_source_type->max_value)
        {
            ramp_source_type->out = ramp_source_type->max_value;
        }
    }
}
