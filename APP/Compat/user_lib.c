/* user_lib.c：机械臂斜坡（ramp）辅助函数。
 * 用途：让输出按固定周期、固定增量逐步逼近目标，避免关节指令阶跃。
 * 调用者：Arm_L_task.c；实际增量 = input * frame_period，本工程把
 *       frame_period 初始化为 1，因此 input 直接就是每拍步长（rad/拍）。
 * 约束：input 为正时 out 只会朝 max_value 靠近并钳在目标上，不会越过；
 *       本裁剪版未使用 min_value（字段保留但计算不参与）。 */
#include "user_lib.h"

/* arm_data_ramp_init：初始化斜坡发生器。
 * 参数：ramp_source_type 发生器对象；frame_period 周期系数（本工程填 1）；
 *       now 初始输出值（通常传当前关节角）。
 * 返回：无；input 清零、out 置为 now，使第一次计算从当前位置开始。 */
void arm_data_ramp_init(ramp_function_source_t *ramp_source_type,
                        fp32 frame_period, fp32 now)
{
    ramp_source_type->frame_period = frame_period; // 周期系数；本工程填 1
    ramp_source_type->input        = 0.0f; // 初始步长为 0
    ramp_source_type->out          = now; // 从当前值开始
}

/* arm_ramp_calc：按固定步长把 out 推向 max。
 * 参数：ramp_source_type 发生器；input 每拍增量基准（实际增量 =
 *       input * frame_period；本工程 frame_period=1，故 input 即每拍步长）；
 *       max 本周期目标值。
 * 返回：无；结果写回 ramp_source_type->out。
 * 注意：out 到目标后钳位，不会过冲；若 input 为负，out 会朝远离目标的
 *       方向移动（调用方应传正值）。 */
void arm_ramp_calc(ramp_function_source_t *ramp_source_type,
                   fp32 input, fp32 max)
{
    ramp_source_type->max_value = max; // 本周期目标值
    ramp_source_type->input     = input;

    if (ramp_source_type->max_value > ramp_source_type->out) // 未到目标：向目标推进
    {
        ramp_source_type->out +=
            ramp_source_type->input * ramp_source_type->frame_period;
        if (ramp_source_type->out > ramp_source_type->max_value)
        {
            ramp_source_type->out = ramp_source_type->max_value;
        }
    }
    else if (ramp_source_type->max_value < ramp_source_type->out) // out 高于目标：向下回调
    {
        ramp_source_type->out -=
            ramp_source_type->input * ramp_source_type->frame_period;
        if (ramp_source_type->out < ramp_source_type->max_value)
        {
            ramp_source_type->out = ramp_source_type->max_value;
        }
    }
}
