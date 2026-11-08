/* arm_motion.h —— 上位机（PC）控制链路的单关节运动插补
 *
 * 背景：PC 下发的目标位置可能跳变，若直接赋给 send_set 会造成力矩冲击。
 *       本文件提供带“速度上限 + 加速度上限 + 制动包络”的一维插补，
 *       每拍把目标逼近一点，且保证在剩余距离内能刹住、不越过目标。
 * 用法：每个关节保存一个 ArmMotion 状态，周期调用 Arm_Motion_Step() 推进。
 * 注意：这里只是软限幅，固件的硬限速/限位仍然生效，二者叠加不冲突。
 */
#ifndef ARM_MOTION_H
#define ARM_MOTION_H
#include <math.h>
/* 以下三个宏是 PC 控制链路专用的保守默认值，调试期间可以调低。 */
/* Conservative joint-space defaults; commissioning may lower these values.
 * A positive host limit is a cruise ceiling; a decrease brakes at AMAX.
 * Hard firmware speed/acceleration limits always remain in force. */
#define ARM_PC_VMAX_RAD_S 0.5f // 关节空间速度上限 0.5 rad/s
#define ARM_PC_AMAX_RAD_S2 1.0f // 加速度上限 1 rad/s^2（同时用于制动包络）
#define ARM_PC_DT_MAX_S 0.016f // 单步 dt 上限 16 ms：任务抖动/丢帧导致间隔变大时，防止积分一步跳太远
typedef struct { float position, velocity; } ArmMotion;
/* Arm_Motion_Step() —— 单关节限速限加速插补一步
 * 参数：s       运动状态（ArmMotion{position,velocity}，会被就地更新）
 *       target  目标位置(rad)
 *       vmax    本拍允许的巡航速度上限(rad/s)，内部再钳到 ARM_PC_VMAX_RAD_S
 *       dt      本拍时间步长(s)，非法值返回原位置，超过 16 ms 时按 16 ms 计
 * 返回：积分后的新位置(rad)
 * 调用：控制层每个控制周期对每个受控关节调用一次。
 */
static inline float Arm_Motion_Step(ArmMotion *s, float target, float vmax, float dt)
{
    if (dt <= 0.0f) return s->position; // dt 非法：本拍不推进，保持原位
    if (dt > ARM_PC_DT_MAX_S) dt = ARM_PC_DT_MAX_S; // dt 上限钳位
    if (vmax > ARM_PC_VMAX_RAD_S) vmax = ARM_PC_VMAX_RAD_S; // 速度上限再钳位，避免调用方传入过大的值
    float delta = target - s->position;
    float dv = ARM_PC_AMAX_RAD_S2 * dt;
    /* Discrete braking envelope leaves room for the next velocity sample. */
    float brake = sqrtf(dv * dv + 2.0f * ARM_PC_AMAX_RAD_S2 * fabsf(delta)) - dv; // 离散制动包络：本拍允许的最大速度，保证剩余距离内能刹停（预留一拍余量）
    float desired = copysignf(fminf(vmax, brake), delta); // 取 vmax 与 brake 的较小者，方向跟随 delta
    float change = desired - s->velocity; // 实际速度朝 desired 靠拢，单拍变化量不超过 dv
    if (change > dv) change = dv; // 正向加速度限幅
    if (change < -dv) change = -dv; // 负向加速度限幅
    s->velocity += change;
    s->position += s->velocity * dt; // 位置积分
    return s->position; // 返回插补后的新位置
}
#endif // ARM_MOTION_H
