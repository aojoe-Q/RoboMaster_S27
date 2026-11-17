/* arm_output.h —— 机械臂输出层接口（使能状态机 + 周期发帧）
 *
 * 位置：控制层（Arm_L_Task）只负责算“想要什么”，本层负责“怎么安全地发出去”。
 * 数据流：Arm_Control_Publish() 写入一整帧快照 -> Arm_Output_Step() 每
 *         ARM_OUTPUT_PERIOD_MS 到点取出快照，按每路使能状态发 MIT 帧。
 * 关键点：反馈超时会跳过该关节（offline_skip），总线异常会置 bus_inhibit，
 *         使能失败可请求重试；所有异常都只影响本层，不阻塞控制层计算。
 */
#ifndef ARM_OUTPUT_H
#define ARM_OUTPUT_H
#include <stdint.h>
#define ARM_OUTPUT_PERIOD_MS 4U // 输出步进周期 4 ms（250 Hz），即相邻两帧 MIT 指令的最小间隔
#define ARM_OUTPUT_FB_TIMEOUT_MS 200U // 反馈超时 200 ms：超时判该关节离线，跳过发帧并计入 offline_skip
typedef struct { // 一帧控制快照：7 路关节的 MIT 指令与发布时刻，由控制层整体写入
    float pos[7], kp[7], kd[7], torque[7]; // 下标 0..6 与电机 ID 1..7 一一对应；pos 目标位置，kp/kd 增益，torque 前馈力矩
    uint32_t tick_ms; // 快照发布时刻（ms），输出层据此判断数据是否新鲜
} Arm_Control_Snapshot;
typedef enum { EN_WAIT_BOOT, EN_SEND, EN_OK, EN_FAIL } ArmEnableState; // 单路使能状态机：WAIT_BOOT 上电等待 -> SEND 已发使能帧 -> OK 已确认 / FAIL 失败
typedef struct { // 输出层诊断计数：只读观测用，不参与控制决策
    uint32_t offline_skip[7], fifo_defer[7], first_feedback_ms[7]; // offline_skip 因离线跳过的拍数；fifo_defer 因发送 FIFO 无空间而延后的次数；first_feedback_ms 首次收到反馈的时刻
    uint8_t state[7], attempts[7]; // state 每路使能状态（ArmEnableState）；attempts 该路使能尝试次数
    uint8_t startup_failed, bus_inhibit; // startup_failed 启动阶段失败标志；bus_inhibit 总线级禁发（如 bus-off）
} ArmOutputDiagnostics;
extern ArmOutputDiagnostics arm_output_diag; // 全局诊断实例：BSP/输出层写，其他层只读
void Arm_Control_Publish(const Arm_Control_Snapshot *cmd); // 控制层 -> 输出层：覆盖式发布一帧控制快照
Arm_Control_Snapshot Arm_Control_GetSnapshot(void); // 输出层 -> 控制层：取最近一次快照（回读/调试用）
void Arm_Output_Start(void); // 请求启动使能流程（上电或整机复位后调用）
void Arm_Output_RequestRetry(void); /* explicit DOWN/DOWN edge, after zero-output publication */
uint8_t Arm_Output_AllEnabled(void); // 7 路是否全部进入 EN_OK
void Arm_Output_Step(uint32_t now_ms); // 周期步进：推进使能状态机，按周期发送 MIT 帧
#endif // ARM_OUTPUT_H
