/*
 * 文件：Moter_task.h
 * 机械臂输出任务对外暴露的唯一接口：任务入口 Moter_Task()。
 *
 * 任务体实现位于 Moter_task.c：7 个达妙电机（FDCAN1，0x01..0x07）的使能
 * 状态机 + MIT 周期输出，节拍由 ARM_OUTPUT_PERIOD_MS（4 ms）决定。
 * 状态机内部结构（ArmOutputDiagnostics / Arm_Control_Snapshot）放在
 * arm_output.h，本头文件不重复暴露，避免上层依赖实现细节。
 */
#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H





/* Moter_Task: 机械臂输出任务入口。
 * 参数 argument：FreeRTOS 任务参数，本任务不使用，传 NULL 即可。
 * 返回：无（任务函数不返回）。
 * 调用者：osThreadDef/osThreadCreate 创建后由调度器调用。
 * 注意：任务体用 vTaskDelayUntil 维持 4 ms 固定节拍，内部不阻塞等待事件；
 *       创建时必须给出足够的栈（状态机会用 7 路反馈快照）。 */
void Moter_Task(void const * argument);

#endif
