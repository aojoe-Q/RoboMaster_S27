/**
  ******************************************************************************
  * @file    Arm_USB_task.h
  * @brief   USB CDC link between the arm and ROS2 / MoveIt2.
  *
  * Data flow:
  *   CDC_Init_HS / CDC_DeInit_HS (IRQ) -> Arm_USB_OnSessionStart / End
  *   CDC_Receive_HS (IRQ)             -> Arm_USB_RxPush -> RX ring
  *   Arm_USB_Task (100 Hz)            -> parse, validate, update the PC
  *                                       command buffer, send STATE/ACK/FAULT
  *   Arm_L_Task                       -> Arm_USB_Get_PC_Target()
  *
  * The USB task never writes ArmData. PC control starts disabled, needs an
  * ENABLE in the current USB session (refused until Arm_L_Init has finished
  * and every joint has fresh CAN feedback), and drops back to disabled on
  * link timeout, session end, USB suspend or STOP.
  ******************************************************************************
  */
/*
 * 中文说明：本头文件是 USB CDC 链路的对外契约 —— 周期/超时常量、统计计数器、
 * 任务入口，以及给 Arm_L_Task 取目标位置的接口。
 *
 * 任务：Arm_USB_Task，周期 10 ms（100 Hz，ARM_USB_TASK_PERIOD_MS），在
 * Core/Src/freertos.c 里创建，优先级在创建处指定。用 vTaskDelayUntil 做绝对
 * 延时，10 ms 一拍不累积漂移；固件要求 configTICK_RATE_HZ == 1000，这样
 * xTaskGetTickCount() 的 tick 数可以直接当毫秒用。
 *
 * 上下文划分：
 *   中断上下文：Arm_USB_OnSessionStart / OnSessionEnd / RxPush（CDC 回调，
 *              只碰 rx_ring 与会话代号，不加锁、不阻塞）；
 *   任务上下文：Arm_USB_Step 及其内部函数，内部用 taskENTER_CRITICAL 临界区；
 *   Arm_L_Task：Arm_USB_Get_PC_Target / Arm_USB_PC_Disarm，自行加临界区。
 *
 * 失败与超时（故障位定义见 arm_usb_protocol.h）：
 *   300 ms 无有效帧        -> 断开 PC 控制 + ARM_FAULT_LINK_TIMEOUT
 *   关节反馈 100 ms 未更新  -> 断开 + ARM_FAULT_FEEDBACK_LOST
 *   USB 会话结束 / 挂起     -> 断开 + ARM_FAULT_USB_DISCONNECT
 * 故障位锁存到下一次成功 ENABLE；断开只清使能与目标，关节位置由 Arm_L_Task
 * 保持，不会掉力。
 */

#ifndef __ARM_USB_TASK_H
#define __ARM_USB_TASK_H

#include <stdint.h>
#include "arm_usb_protocol.h"

// 任务周期 10 ms：STATE 帧 100 Hz，与 MoveIt 的控制周期同量级
#define ARM_USB_TASK_PERIOD_MS      10U     /* STATE at 100 Hz                      */
// 300 ms 没有任何有效帧即判链路超时（约 30 个任务周期），清使能 + 丢目标
#define ARM_USB_LINK_TIMEOUT_MS     300U    /* no valid frame -> disable PC control  */
// 单条 POSITION_CMD 最长有效 300 ms，也是上位机 valid_time_ms 的上限
#define ARM_USB_CMD_TIMEOUT_MS      300U    /* max life of one POSITION_CMD          */
// valid_time_ms 下限 10 ms，防止上位机传极小值让每条指令一进来就判过期
#define ARM_USB_CMD_MIN_VALID_MS    10U     /* floor for the host valid_time_ms      */
// 收到半个帧后 20 ms 还没补齐，就丢 1 B 重新找 SOF，避免被假 SOF 长期卡住
#define ARM_USB_RX_STALL_MS         20U     /* partial frame older than this: resync */
// 关节 CAN 反馈超过 100 ms 未更新即视为失效，PC 控制随之断开（运行期看门狗）
#define ARM_USB_FEEDBACK_TIMEOUT_MS 100U    /* joint feedback older: not valid       */
// 每 1 s 让主机时基偏移基线 +1 ms，跟踪两侧晶振漂移（约 100 ppm = 0.1 ms/s）
#define ARM_USB_OFFSET_CREEP_MS     1000U   /* clock drift tracking, 1 ms per period */
// 待发 ACK 队列深度 8；溢出时丢最旧的一条
#define ARM_USB_ACK_QUEUE_LEN       8U

/* 统计计数器：32 位自由计数，100 Hz 下约 497 天才回绕一次，只增不减。
   供 DIAGNOSTIC 帧与调试器读取，不参与任何控制判断。 */
typedef struct
{
    uint32_t tx_frames;         /* frames handed to CDC               */
    uint32_t tx_busy;           /* ticks skipped, CDC still sending   */
    uint32_t tx_ack_dropped;    /* ACK queue overflow                 */
    uint32_t rx_frames;         /* frames with valid CRC              */
    uint32_t rx_crc_errors;
    uint32_t rx_resync_bytes;   /* bytes skipped while hunting SOF    */
    uint32_t rx_rejected;       /* valid CRC but refused (see ACK)    */
    uint32_t rx_stale;          /* POSITION_CMD older than the last or
                                   queued past its valid time         */
    uint32_t rx_overflow_bytes; /* bytes the RX ring refused          */
} USB_Stats_t;

extern USB_Stats_t usb_stats;  // 全局统计：任务写，诊断与调试读

/* FreeRTOS entry, created in Core/Src/freertos.c. */
/* FreeRTOS 任务入口，由 Core/Src/freertos.c 创建；周期 10 ms，优先级在创建处指定。 */
void Arm_USB_Task(void const *argument);

/* Called once by the task; exposed for the host self-test. */
/* 初始化全部状态，由任务启动时调用一次；会清空 RX 环与应答队列。 */
void Arm_USB_Init(void);

/* One task period: session check, RX parse, timeouts, TX. */
/* 一个任务周期：会话检查、RX 解析、超时与看门狗、发送。now_ms 为 tick 当毫秒。 */
void Arm_USB_Step(uint32_t now_ms);

/* USB interrupt context. */
// 下面三个函数都在 USB 中断上下文里执行：只入环或改标志，不阻塞、不加锁。
void Arm_USB_OnSessionStart(void);   /* CDC_Init_HS   */
void Arm_USB_OnSessionEnd(void);     /* CDC_DeInit_HS */
void Arm_USB_RxPush(const uint8_t *data, uint32_t len);

/* PC control enabled (ENABLE received in this session, no fault since). */
/* 查询 PC 控制是否使能（使能 + 会话一致 + 未挂起），给 Arm_L_Task 用。 */
uint8_t Arm_USB_PC_Enabled(void);

/* Arm_L_Task left ARM_MODE_PC_CONTROL (RC switch moved): drop the enable and
   the command, so re-entering the mode needs a new ENABLE from the host.
   Task context. */
/* 遥控开关离开 PC 模式时由 Arm_L_Task 调用：断使能并清目标，回模式需重新 ENABLE。 */
void Arm_USB_PC_Disarm(void);

/* Copy the PC joint targets (rad) when enabled, in the same USB session and
   the command is fresh. Returns 1 on success, 0 otherwise (targets
   untouched). Task context. */
/* 取 PC 关节目标（rad）；仅当使能、会话一致、指令未过期且 7 关节反馈都有效时返回 1。 */
uint8_t Arm_USB_Get_PC_Target(float target_rad[ARM_PROTO_JOINT_NUM]);

/* 同上，另外输出每关节速度上限（rad/s）；vmax 可为 NULL。 */
uint8_t Arm_USB_Get_PC_Command(float target_rad[ARM_PROTO_JOINT_NUM], float vmax[ARM_PROTO_JOINT_NUM]);

#endif /* __ARM_USB_TASK_H */
