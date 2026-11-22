/*
 * 模块说明：Moter_task.c —— 机械臂 7 关节输出/使能任务
 * ------------------------------------------------------------------
 * 运行环境：STM32H723 + FreeRTOS。任务优先级为 osPriorityRealtime，
 *           周期由 ARM_OUTPUT_PERIOD_MS 决定（当前 = 4 ms，即 250 Hz），
 *           Moter_Task 用 vTaskDelayUntil 维持绝对节拍。
 *
 * 职责：
 *   1) 维护控制指令快照：Arm_Control_Publish() 写、Arm_Control_GetSnapshot() 读，
 *      两侧都在 taskENTER_CRITICAL() 内整体复制，避免结构体撕裂；
 *   2) 在 Arm_Output_Step() 中推进 7 个达妙电机（FDCAN1，ID 0x01..0x07）的
 *      使能状态机：EN_WAIT_BOOT -> EN_SEND -> EN_OK，异常进入 EN_FAIL；
 *   3) 发送前检查 HAL_FDCAN_GetTxFifoFreeLevel() >= 2，始终给其他有界 CAN 流量
 *      留 1 个 TX 槽位；7 个关节按 rotating_first 轮转，避免 FIFO 饥饿；
 *   4) 监视总线健康（can_health[0]）与反馈超时（ARM_OUTPUT_FB_TIMEOUT_MS = 200 ms），
 *      总线异常时置 bus_inhibit，只允许“新鲜的无动力指令”清故障并重试。
 *
 * 关键约束：
 *   - FDCAN1 为 classic CAN 1 Mbit/s，单帧 8 字节；每关节每圈至多一次发送尝试（EN_OK 1 帧，握手时 2 帧）；
 *   - 指令快照超过 100 ms 未刷新时，回落到“实测位置 + 零增益/零力矩”发送，
 *     防止上位机失联后继续按旧指令出力；
 *   - 使能帧 ID 就是电机 ID（模式号 0），数据为 FF FF FF FF FF FF FF FC。
 */
#include "Moter_task.h"
#include "arm_output.h"
#include "can_health.h"
#include "Arm_L_task.h"
#include "DM_motor.h"
#include "can_bsp.h"
#include "fdcan.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* published：最近一次由控制任务发布的 7 关节指令快照。
 * 写侧（Arm_Control_Publish）与读侧（Arm_Control_GetSnapshot）都整体关中断拷贝，
 * 保证 7 路 pos/kp/kd/torque 与 tick_ms 来自同一帧。 */
static Arm_Control_Snapshot published;
/* arm_output_diag：输出状态机诊断信息（状态、重试次数、FIFO 推迟次数等），
 * 由 USB 遥测/上层逻辑读取，便于现场判断某个关节卡在哪个阶段。 */
ArmOutputDiagnostics arm_output_diag;
/* start_requested / retry_requested：跨任务请求标志，只置位、由本任务消费；
 * 用 volatile 防止编译器把轮询优化掉。 */
static volatile uint8_t start_requested, retry_requested;
/* 每条关节状态机的簿记数组（索引 0..6 对应 ID 0x01..0x07）：
 *   baseline[i]  —— 发使能帧时的 update_count，用于判定“使能后是否收到新反馈”；
 *   sent_at[i]   —— 首次发使能帧的时刻（ms），用于判定反馈是否晚于使能帧；
 *   last_tx[i]   —— 最近一次发使能帧的时刻，用于 20 ms 节流；
 *   seen_epoch   —— 最近一次看到的 CAN 恢复代数（can_recovery_epoch 的快照）。 */
static uint32_t baseline[7], sent_at[7], last_tx[7], seen_epoch;
/* started：状态机总开关，收到 Arm_Output_Start() 后置 1 且不再清零；
 * rotating_first：每圈轮转的起始关节，保证 FIFO 紧张时各关节公平发送。 */
static uint8_t started, rotating_first;

/* Arm_Control_Publish: 发布一帧 7 关节控制指令快照。
 * 参数 cmd：上层填好的快照（pos/kp/kd/torque 各 7 路 + tick_ms）。
 * 返回：void。
 * 调用者：Arm_L_task 的控制循环（任务上下文）。
 * 注意：内部 taskENTER_CRITICAL() 整体拷贝，临界区只覆盖一次结构体复制；
 *       调用者需保证 7 路数组都已赋值，本函数不做默认值填充。 */
void Arm_Control_Publish(const Arm_Control_Snapshot *cmd)
{
    taskENTER_CRITICAL();
    published = *cmd;
    taskEXIT_CRITICAL();
}
/* Arm_Control_GetSnapshot: 取回最近一次发布的指令快照副本。
 * 返回：Arm_Control_Snapshot 结构体副本（可在临界区外安全使用）。
 * 调用者：Arm_Output_Step 每圈调用一次。
 * 注意：返回副本而非指针，调用者修改它不会影响 published。 */
Arm_Control_Snapshot Arm_Control_GetSnapshot(void)
{
    Arm_Control_Snapshot cmd;
    taskENTER_CRITICAL();
    cmd = published;
    taskEXIT_CRITICAL();
    return cmd;
}
/* Arm_Output_Start: 请求启动输出状态机。
 * 注意：这里只置标志，真正启动发生在下一次 Arm_Output_Step（任务上下文），
 *       避免在中断/初始化阶段直接操作状态机。 */
void Arm_Output_Start(void) { start_requested = 1U; }
/* Arm_Output_RequestRetry: 请求清除总线抑制（bus_inhibit）。
 * 上层在“明确切到无动力指令”后调用；是否接受由 Arm_Output_Step 判定：
 * 只认同新鲜（≤100 ms）且 kp/kd/torque 全零的指令。 */
void Arm_Output_RequestRetry(void) { retry_requested = 1U; }
/* Arm_Output_AllEnabled: 判断 7 个关节是否全部使能且反馈新鲜。
 * 返回：1 = 全部 EN_OK、电机 status==1、反馈年龄 ≤100 ms；否则 0。
 * 调用者：Arm_L_task 用它决定能否进入受控运动/夹爪零点确认。
 * 注意：函数内只取一次 now，7 路判定基于同一时刻，避免边界抖动。 */
uint8_t Arm_Output_AllEnabled(void)
{
    uint8_t ok = started && !can_health[0].blocked && !arm_output_diag.bus_inhibit; // 总线抑制/未启动/任一关节未使能都视为未就绪
    uint32_t now = xTaskGetTickCount()*portTICK_PERIOD_MS;
    taskENTER_CRITICAL();
    /* 逐关节检查：状态机 EN_OK、DM 反馈 status==1（已使能）、反馈年龄 ≤100 ms；
     * 任何一路不满足就整体判为未就绪。 */
    for (unsigned i=0; i<7; ++i)
        if (arm_output_diag.state[i] != EN_OK || joint_L_motor[i].status != 1U ||
            now-joint_L_motor[i].update_tick > 100U) ok = 0U;
    taskEXIT_CRITICAL();
    return ok;
}
/* restart: 把单个关节打回“等待启动”状态，准备重新握手。
 * 参数 i：关节索引 0..6；now：当前毫秒时间；fb：该关节的反馈快照。
 * 注意：baseline 记录当前 update_count，只有之后计数增加才认为“使能后收到新反馈”；
 *       last_tx 预置为 now-20，让第一帧使能命令立即通过 20 ms 节流。 */
static void restart(unsigned i, uint32_t now, const joint_motor_measure_t *fb)
{
    arm_output_diag.state[i] = EN_WAIT_BOOT;
    arm_output_diag.attempts[i] = 0U;
    baseline[i] = fb->update_count;
    last_tx[i] = now - 20U;
}
/* ==================== 状态检查类函数 ====================*/

/* 检测 CAN 总线恢复：若 can_recovery_epoch 变化，标记总线抑制 */
static void Arm_CheckBusRecovery(uint32_t now)
{
    (void)now; /* 参数预留，当前未使用 */
    if (seen_epoch != can_recovery_epoch) {
        seen_epoch = can_recovery_epoch;
        /* An authorization from before this CAN fault cannot rearm its new epoch. */
        retry_requested = 0U;
        arm_output_diag.bus_inhibit = 1U;
        for (unsigned i = 0; i < 7; ++i) {
            arm_output_diag.state[i] = EN_FAIL;
        }
    }
}

/* 判定 retry 请求是否有效（需新鲜指令 + 全零增益/力矩） */
static uint8_t Arm_CheckRetryValid(const Arm_Control_Snapshot *cmd, uint32_t now)
{
    /* 指令超过 100ms 视为过期 */
    if (now - cmd->tick_ms > 100U) return 0U;

    /* 检查所有关节增益/力矩是否为零 */
    for (unsigned i = 0; i < 7; ++i) {
        if (cmd->kp[i] != 0.0f || cmd->kd[i] != 0.0f || cmd->torque[i] != 0.0f) {
            return 0U;
        }
    }
    return 1U;
}

/* 检查单关节反馈超时或电机状态异常 */
static void Arm_CheckFeedbackTimeout(unsigned i, const joint_motor_measure_t *fb, uint32_t now)
{
    if (arm_output_diag.state[i] != EN_OK) return;

    uint32_t age = now - fb->update_tick;
    if (age > ARM_OUTPUT_FB_TIMEOUT_MS || fb->status != 1U) {
        arm_output_diag.state[i] = EN_FAIL;
    }
}

/* 检查 TX FIFO 是否有足够空间（需 >= 2 个槽位） */
static uint8_t Arm_CheckFifoSpace(void)
{
    return (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) >= 2U) ? 1U : 0U;
}

/* ==================== 握手和发送类函数 ====================*/

/* 检查使能握手是否成功 */
static uint8_t Arm_CheckEnableHandshake(unsigned i, const joint_motor_measure_t *fb, uint32_t now)
{
    if (arm_output_diag.state[i] != EN_SEND) return 0U;

    uint32_t age = now - fb->update_tick;

    /* 握手成功条件：反馈 ID 匹配、status==1、update_count 有增加、年龄≤100ms、时间戳晚于使能 */
    if (fb->motorid == i + 1U &&
        fb->status == 1U &&
        fb->update_count != baseline[i] &&
        age <= 100U &&
        (int32_t)(fb->update_tick - sent_at[i]) > 0) {
        return 1U;
    }
    return 0U;
}

/* 发送使能帧 + 零增益 MIT 帧 */
static void Arm_SendEnableFrame(unsigned i, const joint_motor_measure_t *fb, uint32_t now)
{
    /* 20ms 节流：避免同一关节把 TX FIFO 打满 */
    if (now - last_tx[i] < 20U) return;

    /* FIFO 空间检查 */
    if (!Arm_CheckFifoSpace()) {
        arm_output_diag.fifo_defer[i]++;
        return;
    }

    last_tx[i] = now;
    arm_output_diag.attempts[i]++;

    /* DM 使能帧：FF FF FF FF FF FF FF FC */
    uint8_t enable[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

    if (canx_send_data(&hfdcan1, (uint16_t)(i + 1U), enable, 8U) == HAL_OK) {
        /* 首次发送：记录 baseline 和时刻 */
        if (arm_output_diag.state[i] == EN_WAIT_BOOT) {
            baseline[i] = fb->update_count;
            sent_at[i] = now;
        }

        arm_output_diag.state[i] = EN_SEND;

        /* 紧跟一帧零增益 MIT 指令 */
        mit_ctrl(&hfdcan1, (uint16_t)(i + 1U), 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
}

/* 发送控制帧（MIT 模式） */
static void Arm_SendControlFrame(unsigned i, const joint_motor_measure_t *fb,
                                 const Arm_Control_Snapshot *cmd, uint32_t now)
{
    /* FIFO 空间检查 */
    if (!Arm_CheckFifoSpace()) {
        arm_output_diag.fifo_defer[i]++;
        return;
    }

    /* 指令超时判定 */
    if (now - cmd->tick_ms > 100U) {
        /* 超时：回落到实测位置 + 零增益/力矩 */
        mit_ctrl(&hfdcan1, (uint16_t)(i + 1U), fb->position, 0.0f, 0.0f, 0.0f, 0.0f);
    } else {
        /* 正常：下发快照指令 */
        mit_ctrl(&hfdcan1, (uint16_t)(i + 1U), cmd->pos[i], 0.0f,
                 cmd->kp[i], cmd->kd[i], cmd->torque[i]);
    }
}
// * 参数 now：当前毫秒时间（xTaskGetTickCount() * portTICK_PERIOD_MS）。
// * 流程：总线健康 -> 拷贝反馈 -> 启动/总线恢复判定 -> retry/抑制判定 ->
// *       全轴状态判定 -> 故障统一清零 -> 逐关节发送。
// * 注意：本函数是唯一向 FDCAN1 发关节帧的地方，不可重入；
// *       所有发送前都检查 TX FIFO 至少剩 2 个槽位。 */
void Arm_Output_Step(uint32_t now)
{
    /* 本地反馈快照：一次临界区拷贝 7 路，后续循环不再反复关中断。 */
    joint_motor_measure_t fb[7];
    // 先推进 FDCAN 健康状态机：更新 TEC/REC、bus-off/恢复计数与 blocked 标志
    BSP_CAN_HealthStep(now);
    // 拷贝 7 路反馈到本地，避免与接收路径并发写同一结构体
    taskENTER_CRITICAL();
    memcpy(fb, joint_L_motor, sizeof(fb));
    taskEXIT_CRITICAL();
    /* 启动闸门：未收到 Arm_Output_Start 前不发任何帧，防止上电瞬间机械臂抽动。 */
    if (!started) {
        // 还没有启动请求：本圈什么都不做
        if (!start_requested) return;
        // 首次启动：打开总开关，并记录当前 CAN 恢复代数
        started = 1U;
        seen_epoch = can_recovery_epoch;
        // 7 个关节全部从 EN_WAIT_BOOT 开始，逐个进入使能握手
        for (unsigned i=0; i<7; ++i) restart(i, now, &fb[i]);
    }
    /* CAN 恢复检测 */
    Arm_CheckBusRecovery(now);
    // 总线健康认为当前不可发送（错误计数过高或尚未恢复）：整圈直接退出，不累加 attempts
    if (can_health[0].blocked) return;
    // 取指令快照（内部临界区）；后面用 cmd.tick_ms 判断指令是否新鲜
    Arm_Control_Snapshot cmd = Arm_Control_GetSnapshot();
    /* Retry 判定 */
    if (retry_requested) {
        retry_requested = 0U;
        if (Arm_CheckRetryValid(&cmd, now)) {
            arm_output_diag.bus_inhibit = 0U;
            for (unsigned i = 0; i < 7; ++i) {
                if (arm_output_diag.state[i] == EN_FAIL) {
                    restart(i, now, &fb[i]);
                }
            }
        }
    }
    // 仍处于总线抑制状态：本圈不向电机发任何帧
    if (arm_output_diag.bus_inhibit) return;
    /* startup_failed：例如夹爪零点确认失败，属于不可自动恢复的故障。
     * 处理方式：强制所有增益/力矩为 0，未就绪的关节标记 EN_FAIL。 */
    if (arm_output_diag.startup_failed) {
        for (unsigned i=0; i<7; ++i) {
            // 即使上层还在下发指令，也把输出清零
            cmd.kp[i]=cmd.kd[i]=cmd.torque[i]=0.0f;
            if (arm_output_diag.state[i] != EN_OK) arm_output_diag.state[i] = EN_FAIL;
        }
    }
    /* 第一步：发送前完成所有状态转换，覆盖本拍新增的全部故障。 */
    uint8_t has_failed_joint = 0;
    for (unsigned i = 0; i < 7; ++i) {
        /* 反馈超时检查 */
        Arm_CheckFeedbackTimeout(i, &fb[i], now);
        /* 先确认握手，允许最后一次使能尝试收到的有效反馈生效。 */
        if (Arm_CheckEnableHandshake(i, &fb[i], now)) {
            arm_output_diag.state[i] = EN_OK;
            if (!arm_output_diag.first_feedback_ms[i]) {
                arm_output_diag.first_feedback_ms[i] = now;
            }
        }
        /* 保留最后一次尝试后的 20 ms 等待窗口；发送失败而停在
         * EN_WAIT_BOOT 的轴也必须受同一尝试上限约束。 */
        if ((arm_output_diag.state[i] == EN_WAIT_BOOT ||
             arm_output_diag.state[i] == EN_SEND) &&
            arm_output_diag.attempts[i] >= 50U && now - last_tx[i] >= 20U) {
            arm_output_diag.state[i] = EN_FAIL;
        }
        if (arm_output_diag.state[i] == EN_FAIL) {
            has_failed_joint = 1;
        }
    }

    /* 第二步：如果检测到故障，立即清零所有轴的输出 */
    if (has_failed_joint) {
        for (unsigned i = 0; i < 7; ++i) {
            cmd.kp[i] = cmd.kd[i] = cmd.torque[i] = 0.0f;
        }
    }

    /* 第三步：轮转发送，避免 FIFO 紧张时总是前几个轴优先。
     * 此阶段不再产生 EN_FAIL，故障清零不会遗漏已经发出的控制帧。 */
    for (unsigned n=0; n<7; ++n) {
        unsigned i = (rotating_first + n) % 7U; // 本圈第 n 个处理的关节
        uint8_t state = arm_output_diag.state[i];

        /* EN_FAIL 的关节本圈直接跳过：不自恢复，必须由上层重新授权 */
        if (state == EN_FAIL) { arm_output_diag.offline_skip[i]++; continue; }

        /* 未就绪关节处理 */
        if (state != EN_OK) {
            Arm_SendEnableFrame(i, &fb[i], now);
            continue;
        }
        /* 发送控制帧 */
        Arm_SendControlFrame(i, &fb[i], &cmd, now);
    }
    rotating_first = (uint8_t)((rotating_first + 1U) % 7U); // 轮转起点 +1，下一圈从下一个关节开始
}
/* Moter_Task: 机械臂输出任务入口（FreeRTOS 任务，osPriorityRealtime）。
 * 参数 argument：未使用。
 * 返回：无（任务函数不返回）。
 * 周期：ARM_OUTPUT_PERIOD_MS = 4 ms，用 vTaskDelayUntil 维持绝对节拍。
 * 注意：now 由 tick 换算成毫秒传给状态机；若本圈耗时超过 4 ms，
 *       vTaskDelayUntil 会立即返回以追赶节拍，不会累积漂移。 */
void Moter_Task(void const *argument)
{
    (void)argument;
    TickType_t next = xTaskGetTickCount(); // 绝对节拍基准：避免相对延时累积漂移
    for (;;) {
        Arm_Output_Step(xTaskGetTickCount() * portTICK_PERIOD_MS); // 状态机内部完成 7 关节的发送/重试
        vTaskDelayUntil(&next, pdMS_TO_TICKS(ARM_OUTPUT_PERIOD_MS)); // 固定 4 ms 周期
    }
}
