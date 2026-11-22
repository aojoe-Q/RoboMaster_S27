/**
  ******************************************************************************
  * @file    Arm_USB_task.c
  * @brief   USB CDC link between the arm and ROS2 / MoveIt2.
  ******************************************************************************
  */
/*
 * 模块职责 / 上下文 / 失效处理
 * ----------------------------------------------------------------
 * 本文件实现机械臂固件与上位机（ROS2 + MoveIt2）之间的 USB CDC 虚拟串口链路：
 *   收：POSITION_CMD(0x02) / ENABLE(0x03) / STOP(0x04) / HEARTBEAT(0x05)
 *   发：STATE(0x01, 100 Hz) / ACK(0x06) / FAULT(0x07) / DIAGNOSTIC(0x08, 500 ms)
 *
 * 任务与周期：Arm_USB_Task 由 Core/Src/freertos.c 创建（优先级在创建处指定），
 * 用 vTaskDelayUntil 维持 10 ms（100 Hz）的绝对周期，不累积漂移。本模块把
 * xTaskGetTickCount() 直接当毫秒用，因此要求 configTICK_RATE_HZ == 1000，
 * 由下面的编译期断言保证。
 *
 * 上下文：CDC_Init_HS / CDC_DeInit_HS / CDC_Receive_HS 三个 USB 中断回调分别
 * 调用 OnSessionStart / OnSessionEnd / RxPush，它们只写 rx_ring 与会话代号；
 * 任务侧用 taskENTER_CRITICAL() 临界区读取，单核上不需要互斥量。Arm_L_Task
 * 只在 Get_PC_Target / PC_Disarm 两处进入，且各自加临界区。
 *
 * 失效处理统一为：断开 PC 控制（清使能 + 丢目标）+ 锁存故障码 + 发 FAULT 帧；
 * 关节位置由 Arm_L_Task 保持，不掉力：
 *   300 ms 无有效帧               -> ARM_FAULT_LINK_TIMEOUT
 *   关节反馈 100 ms 未更新         -> ARM_FAULT_FEEDBACK_LOST
 *   USB 会话结束 / 挂起            -> ARM_FAULT_USB_DISCONNECT
 *   RX 环拒收字节                  -> ARM_FAULT_RX_OVERFLOW
 *   CAN 总线抑制 / 电机使能失败     -> ARM_FAULT_CAN_BUS / ARM_FAULT_DM_ENABLE
 * 故障位锁存到下一次成功的 ENABLE。ENABLE 还要求遥控开关在 PC 模式、
 * Arm_L_Init 已完成，且 7 个关节的反馈都在 100 ms 内。
 *
 * 会话代号（session_gen）：每次 CDC 初始化/去初始化自增一次。帧在解析前与
 * 提交时两次比对代号，代号变了就作废，因此旧连接残留的指令不会在新连接上
 * 生效，这是唯一区分「指令属于哪次连接」的依据。
 */

#include "Arm_USB_task.h"
#include "ring_buffer.h"
#include "Minipc.h"
#include "Arm_L_task.h"
#include "can_bsp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "arm_motion.h"
#include "arm_output.h"
#include "can_health.h"
#include "motor_encoding.h"

#ifdef configTICK_RATE_HZ
typedef char ArmUsbRequires1kHzTick[(configTICK_RATE_HZ == 1000U) ? 1 : -1];  // 编译期断言：把 tick 当毫秒用，tick 频率不是 1 kHz 时数组长度为 -1，编译直接失败
#endif
USB_Stats_t usb_stats;  // 全局统计：任务写，调试器与诊断帧读

/* ------------------------------------------------------- session state
 *
 * The USB interrupt bumps session_gen at every CDC Init / DeInit and sets
 * session_open accordingly. Bytes are tagged implicitly: the ring is flushed
 * by the task whenever it sees a new generation, and the RX push ignores
 * bytes while no session is open. A frame is only acted upon when the
 * generation is unchanged from before parsing to after the state update,
 * checked inside the same critical section that writes the state.
 */
// ---- 会话状态（session state）----
// USB 中断在每次 CDC Init / DeInit 时自增 session_gen 并设置 session_open。
// 字节的归属是隐式的：任务看到代号变化就冲刷环，无会话时 RxPush 直接拒收。
// 一帧只有在「解析前代号 == 提交时代号」时才生效，该比较在写状态的同一个临界
// 区里完成，因此 USB 复位插在中间时旧帧会被判 NO_SESSION 丢弃。

static RingBuffer_t rx_ring;  // RX 字节环：USB 中断是唯一生产者，本任务是唯一消费者

static volatile uint32_t session_gen;       /* written by the USB IRQ   */
static volatile uint8_t  session_open;      /* written by the USB IRQ   */
static volatile uint16_t session_rx_start;  /* ring head at session start */
static uint32_t          task_gen;          /* generation the task saw  */

/* PC 控制状态：只在 taskENTER_CRITICAL() 临界区里读写。
   字段按「会话 -> 时基 -> 指令」排列，对照 handle_position_cmd 的处理顺序看。 */
typedef struct
{
    uint8_t  enabled;  // 1 = 本会话已收到 ENABLE，PC 控制的总闸
    uint8_t  command_valid;  // 1 = 持有未过期、可用的 POSITION_CMD 目标
    uint16_t fault_code;  // 锁存故障位（见 arm_usb_protocol.h），成功 ENABLE 时清零
    uint32_t gen;                   /* session the enable belongs to */
    uint32_t last_rx_ms;            /* any valid frame               */
    uint32_t cmd_expire_ms;         /* POSITION_CMD freshness        */
    uint8_t  has_last_cmd;  // 1 = 本会话内至少收到过一条 POSITION_CMD
    uint32_t last_cmd_host_ms;      /* POSITION_CMD.command_timestamp_ms */
    uint32_t host_offset_ms;        /* mcu_ms - host_ms, minimum seen */
    uint32_t offset_creep_ms;       /* last drift adjustment          */
    float    target_rad[ARM_PROTO_JOINT_NUM];  // 目标位置（rad），由 position_urad 除以 1e6 得到
    float velocity_limit[ARM_PROTO_JOINT_NUM];  // 每关节速度上限（rad/s），0 已换成固件默认值 ARM_PC_VMAX_RAD_S
} PcControl_t;

static PcControl_t pc;              /* guarded by a critical section */

/* 待发 ACK：一条指令一个结果。POSITION_CMD 只在出错时才入队（100 Hz 全回应答会把反向流量放大一倍）。 */
typedef struct
{
    uint8_t type;  // 被应答消息的类型（ArmMsgType_t）
    uint8_t seq;  // 被应答帧的 sequence，上位机据此把 ACK 配回请求
    uint8_t result;  // ArmAckResult_t 取值
} PendingAck_t;

/* ACK 环形队列，深度 ARM_USB_ACK_QUEUE_LEN（8）；写满时丢最旧的一条，最新结果优先。 */
static PendingAck_t ack_queue[ARM_USB_ACK_QUEUE_LEN];
static uint8_t      ack_count;  // 队列中待发条数，只在任务上下文里改
static uint8_t      fault_pending;  // 1 = 故障位有变化，本周期末补发一帧 FAULT

static uint8_t  tx_seq;  // 发送方序号，8 位回绕；上位机用它配对 ACK 与 STATE
static uint32_t diag_last_ms;  // 上一帧 DIAGNOSTIC 的时刻，500 ms 节流用
static uint8_t diag_page;  // 诊断页号 0/1 交替：0 = CAN 总线健康，1 = 关节发送诊断
static uint8_t  tx_buf[(ARM_USB_ACK_QUEUE_LEN + 3U) * ARM_PROTO_MAX_FRAME];  // 一周期内所有待发帧拼在这里一次提交：ACK×8 + FAULT + STATE + DIAGNOSTIC = 11 帧上限
static uint8_t  rx_frame[ARM_PROTO_MAX_FRAME];  // 解析用的整帧暂存，不跨周期保存

static uint32_t rx_seen_dropped;  // ring.dropped 的已统计快照，用来算增量
static uint8_t  rx_stalled;  // 1 = 正卡在半个帧上，等满 ARM_USB_RX_STALL_MS 就丢 1 B 重新找 SOF
static uint32_t rx_stall_since_ms;  // 开始等待半个帧的时刻

/* Joint limits from Arm_L_task.h, same order as the wire arrays. */
/* 单位 rad。顺序必须与线上数组一致：J1 J2 J3 J4 J5 J6 夹爪。
   越限判定用这套限位，任一关节越限整条指令拒绝，不做单关节限幅。 */
static const float joint_max[ARM_PROTO_JOINT_NUM] = {
    J1_Max_position, J2_Max_position, J3_Max_position, J4_Max_position,
    J5_Max_position, J6_Max_position, Grip_Max_position
};
static const float joint_min[ARM_PROTO_JOINT_NUM] = {
    J1_Min_position, J2_Min_position, J3_Min_position, J4_Min_position,
    J5_Min_position, J6_Min_position, Grip_Min_position
};

/* ------------------------------------------------------------- helpers */

/* 毫秒比较：用有符号差值判断 deadline 是否已到，天然处理 32 位毫秒回绕（约 49.7 天）。
   返回 1 = 已到/已过，0 = 还没到。任务上下文。 */
static uint8_t time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (uint8_t)((int32_t)(now_ms - deadline_ms) >= 0);  // 有符号差值比较，避免 now 回绕后误判
}

/* 推导对外的系统状态：使能且有新鲜指令 = RUNNING；使能但无指令 = ENABLED；
   未使能时只有严重故障位才报 FAULT，否则 IDLE。任务上下文。 */
static uint8_t system_state(void)
{
    if (pc.enabled)
    {
        return pc.command_valid ? ARM_SYS_RUNNING : ARM_SYS_ENABLED;  // 使能 + 新鲜指令 = RUNNING
    }
    return (pc.fault_code & (ARM_FAULT_LINK_TIMEOUT | ARM_FAULT_USB_DISCONNECT |  // 只有这几类严重故障才让对外状态变成 FAULT
                             ARM_FAULT_FEEDBACK_LOST | ARM_FAULT_CAN_BUS | ARM_FAULT_DM_ENABLE))
           ? ARM_SYS_FAULT : ARM_SYS_IDLE;
}

/* 入队一条 ACK。队列满时整体前移丢弃最旧的一条，保证最新结果一定发得出去。
   参数：type/seq 原样回填被应答的帧；result = ArmAckResult_t。
   上下文：任务上下文（send_tx 消费），不单独进临界区。 */
static void queue_ack(uint8_t type, uint8_t seq, uint8_t result)
{
    if (ack_count >= ARM_USB_ACK_QUEUE_LEN)
    {
        for (uint8_t i = 1; i < ARM_USB_ACK_QUEUE_LEN; i++)
        {
            ack_queue[i - 1U] = ack_queue[i];
        }
        ack_count--;
        usb_stats.tx_ack_dropped++;  // 队列已满：丢最旧的，计入 tx_ack_dropped
    }
    ack_queue[ack_count].type   = type;
    ack_queue[ack_count].seq    = seq;
    ack_queue[ack_count].result = result;
    ack_count++;
}

/* 锁存故障位。只有位图真的变化才置 fault_pending，避免重复发 FAULT 帧。
   上下文：任务上下文，调用者持有临界区。 */
static void set_fault(uint16_t bits)
{
    if ((pc.fault_code & bits) != bits)
    {
        pc.fault_code |= bits;  // 锁存：故障位只增不减，直到下一次成功 ENABLE
        fault_pending = 1U;  // 置位后本周期末一定补发一帧 FAULT
    }
}

/* Caller holds the critical section. */
/* 断开 PC 控制：清使能 + 丢目标，不碰故障位、不碰关节位置。
   调用者必须已持有临界区（函数名里的 locked 即此意）。 */
static void disable_locked(void)
{
    pc.enabled       = 0U;
    pc.command_valid = 0U;
}

/* Bit i set when joint i has fed back within ARM_USB_FEEDBACK_TIMEOUT_MS.
   Ages are clamped to 0xFFFE; ARM_FEEDBACK_AGE_NEVER means no feedback yet.
   Caller holds the critical section. */
/* 计算每关节反馈有效掩码。
   参数：now_ms 当前毫秒；age_out 可为 NULL，非 NULL 时写回每关节年龄。
   返回：bit i = 关节 i 的反馈在 ARM_USB_FEEDBACK_TIMEOUT_MS（100 ms）内且已使能。
   年龄封顶 0xFFFE，0xFFFF（ARM_FEEDBACK_AGE_NEVER）表示从未收到过反馈。
   调用者必须已持有临界区。 */
static uint8_t feedback_mask_locked(uint32_t now_ms, uint16_t age_out[ARM_PROTO_JOINT_NUM])
{
    uint8_t mask = 0U;

    for (uint8_t i = 0; i < ARM_PROTO_JOINT_NUM; i++)
    {
        uint16_t age = ARM_FEEDBACK_AGE_NEVER;

        if (joint_L_motor[i].update_count != 0U)  // update_count == 0：该关节从未收到过 CAN 反馈
        {
            int32_t d = (int32_t)(now_ms - joint_L_motor[i].update_tick);
            // 反馈时刻晚于采样时刻（同一周期内）时按 0 处理
            uint32_t a = d < 0 ? 0U : (uint32_t)d;      /* feedback after now_ms was sampled */
            age = (uint16_t)(a > 0xFFFEU ? 0xFFFEU : a);
            if (a <= ARM_USB_FEEDBACK_TIMEOUT_MS && joint_L_motor[i].status == 1U)  // 100 ms 内且电机状态正常才算该关节有效
            {
                mask |= (uint8_t)(1U << i);  // bit i = 关节 i 有效
            }
        }
        if (age_out != 0)
        {
            age_out[i] = age;
        }
    }
    return mask;
}

#define ALL_JOINTS_MASK  ((uint8_t)((1U << ARM_PROTO_JOINT_NUM) - 1U))  // 7 个关节全部就绪的掩码 0x7F

/* -------------------------------------------------------- frame handlers
 * Each handler decodes and validates without the lock, then commits under
 * the lock only if the USB session is still the one the bytes came from.
 */

/* 处理 POSITION_CMD (0x02)，上位机 100 Hz 下发的 7 关节目标点。
   参数：hdr 已解出的帧头；payload 载荷起点；now_ms；gen 解析这条帧时的会话代号。
   返回：ArmAckResult_t（成功 = ARM_ACK_OK）。
   流程：长度校验 -> 解码 -> 全量越限校验 -> 进临界区按会话/使能/时戳判定 -> 提交目标。
   上下文：任务上下文，内部自行进临界区；解码与校验放在锁外，缩短临界区。 */
static uint8_t handle_position_cmd(const ArmFrameHeader_t *hdr, const uint8_t *payload,
                                   uint32_t now_ms, uint32_t gen)
{
    ArmPositionCommand_t cmd;
    float target[ARM_PROTO_JOINT_NUM];
    uint8_t result = ARM_ACK_OK;

    if (hdr->payload_len != ARM_POS_CMD_PAYLOAD_SIZE)  // 长度必须恰好 64 B，否则整帧拒绝
    {
        return ARM_ACK_BAD_LENGTH;
    }

    arm_proto_decode_position_cmd(payload, &cmd);  // 解码到栈上：本段在锁外，不碰共享状态

    /* Reject the whole command if one joint is outside its limits:
       clamping one joint of a trajectory point distorts the path. */
    for (uint8_t i = 0; i < ARM_PROTO_JOINT_NUM; i++)
    {
        if (cmd.velocity_limit_urad_s[i] < 0) return ARM_ACK_BAD_VALUE;  // 速度为负视为非法值：整条指令拒绝
        target[i] = arm_proto_urad_to_rad(cmd.position_urad[i]);  // µrad -> rad（除以 1e6）
        if (target[i] > joint_max[i] || target[i] < joint_min[i])
        {
            return ARM_ACK_OUT_OF_RANGE;  // 越限即整条拒绝：单关节限幅会把轨迹点拉变形
        }
    }

    uint32_t valid_ms = cmd.valid_time_ms;
    if (valid_ms == 0U || valid_ms > ARM_USB_CMD_TIMEOUT_MS)  // 0 或超过 300 ms 一律按 300 ms 上限处理
    {
        valid_ms = ARM_USB_CMD_TIMEOUT_MS;
    }
    if (valid_ms < ARM_USB_CMD_MIN_VALID_MS)
    {
        valid_ms = ARM_USB_CMD_MIN_VALID_MS;  // 下限 10 ms，防止上位机传极小值导致每帧都判过期
    }

    taskENTER_CRITICAL();  // 从这里到 EXIT：与 USB 中断的会话切换、与 Arm_L_Task 的取目标互斥
    if (gen != session_gen)  // 解析期间可能发生 USB 复位，代号变了这条指令就作废
    {
        result = ARM_ACK_NO_SESSION;
    }
    else if (!pc.enabled || pc.gen != gen)  // 使能属于别的会话：旧连接的 ENABLE 不能给新连接授权
    {
        result = ARM_ACK_NOT_ENABLED;
    }
    else if (pc.has_last_cmd &&
             (int32_t)(cmd.command_timestamp_ms - pc.last_cmd_host_ms) <= 0)  // 主机时戳必须严格递增：相等或更旧都是重发/乱序
    {
        /* Host timestamps are monotonic per session: older or equal means a
           resend or an out-of-date frame, never a new trajectory point. */
        result = ARM_ACK_STALE;
    }
    else
    {
        /* Queueing delay: (mcu - host) grows when a frame waited in a buffer.
           The minimum seen is the delay of the fastest frame; measure the
           extra delay against it, which needs no clock synchronisation. */
        uint32_t offset = now_ms - cmd.command_timestamp_ms;  // 排队时延 = 本机收帧时刻 - 主机发送时刻（含两侧时基偏差）
        if (!pc.has_last_cmd || (int32_t)(offset - pc.host_offset_ms) < 0)
        {
            pc.host_offset_ms = offset;  // 取见过的最小偏移 = 最快一帧的时延，作为时基基线（无需对钟）
            pc.offset_creep_ms = now_ms;
        }
        else if (offset != pc.host_offset_ms &&
                 time_reached(now_ms, pc.offset_creep_ms + ARM_USB_OFFSET_CREEP_MS))
        {
            /* Crystal drift between PC and MCU (~100 ppm = 0.1 ms/s) would
               slowly turn every command stale. Let the baseline follow by
               1 ms per second; a real backlog grows far faster than that. */
            pc.host_offset_ms++;  // 晶振漂移约 100 ppm = 0.1 ms/s：基线每秒放 1 ms 跟随，真实积压涨得更快
            pc.offset_creep_ms = now_ms;
        }
        uint32_t queued_ms = offset - pc.host_offset_ms;  // 扣掉基线后剩下的才是真正的排队时延

        pc.last_cmd_host_ms = cmd.command_timestamp_ms;  // 记下本帧时戳，作为下一帧的下限
        pc.has_last_cmd     = 1U;

        if (queued_ms >= valid_ms)  // 排队时间已经吃掉整个有效期：判过期，不提交
        {
            result = ARM_ACK_STALE;
        }
        else if (!cmd.enable)
        {
            pc.command_valid = 0U;  /* host asks to hold: drop the target */
        }
        else
        {
            for (uint8_t i = 0; i < ARM_PROTO_JOINT_NUM; i++)
            {
                pc.target_rad[i] = target[i];
                pc.velocity_limit[i] = cmd.velocity_limit_urad_s[i] == 0 ? ARM_PC_VMAX_RAD_S :
                    fminf(ARM_PC_VMAX_RAD_S, arm_proto_urad_to_rad(cmd.velocity_limit_urad_s[i]));  // 0 表示用固件默认限速；正数取与固件上限的较小值
            }
            /* Valid time counts from the host send time, as far as the
               offset estimate can tell. */
            pc.cmd_expire_ms = now_ms + (valid_ms - queued_ms);  // 过期时刻从主机发送时刻起算：现在 + 剩余有效期
            pc.command_valid = 1U;  // 目标已刷新，command_valid 置位后才允许运动
        }
    }
    taskEXIT_CRITICAL();

    return result;
}

/* 处理 ENABLE (0x03)。载荷固定 4 B，payload[0] 只接受 0/1。
   使能前必须同时满足：会话一致、遥控开关在 PC 模式、Arm_L_Init 完成、
   电机全部使能、7 个关节反馈都在 100 ms 内；任一不满足就拒绝并回原因。
   返回 ArmAckResult_t；上下文：任务上下文，内部进临界区。 */
static uint8_t handle_enable(const ArmFrameHeader_t *hdr, const uint8_t *payload,
                             uint32_t now_ms, uint32_t gen)
{
    uint8_t result = ARM_ACK_OK;
    uint8_t enabled_now = 0U;

    if (hdr->payload_len != ARM_ENABLE_PAYLOAD_SIZE)  // 固定 4 B：enable + 3 B 保留
    {
        return ARM_ACK_BAD_LENGTH;
    }
    if (payload[0] > 1U)  // 只接受 0/1，其它值回 BAD_VALUE
    {
        return ARM_ACK_BAD_VALUE;
    }

    taskENTER_CRITICAL();
    if (gen != session_gen || !session_open)
    {
        result = ARM_ACK_NO_SESSION;
    }
    /* enable = 0：立即撤销，无任何前置条件（断使能永远不能失败）。 */
    else if (payload[0] == 0U)
    {
        disable_locked();
    }
    /* 遥控开关不在 PC 模式时拒绝 ENABLE：Arm_L_Task 在离开该模式时已断过使能，
       这里堵住另一条会留下陈旧使能的路径。 */
    else if (ArmData.ArmMode != ARM_MODE_PC_CONTROL)
    {
        /* Arm_L_Task disarms on leaving PC mode; refusing ENABLE outside it
           closes the other path to a stale enable. */
        result = ARM_ACK_WRONG_MODE;
    }
    /* 三个前置条件：Arm_L_Init 完成、电机全部使能、7 关节反馈都在 100 ms 内。 */
    else if (!Arm_L_Ready || !Arm_Output_AllEnabled() ||
             feedback_mask_locked(now_ms, 0) != ALL_JOINTS_MASK)
    {
        result = ARM_ACK_NOT_READY;
    }
    else
    {
        pc.enabled       = 1U;  // 通过全部检查：打开总闸
        pc.command_valid = 0U;      /* a fresh POSITION_CMD is required */
        pc.gen           = gen;  // 绑定当前会话代号：旧连接的指令在新会话里一律 NOT_ENABLED
        pc.has_last_cmd  = 0U;      /* new host timeline */
        pc.fault_code    = 0U;      /* ENABLE acknowledges the old faults */
        enabled_now      = 1U;
    }
    taskEXIT_CRITICAL();

    /* 成功使能也要发一帧 FAULT：让上位机立刻看到故障位已被 ENABLE 清零。 */
    if (enabled_now)
    {
        fault_pending = 1U;         /* report the cleared state */
    }
    return result;
}

/* Host self-test only: lets the test end the USB session at the point an
   interrupt could, between validating a frame and committing it. */
/* 自测注入点：仅 ARM_USB_SELFTEST 固件定义，用来在「校验通过、提交之前」
   模拟一次 USB 复位，验证会话代号的竞态保护。正式固件里展开为空操作。 */
#ifdef ARM_USB_SELFTEST
void arm_usb_selftest_before_commit(uint8_t type);
#define BEFORE_COMMIT(type)  arm_usb_selftest_before_commit(type)
#else
#define BEFORE_COMMIT(type)  ((void)0)
#endif

/* 分发一条已通过 CRC 的帧。
   参数：frame 整帧起点；hdr 已解出的帧头；now_ms；gen 解析时的会话代号。
   返回：无（结果通过 ACK 队列上报）。
   上下文：任务上下文。 */
static void handle_frame(const uint8_t *frame, const ArmFrameHeader_t *hdr,
                         uint32_t now_ms, uint32_t gen)
{
    const uint8_t *payload = &frame[ARM_PROTO_HEADER_SIZE];  // 跳过 10 B 帧头，载荷起点
    uint8_t result;
    uint8_t ack_always = 1U;

    BEFORE_COMMIT(hdr->type);  // 自测钩子；正式固件里是空操作 (void)0

    if (hdr->version != ARM_PROTO_VERSION)  // 版本不符：不解析载荷，直接回 BAD_VERSION
    {
        result = ARM_ACK_BAD_VERSION;
    }
    else
    {
        switch (hdr->type)
        {
            case ARM_MSG_POSITION_CMD:
                result = handle_position_cmd(hdr, payload, now_ms, gen);
                ack_always = 0U;    /* 100 Hz stream: only errors are acked */
                /* 100 Hz 的指令流成功时不回 ACK，否则反向流量翻倍；出错必须回报。 */
                break;

            case ARM_MSG_ENABLE:
                result = handle_enable(hdr, payload, now_ms, gen);
                break;

            /* STOP：不需要额外的长度/数值校验，清使能 + 丢目标即可。 */
            case ARM_MSG_STOP:
                taskENTER_CRITICAL();
                disable_locked();
                taskEXIT_CRITICAL();
                result = ARM_ACK_OK;
                break;

            case ARM_MSG_HEARTBEAT:
                result = ARM_ACK_OK;
                ack_always = 0U;  // HEARTBEAT 只刷新链路活跃时间，成功不回应答
                break;

            default:
                result = ARM_ACK_UNKNOWN_TYPE;  // 未知类型：回 UNKNOWN_TYPE，链路保持不断
                break;
        }
    }

    /* Only a frame of the live session keeps the link alive. */
    if (result != ARM_ACK_NO_SESSION && gen == session_gen)  // 只有当前会话的有效帧才刷新链路活跃时间
    {
        pc.last_rx_ms = now_ms;
    }

    if (result != ARM_ACK_OK)
    {
        usb_stats.rx_rejected++;  // 统计被拒绝的帧数
        if (result == ARM_ACK_STALE)
        {
            usb_stats.rx_stale++;  // STALE 单独计数：上位机据此区分「指令丢了」和「指令晚了」
        }
    }
    /* 会话已变：没有接收方了，连错误也不回。 */
    if (result == ARM_ACK_NO_SESSION)
    {
        return;                     /* nobody left to answer */
    }
    /* 成功帧只在 ack_always 的消息上回 ACK；失败一律回报。 */
    if (ack_always || result != ARM_ACK_OK)
    {
        queue_ack(hdr->type, hdr->sequence, result);
    }
}

/* ------------------------------------------------------------- RX path */

/* RX 解析状态机，每周期调用，直到无数据或帧不完整：
     1) 逐字节找 SOF 0xA5，不是就丢 1 B 并计入重同步；
     2) 头部到齐后取 payload_len，超过 160 B 判为假 SOF；
     3) 帧不完整就等下一周期，等满 20 ms 仍不完整则丢 1 B 重来；
     4) 整帧到齐后校验 CRC，通过才推进读指针并交给 handle_frame。
   任何长度的垃圾数据都只按 1 B 步进恢复，不会被一个假 SOF 长期卡死。
   上下文：任务上下文。 */
static void parse_rx(uint32_t now_ms, uint32_t gen)
{
    ArmFrameHeader_t hdr = {0};

    for (;;)
    {
        if (gen != session_gen)  // 会话代号变化：立即返回，由 check_session 下周期统一清理
        {
            return;                 /* session changed: next step flushes */
        }

        uint16_t avail = ring_buffer_available(&rx_ring);  // 可读字节数的一次快照

        if (avail == 0U)
        {
            rx_stalled = 0U;  // 没有数据：清掉卡半帧状态
            return;
        }

        /* Hunt for SOF one byte at a time. */
        ring_buffer_peek(&rx_ring, rx_frame, 1U);  // 一次只 peek 1 B 找 SOF，避免读过界
        if (rx_frame[0] != ARM_PROTO_SOF)
        {
            ring_buffer_skip(&rx_ring, 1U);  // 不是 SOF：丢 1 B 继续找，并计入重同步字节
            usb_stats.rx_resync_bytes++;
            rx_stalled = 0U;
            continue;
        }

        uint16_t frame_len = ARM_PROTO_HEADER_SIZE;
        if (avail >= ARM_PROTO_HEADER_SIZE)
        {
            ring_buffer_peek(&rx_ring, rx_frame, ARM_PROTO_HEADER_SIZE);
            arm_proto_parse_header(rx_frame, &hdr);  // 只解头部，SOF / CRC 在别处校验

            if (hdr.payload_len > ARM_PROTO_MAX_PAYLOAD)
            {
                /* payload_len 超过 160 B：这个 0xA5 是载荷里偶然出现的假 SOF。 */
                ring_buffer_skip(&rx_ring, 1U);     /* false SOF */
                usb_stats.rx_resync_bytes++;
                rx_stalled = 0U;
                continue;
            }
            frame_len = (uint16_t)(ARM_PROTO_OVERHEAD + hdr.payload_len);  // 整帧长度 = 10 B 头 + N + 2 B CRC
        }

        /* 帧还没收全：等下一周期。若一直等不齐，下面的超时分支会丢弃重来。 */
        if (avail < frame_len)
        {
            /* Wait for the rest, but not forever: a false SOF with a big
               length would otherwise hide the real frames behind it. */
            if (!rx_stalled)
            {
                rx_stalled = 1U;  // 第一次发现不完整：记下时刻
                rx_stall_since_ms = now_ms;
            }
            /* 等满 ARM_USB_RX_STALL_MS（20 ms）仍不完整：判假 SOF，丢 1 B 重来。 */
            else if (time_reached(now_ms, rx_stall_since_ms + ARM_USB_RX_STALL_MS))
            {
                ring_buffer_skip(&rx_ring, 1U);
                usb_stats.rx_resync_bytes++;
                rx_stalled = 0U;
                continue;
            }
            return;
        }

        rx_stalled = 0U;
        ring_buffer_peek(&rx_ring, rx_frame, frame_len);  // 帧已完整：整帧读出（不移动读指针）

        /* CRC 覆盖头部 + 载荷共 10+N 字节；不过就只丢 1 B，下次从下一个字节重新找 SOF。 */
        if (!arm_proto_frame_crc_ok(rx_frame, hdr.payload_len))
        {
            ring_buffer_skip(&rx_ring, 1U);
            usb_stats.rx_crc_errors++;
            continue;
        }

        ring_buffer_skip(&rx_ring, frame_len);  // CRC 通过才推进读指针；失败时保留数据以逐字节重同步
        usb_stats.rx_frames++;  // 统计：CRC 正确且完整的帧
        handle_frame(rx_frame, &hdr, now_ms, gen);  // 提交这条帧（内部按需进临界区）
    }
}

/* -------------------------------------------------------- safety checks */

/* Runs first: a new session generation discards everything older. */
/* 检查 USB 会话状态，返回当前会话代号。
   顺序上最先执行：新的会话代号会作废所有更早的数据与应答。
   返回：session_gen 的快照，供本周期后续解析/提交比对。
   上下文：任务上下文；读 gen/open/head 三个字段时进临界区。 */
static uint32_t check_session(void)
{
    uint32_t gen;
    uint8_t  open;
    uint16_t start;
    uint8_t  dropped = 0U;

    /* gen, open and session_rx_start are read together: the USB IRQ writes
       them together and is masked here. */
    taskENTER_CRITICAL();  // 三个字段一次临界区读出，避免读到中断写了一半的组合
    gen   = session_gen;
    open  = session_open;
    start = session_rx_start;
    /* 已使能但会话变了 / 已关闭 / 已挂起：立即断开，等上位机重新 ENABLE。
       挂起时 CDC 类还在，但 CONFIGURED 已清，MiniPC_Is_Connected() 返回 0。 */
    if (pc.enabled && (pc.gen != gen || !open || !MiniPC_Is_Connected()))
    {
        /* Ended, replaced or suspended (suspend keeps the CDC class but
           leaves CONFIGURED). Re-enabling needs a new ENABLE. */
        disable_locked();
        dropped = 1U;
    }
    taskEXIT_CRITICAL();

    if (dropped)
    {
        set_fault(ARM_FAULT_USB_DISCONNECT);  // 掉线锁存为 USB_DISCONNECT，由下一次 ENABLE 清除
    }

    if (gen != task_gen)  // 会话代号变化：清掉上一个会话的残留数据
    {
        /* Drop the bytes of earlier sessions, keep what the new session has
           already sent: session_rx_start is the ring head captured when the
           session opened, so everything before it is old. If the session
           also ended again, the push side refused new bytes, so the ring
           holds nothing worth keeping. */
        if (open)
        {
            /* If the parser already consumed past `start` (a new session
               opened mid-parse; those bytes were refused as NO_SESSION),
               this is a no-op and the host retries ENABLE. */
            ring_buffer_discard_until(&rx_ring, start);  // 只丢新会话开始之前写入的旧字节，保住新会话已到的数据
        }
        else
        {
            ring_buffer_flush(&rx_ring);  // 会话已结束：push 侧不会再进字节，整个环清空
        }
        rx_stalled = 0U;
        /* 这些 ACK 都是给旧会话准备的，一并作废。 */
        ack_count  = 0U;            /* answers to the old session */
        task_gen   = gen;
    }
    return gen;
}

/* Runs after parse_rx so frames of this period count as link activity. */
/* 超时与看门狗检查。放在 parse_rx 之后，本周期收到的帧才算链路活跃。
   三类判定：链路超时（300 ms）、关节反馈丢失（100 ms）、单条指令过期。
   上下文：任务上下文，内部进临界区；故障锁存在临界区外调 set_fault。 */
static void check_timeouts(uint32_t now_ms)
{
    uint8_t timed_out = 0U;
    uint8_t feedback_lost = 0U;

    taskENTER_CRITICAL();
    /* 300 ms 内没有任何有效帧：链路超时，断开 PC 控制。 */
    if (pc.enabled && time_reached(now_ms, pc.last_rx_ms + ARM_USB_LINK_TIMEOUT_MS))
    {
        disable_locked();
        timed_out = 1U;
    }
    /* Runtime CAN watchdog: a joint silent for ARM_USB_FEEDBACK_TIMEOUT_MS
       disables PC control, so the old target cannot resume by itself when
       the feedback comes back. Arm_L_Task holds the position meanwhile. */
    /* 运行期 CAN 看门狗：任一路反馈断流 100 ms 即断开，这样反馈恢复时不会让
       旧目标突然重新生效；断流期间由 Arm_L_Task 保持位置。 */
    if (pc.enabled && feedback_mask_locked(now_ms, 0) != ALL_JOINTS_MASK)
    {
        disable_locked();
        feedback_lost = 1U;
    }
    /* 指令自身有效期（10..300 ms）到了：只清 command_valid，使能保留。 */
    if (pc.command_valid && time_reached(now_ms, pc.cmd_expire_ms))
    {
        pc.command_valid = 0U;
    }
    taskEXIT_CRITICAL();

    if (timed_out)
    {
        set_fault(ARM_FAULT_LINK_TIMEOUT);  // 故障锁存，等下一次 ENABLE 清除
    }
    if (feedback_lost)
    {
        set_fault(ARM_FAULT_FEEDBACK_LOST);  // 反馈丢失同样锁存
    }

    uint32_t dropped = rx_ring.dropped;  // 增量统计：环拒收的字节数
    if (dropped != rx_seen_dropped)
    {
        usb_stats.rx_overflow_bytes += dropped - rx_seen_dropped;
        rx_seen_dropped = dropped;
        set_fault(ARM_FAULT_RX_OVERFLOW);  // 环溢出通常是任务被饿死（中断风暴/长临界区），按故障上报
    }
}

/* ------------------------------------------------------------- TX path */

/* 填充 STATE 载荷（138 B）。
   参数：s 输出结构体；now_ms 本帧时戳。
   上下文：任务上下文；位置/速度/力矩/目标在同一临界区里取，保证来自同一拍反馈。 */
static void fill_state(ArmStatePayload_t *s, uint32_t now_ms)
{
    Arm_Control_Snapshot output = Arm_Control_GetSnapshot();
    float pos[ARM_PROTO_JOINT_NUM], vel[ARM_PROTO_JOINT_NUM];
    float tor[ARM_PROTO_JOINT_NUM], set[ARM_PROTO_JOINT_NUM];
    uint8_t mode, ready, mask;

    /* One critical section: position / velocity / torque all come from the
       same CAN feedback sample of each motor (written by the FDCAN RX
       interrupt), the target from the controller output (Arm_L_Task). */
    taskENTER_CRITICAL();
    mode  = ArmData.ArmMode;
    ready = Arm_L_Ready;
    mask  = feedback_mask_locked(now_ms, s->feedback_age_ms);  // 同时把每关节反馈年龄写进载荷数组（0xFFFE 封顶）
    for (uint8_t i = 0; i < ARM_PROTO_JOINT_NUM; i++)
    {
        pos[i] = joint_L_motor[i].position;
        vel[i] = joint_L_motor[i].velocity;
        tor[i] = joint_L_motor[i].torque;
        set[i] = output.pos[i];
    }
    taskEXIT_CRITICAL();

    s->timestamp_ms   = now_ms;  // 固件本地毫秒时基：与主机时基无关，持续递增
    s->arm_mode       = mode;
    s->system_state   = system_state();  // 由使能/指令/故障推导出来的对外状态
    s->fault_code     = pc.fault_code;
    s->flags          = ready ? ARM_STATE_FLAG_ARM_READY : 0U;
    for (unsigned bus=0; bus<3; ++bus) {
        if (can_health[bus].tx_drop) s->flags |= ARM_STATE_FLAG_TX_DROP;  // 任一路 CAN 出现发送丢弃/接收溢出/总线错误都汇总进 flags
        if (can_health[bus].rx_overflow) s->flags |= ARM_STATE_FLAG_CAN_RX_OVERFLOW;
        if (can_health[bus].flags) s->flags |= ARM_STATE_FLAG_CAN_FAULT;
    }
    for (unsigned j=0; j<7; ++j)
        if (arm_output_diag.state[j] == EN_FAIL) s->flags |= ARM_STATE_FLAG_DM_FAILED;  // 任一电机使能失败即上报 DM_FAILED
    if (arm_output_diag.startup_failed) s->flags |= ARM_STATE_FLAG_DM_FAILED;
    s->feedback_valid = mask;  // 每关节反馈有效掩码：bit i = 关节 i 的反馈在 100 ms 内
    for (uint8_t i = 0; i < ARM_PROTO_JOINT_NUM; i++)
    {
        s->position_urad[i]        = arm_proto_rad_to_urad(pos[i]);  // rad -> µrad（×1e6，饱和）
        s->velocity_urad_s[i]      = arm_proto_rad_to_urad(vel[i]);  // rad/s -> µrad/s
        s->torque_mNm[i]           = arm_proto_nm_to_mnm(tor[i]);  // N·m -> mN·m（×1000）
        s->target_position_urad[i] = arm_proto_rad_to_urad(set[i]);  // 控制器输出的目标位置（ArmData.Jn.send_set）
    }
}

/* Optional diagnostic extension; STATE v2 remains exactly 138 bytes. */
/* 小端写 4 字节：诊断帧不走 put_u32，避免与协议层的字节序约定耦合。 */
static void diag_u32(uint8_t *p, uint32_t value)
{
    for (unsigned i=0; i<4; ++i) p[i]=(uint8_t)(value >> (i*8U));
}
/* 填充 DIAGNOSTIC 载荷（自定义格式，不属于 STATE v2）。
   参数：p 输出缓冲；now 当前毫秒。
   返回：载荷长度（页 0 = 90 B，页 1 = 160 B，都不超过 ARM_PROTO_MAX_PAYLOAD）。
   页 0：3 条 CAN 总线健康，每条 28 B；页 1：7 个关节发送诊断，每个 22 B。
   上下文：任务上下文，内部进临界区保证计数一致。 */
static uint16_t fill_diagnostic(uint8_t *p, uint32_t now)
{
    p[0]=1U; p[1]=diag_page;  // p[0] = 格式版本，p[1] = 页号，p[2..5] = 时戳
    diag_u32(p+2,now);
    unsigned offset=6U;  // 诊断头 6 B，之后是页内容
    taskENTER_CRITICAL();
    /* 页 0：3 条 CAN 总线的健康计数，每条 28 B（6×u32 + 4×u8）。 */
    if (diag_page == 0U) {
        for (unsigned b=0; b<3; ++b) {
            CanBusHealth h=can_health[b];
            diag_u32(p+offset,h.tx_drop); diag_u32(p+offset+4,h.rx_overflow);
            diag_u32(p+offset+8,h.rx_invalid); diag_u32(p+offset+12,h.busoff_count);
            diag_u32(p+offset+16,h.flags); diag_u32(p+offset+20,h.recovery_count);
            p[offset+24]=h.tec; p[offset+25]=h.rec;
            p[offset+26]=h.tx_free_min; p[offset+27]=h.blocked;
            offset+=28U;
        }
    /* 页 1：7 个关节的发送诊断，每个 22 B（5×u32 + 2×u8），合计 154 B。 */
    } else {
        for (unsigned j=0; j<7; ++j) {
            diag_u32(p+offset,motor_tx_invalid[j]); diag_u32(p+offset+4,can_joint_tx_drop[j]);
            diag_u32(p+offset+8,arm_output_diag.offline_skip[j]);
            diag_u32(p+offset+12,arm_output_diag.fifo_defer[j]);
            diag_u32(p+offset+16,arm_output_diag.first_feedback_ms[j]);
            p[offset+20]=arm_output_diag.state[j]; p[offset+21]=arm_output_diag.attempts[j];
            offset+=22U;
        }
    }
    taskEXIT_CRITICAL();
    return (uint16_t)offset;  // 返回实际载荷长度，交给 build_frame
}

/* 组包并提交一个周期的发送：ACK -> FAULT -> STATE -> DIAGNOSTIC，一次 CDC 传输。
   参数：now_ms 本帧时戳。返回：无。
   提交失败（CDC 忙 / 指针失效）时保留 ACK 与 FAULT，下个周期重试。
   上下文：任务上下文。 */
static void send_tx(uint32_t now_ms)
{
    uint8_t payload[ARM_PROTO_MAX_PAYLOAD];
    ArmStatePayload_t state;
    uint16_t len = 0;
    uint16_t n;

    /* 没有会话或已挂起：没有接收方，连 ACK 一起丢。 */
    if (!session_open || !MiniPC_Is_Connected())
    {
        ack_count = 0U;             /* nobody to answer */
        return;
    }
    /* 上一包还在发：本周期整体跳过，tx_buf 不能改写。 */
    if (MiniPC_Tx_Busy())
    {
        usb_stats.tx_busy++;        /* tx_buf is still being sent */
        return;
    }

    /* ACK and FAULT first, then the STATE frame, in one CDC transfer. */
    /* 顺序固定：ACK 先走（上位机在等结果）、FAULT 次之、STATE 每周期必有、诊断最后，
       即使容量不足，最关键的应答也排在前面。 */
    for (uint8_t i = 0; i < ack_count; i++)
    {
        n = arm_proto_encode_ack(payload, ack_queue[i].type, ack_queue[i].seq,
                                 ack_queue[i].result);
        len += arm_proto_build_frame(&tx_buf[len], (uint16_t)(sizeof(tx_buf) - len),
                                     ARM_MSG_ACK, tx_seq++, now_ms, payload, n);  // 依次追加 ACK 帧；capacity 传剩余空间，写不下时 build_frame 返回 0
    }

    if (fault_pending)
    {
        n = arm_proto_encode_fault(payload, pc.fault_code, system_state());  // 故障变化时补一帧 FAULT（与 STATE.fault_code 冗余，便于上位机立即反应）
        len += arm_proto_build_frame(&tx_buf[len], (uint16_t)(sizeof(tx_buf) - len),
                                     ARM_MSG_FAULT, tx_seq++, now_ms, payload, n);
    }

    fill_state(&state, now_ms);
    n = arm_proto_encode_state(payload, &state);
    len += arm_proto_build_frame(&tx_buf[len], (uint16_t)(sizeof(tx_buf) - len),
                                 ARM_MSG_STATE, tx_seq++, now_ms, payload, n);  // 每周期必有一帧 STATE（100 Hz）

    uint8_t diagnostic_due = (uint8_t)(now_ms-diag_last_ms >= 500U);  // 诊断帧 500 ms 一帧，两页交替
    if (diagnostic_due) {
        n=fill_diagnostic(payload,now_ms);
        len += arm_proto_build_frame(&tx_buf[len], (uint16_t)(sizeof(tx_buf)-len),
                                     ARM_MSG_DIAGNOSTIC, tx_seq++, now_ms, payload, n);  // 页 0/1 交替，单帧载荷不超过 160 B
    }

    /* MiniPC_Transmit_Info re-checks the class pointer with interrupts
       masked, so a reset between the checks above and here is harmless. */
    if (MiniPC_Transmit_Info(tx_buf, len) == MINIPC_TX_OK)
    {
        usb_stats.tx_frames += (uint32_t)ack_count + fault_pending + 1U + diagnostic_due;  // 实际发出的帧数：ACK 条数 + FAULT + STATE + DIAGNOSTIC
        if (diagnostic_due) { diag_last_ms=now_ms; diag_page^=1U; }
        ack_count     = 0U;  // 提交成功：清空待发 ACK 与故障标志
        fault_pending = 0U;
    }
    else
    {
        /* 提交失败：ACK / FAULT 保留到下一周期重发，STATE 下周期也会再发。 */
        usb_stats.tx_busy++;        /* keep ACK / FAULT for the next period */
    }
}

/* ---------------------------------------------------------- public API */

/* 会话开始回调。上下文：USB 中断（CDC_Init_HS）。
   只记录环头、自增代号、置会话标志；不阻塞、不进临界区（中断本身已屏蔽同级）。 */
void Arm_USB_OnSessionStart(void)
{
    /* 记录会话打开瞬间的 ring head：早于它的字节都属于旧会话。
       与 RxPush 在同一个中断里执行，不存在竞争。 */
    session_rx_start = rx_ring.head;    /* same IRQ as RxPush: no race */
    session_gen++;  // 代号自增：旧会话的帧与应答全部作废
    session_open = 1U;
}

/* 会话结束回调。上下文：USB 中断（CDC_DeInit_HS）。
   只自增代号并清标志，真正清环由任务下个周期看到代号变化后完成。 */
void Arm_USB_OnSessionEnd(void)
{
    session_gen++;  // 代号自增 + 清标志；任务下周期据此清环并清 ACK
    session_open = 0U;
}

/* 接收回调：把 USB 收到的字节塞进 RX 环。
   参数：data/len 一次 CDC 接收到的数据块（USB 中断里给出）。
   上下文：USB 中断，单生产者；与任务的单消费者配合，无需加锁。
   会话未打开或长度超出 16 位计数时，字节直接丢弃并计入 dropped。 */
void Arm_USB_RxPush(const uint8_t *data, uint32_t len)
{
    /* 会话未打开：中断里收到的字节直接丢，不入环。 */
    if (!session_open)
    {
        return;                     /* no session: nothing may be queued */
    }
    if (len > 0xFFFFU)
    {
        rx_ring.dropped += len;  // 长度超出 16 位计数能力：只计丢弃，避免截断后字节错位
        return;
    }
    (void)ring_buffer_write_bytes(&rx_ring, data, (uint16_t)len);
}

/* 初始化全部状态。上下文：任务上下文，Arm_USB_Task 启动时调用一次。
   任务启动前到达的字节会被清掉，避免把上一个会话的半帧带进来。 */
void Arm_USB_Init(void)
{
    diag_last_ms=(uint32_t)xTaskGetTickCount();  // 诊断节流起点
    diag_page=0U;
    taskENTER_CRITICAL();
    disable_locked();
    pc.fault_code   = 0U;
    pc.has_last_cmd = 0U;
    pc.gen          = 0U;
    task_gen        = session_gen;
    taskEXIT_CRITICAL();

    ack_count       = 0U;
    fault_pending   = 0U;
    tx_seq          = 0U;
    rx_stalled      = 0U;
    rx_seen_dropped = rx_ring.dropped;  // 启动前环里的字节属于更早的会话，直接计入丢弃
    ring_buffer_flush(&rx_ring);
}

/* 一个任务周期的主流程（10 ms 一次）。
   参数：now_ms 本周期时戳（tick 当毫秒）。
   顺序：总线/电机故障 -> 会话检查 -> 解析 RX -> 再查会话 -> 超时 -> 发送。
   上下文：任务上下文。 */
void Arm_USB_Step(uint32_t now_ms)
{
    taskENTER_CRITICAL();
    /* CAN 总线被抑制：锁存故障并立即断开 PC 控制。 */
    if (can_health[0].blocked || arm_output_diag.bus_inhibit) {
        set_fault(ARM_FAULT_CAN_BUS);
        disable_locked();
    }
    /* 电机使能失败：同样立即断开，不能凭旧目标继续跑。 */
    if (arm_output_diag.startup_failed) {
        set_fault(ARM_FAULT_DM_ENABLE);
        disable_locked();
    }
    taskEXIT_CRITICAL();

    uint32_t gen = check_session();  // 1) 会话检查（顺带丢弃旧会话字节、清旧 ACK）
    parse_rx(now_ms, gen);  // 2) 解帧、校验、提交指令
    /* Again: a USB reset during parse_rx must be applied (disable, fault,
       drop old bytes) before this period's STATE reports the state. */
    (void)check_session();  // 3) 再查一次：解析期间可能发生 USB 复位，必须在本周期 STATE 之前生效
    check_timeouts(now_ms);  // 4) 超时、看门狗、指令过期
    send_tx(now_ms);  // 5) 发 ACK / FAULT / STATE / DIAGNOSTIC
}

/* 断开 PC 控制。供 Arm_L_Task 在离开 ARM_MODE_PC_CONTROL 时调用：
   之后回到该模式必须收到新的 ENABLE。上下文：任意任务上下文，内部进临界区。 */
void Arm_USB_PC_Disarm(void)
{
    taskENTER_CRITICAL();
    disable_locked();
    taskEXIT_CRITICAL();
}

/* 查询 PC 控制是否处于使能状态。
   返回：1 = 已使能且会话一致且未挂起；0 = 其它。任务上下文，内部进临界区。 */
uint8_t Arm_USB_PC_Enabled(void)
{
    uint8_t on;

    taskENTER_CRITICAL();
    /* 使能 + 会话一致 + 未挂起，三者缺一不可。 */
    on = (uint8_t)(pc.enabled && session_open && pc.gen == session_gen &&
                   MiniPC_Is_Connected());
    taskEXIT_CRITICAL();
    return on;
}

/* 取 PC 关节目标（rad），供 Arm_L_Task 每周期调用。
   参数：target_rad[7] 输出；vmax[7] 可为 NULL，非 NULL 时输出速度上限。
   返回：1 = 目标有效并已拷出；0 = 条件不满足，输出数组不动。
   上下文：任务上下文，内部进临界区做一致性快照。 */
uint8_t Arm_USB_Get_PC_Command(float target_rad[ARM_PROTO_JOINT_NUM], float vmax[ARM_PROTO_JOINT_NUM])
{
    uint8_t ok;
    uint32_t now_ms = (uint32_t)xTaskGetTickCount();  // tick 当毫秒用（见文件头的编译期断言）

    if (target_rad == 0)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    /* Re-check everything here: the USB task runs only every 10 ms, and a
       USB reset may have ended the session since. Also check connection and
       feedback validity (runtime CAN watchdog). */
    /* 条件在临界区里全部重新确认一遍：USB 任务 10 ms 才跑一次，
       期间会话可能已结束、指令可能已过期、某关节反馈可能已断。 */
    ok = (uint8_t)(pc.enabled && pc.command_valid &&
                   session_open && pc.gen == session_gen &&
                   MiniPC_Is_Connected() &&
                   feedback_mask_locked(now_ms, 0) == ALL_JOINTS_MASK &&
                   !time_reached(now_ms, pc.cmd_expire_ms));
    if (ok)
    {
        for (uint8_t i = 0; i < ARM_PROTO_JOINT_NUM; i++)
        {
            target_rad[i] = pc.target_rad[i];  // 拷出快照，调用方拿到的是一致的一组目标
            if (vmax) vmax[i] = pc.velocity_limit[i];
        }
    }
    taskEXIT_CRITICAL();

    return ok;
}

/* FreeRTOS 任务入口（由 Core/Src/freertos.c 创建）。
   参数：argument 未使用。
   行为：初始化一次，之后每 10 ms 调一次 Arm_USB_Step；周期用绝对延时保证。 */
void Arm_USB_Task(void const *argument)
{
    (void)argument;

    Arm_USB_Init();
    TickType_t last_wake = xTaskGetTickCount();  // 绝对延时基准：下一个唤醒点 = 本拍 + 10 ms，不累积漂移

    for (;;)
    {
        Arm_USB_Step((uint32_t)xTaskGetTickCount());
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ARM_USB_TASK_PERIOD_MS));  // 周期 10 ms；tick 1 kHz 时即 10 个 tick
    }
}

/* 取 PC 关节目标的兼容入口：不关心速度上限时走这里，内部按 vmax = NULL 处理。 */
uint8_t Arm_USB_Get_PC_Target(float target_rad[ARM_PROTO_JOINT_NUM])
{
    return Arm_USB_Get_PC_Command(target_rad, 0);  // vmax 传 0 表示不关心速度上限
}
