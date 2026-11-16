/* CAN ISR capture, task decoding, checked TX and bounded bus-off recovery. */
/* 模块职责：三路 FDCAN 的 BSP：过滤器/中断配置、ISR 捕获到软件环形队列、任务侧分发、带健康统计的发送与 bus-off 恢复。 */
/* 总线映射：FDCAN1 -> 达妙关节 0x01..0x07（经典帧 1 Mbit/s）；FDCAN2 -> 双板/自定义控制器 0x212/0x213（FD+BRS）； */
/*           FDCAN3 -> 抬升 DJI 电机 0x201..0x204（经典帧）。 */
/* 运行方式：ISR 只做搬运与任务唤醒，解析/分发在 CAN_Task 上下文完成；发送接口可在任意任务上下文调用。 */
#include "fdcan.h"
#include "bsp_can.h"
#include "can_health.h"
#include "Motor.h"
#include "can_bsp.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

// 每路总线的软件环形队列深度：32 帧；满时丢新帧并计数，保证 ISR 不阻塞。
#define RX_QUEUE_SIZE 32U
// 发送帧初始化宏：标准 ID、数据帧、DLC=8、ESI 主动；brs/fmt 决定经典帧或 FD+BRS。
#define TX_FRAME(bus,brs,fmt) { .hcan = bus, .Header = { \
 .IdType = FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME, \
 .DataLength = FDCAN_DLC_BYTES_8, .ErrorStateIndicator = FDCAN_ESI_ACTIVE, \
 .BitRateSwitch = brs, .FDFormat = fmt, .TxEventFifoControl = FDCAN_NO_TX_EVENTS } }
FDCAN_TxFrame_TypeDef FDCAN1_TxFrame = TX_FRAME(&hfdcan1,FDCAN_BRS_OFF,FDCAN_CLASSIC_CAN);  // FDCAN1：关节电机，经典帧 1 Mbit/s
FDCAN_TxFrame_TypeDef FDCAN2_TxFrame = TX_FRAME(&hfdcan2,FDCAN_BRS_ON,FDCAN_FD_CAN);  // FDCAN2：双板通信，FD 帧 + 波特率切换
FDCAN_TxFrame_TypeDef FDCAN3_TxFrame = TX_FRAME(&hfdcan3,FDCAN_BRS_OFF,FDCAN_CLASSIC_CAN);  // FDCAN3：DJI 抬升电机，经典帧
FDCAN_RxFrame_TypeDef FDCAN_RxFIFO0Frame, FDCAN_RxFIFO1Frame;
// 每路健康统计、FDCAN1 关节维度的发送丢帧计数；can_recovery_epoch 在 FDCAN1 恢复时自增，供上层重发。
volatile CanBusHealth can_health[3];
volatile uint32_t can_joint_tx_drop[7], can_recovery_epoch;
// 索引 0/1/2 与 hfdcan1/2/3 的唯一映射；bus_index() 与所有数组都按此顺序寻址。
static FDCAN_HandleTypeDef *const buses[3] = {&hfdcan1,&hfdcan2,&hfdcan3};
// 队列元素：RX 头 + 8 字节数据 + 接收时刻（ms）；只收 DLC=8 的帧，故定长。
typedef struct {
    FDCAN_RxHeaderTypeDef hdr;
    uint8_t data[8];
    uint32_t tick_ms;
} RxFrame;
// 单生产者/单消费者环形队列：tail 由本总线 ISR 推进，head 由任务推进，无需加锁。
typedef struct {
    RxFrame frame[RX_QUEUE_SIZE];
    volatile uint32_t head, tail; /* task owns head; this bus's ISR owns tail */
} RxQueue;
static RxQueue queues[3];
// RX 分发任务句柄（BSP_CAN_RxTaskInit 记录），ISR 通过任务通知唤醒。
static TaskHandle_t rx_task;
// bus-off 事件标志、恢复阶段与尝试次数、恢复起点/统计窗口起点（均按总线索引）。
static volatile uint8_t busoff_event[3];
static uint8_t recovery_phase[3], attempts[3];
static uint32_t recovery_start[3], window_start[3];

/* 句柄 -> 总线索引（0/1/2）；未知句柄返回 -1，调用方据此拒绝处理。 */
static int bus_index(FDCAN_HandleTypeDef *hcan)
{
    for (int i=0; i<3; ++i) if (hcan == buses[i]) return i;
    return -1;
}
/* 统一检查 HAL 返回值：初始化阶段任何失败都进入 Error_Handler()。 */
static void checked(HAL_StatusTypeDef status)
{
    if (status != HAL_OK) Error_Handler();
}
/* 三路 FDCAN 初始化：过滤器、全局过滤策略、中断使能、FD 发送延迟补偿、启动。 */
// 调用上下文：系统初始化阶段（调度器启动前）；失败即停机。
void BSP_FDCAN_Init(void)
{
    for (unsigned i=0; i<3; ++i) {
        FDCAN_FilterTypeDef filter = {0};
        filter.IdType = FDCAN_STANDARD_ID;
        filter.FilterType = FDCAN_FILTER_MASK;
        filter.FilterConfig = i == 1U ? FDCAN_FILTER_TO_RXFIFO1 : FDCAN_FILTER_TO_RXFIFO0;  // FDCAN2 用 RX FIFO1，其余用 FIFO0

        /* 为各总线配置硬件过滤器，拒绝不相关的 ID 以降低软件队列压力 */
        if (i == 0U) {
            /* FDCAN1：达妙关节 0x01..0x07（bit0-2 关键，bit3+ 无关） */
            filter.FilterID1 = 0x000U;      // 基础 ID
            filter.FilterID2 = 0x7F8U;      // 掩码：0x7F8 = 1111111000b，允许 0x00*..0x07*
        } else if (i == 1U) {
            /* FDCAN2：双板 0x207/0x208/0x209/0x210（CC float data）和 0x212/0x213（遥控数据） */
            filter.FilterID1 = 0x200U;      // 基础 ID
            filter.FilterID2 = 0x700U;      // 掩码：0x700 = 111 0000 0000b，允许 0x200..0x2FF 范围
        } else {
            /* FDCAN3：DJI 抬升电机 0x201..0x204（bit0-2 关键） */
            filter.FilterID1 = 0x200U;
            filter.FilterID2 = 0x7F8U;      // 掩码：0x7F8 = 1111111000b，允许 0x200..0x207
        }
        checked(HAL_FDCAN_ConfigFilter(buses[i], &filter));
        checked(HAL_FDCAN_ConfigGlobalFilter(buses[i], FDCAN_REJECT, FDCAN_REJECT,
                                            FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE));  // 非匹配标准/扩展帧一律拒收，远程帧同样拒绝
        checked(HAL_FDCAN_ActivateNotification(buses[i], FDCAN_IT_BUS_OFF |  // 使能 bus-off 与本路接收新帧/溢出中断
            (i == 1U ? FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_MESSAGE_LOST :
                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST), 0U));
        if (i == 1U) {
            checked(HAL_FDCAN_ConfigTxDelayCompensation(buses[i],14U,14U));  // FD 帧发送延迟补偿参数 14/14（高波特率数据段必需）
            checked(HAL_FDCAN_EnableTxDelayCompensation(buses[i]));
        }
        can_health[i].tx_free_min = 8U;  // 初始化为发送 FIFO 深度，后续记录历史最小空闲水位
        checked(HAL_FDCAN_Start(buses[i]));  // 启动后即可收发（中断已使能）
    }
}
/* 发送一帧：临界区内检查 bus-off/阻塞状态后写 HAL 发送 FIFO。 */
// 参数：frame 预填好的发送帧（hcan 决定走哪路）；返回：HAL_OK 已入队，HAL_ERROR 被拒绝。
// 注意：调用点（CAN_Task）丢弃返回值；丢帧由 can_health[].tx_drop 与 can_joint_tx_drop[] 统计。
HAL_StatusTypeDef USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame_TypeDef *frame)
{
    HAL_StatusTypeDef result;
    int bus = bus_index(frame->hcan);  // 未知句柄直接拒绝
    if (bus < 0) return HAL_ERROR;
    taskENTER_CRITICAL();  // 与 ISR 共享的健康计数需要临界区保护
    uint32_t free = HAL_FDCAN_GetTxFifoFreeLevel(frame->hcan);
    if (free < can_health[bus].tx_free_min) can_health[bus].tx_free_min = (uint8_t)free;  // 记录发送 FIFO 空闲水位的历史最小值，用于诊断拥塞
    if (can_health[bus].blocked || (frame->hcan->Instance->PSR & FDCAN_PSR_BO))  // 总线阻塞或处于 bus-off 时不再入队
        result = HAL_ERROR;
    else result = HAL_FDCAN_AddMessageToTxFifoQ(frame->hcan, &frame->Header, frame->Data);  // 直接写发送 FIFO，不等待发送完成
    if (result != HAL_OK) {
        can_health[bus].tx_drop++;
        if (bus == 0 && frame->Header.Identifier >= 1U && frame->Header.Identifier <= 7U)  // FDCAN1 的 0x01..0x07 为关节电机，按 ID 单独累计
            can_joint_tx_drop[frame->Header.Identifier - 1U]++;
    }
    taskEXIT_CRITICAL();  // 临界区结束
    return result;
}
/* 唤醒 RX 分发任务：ISR 上下文轻量通知，必要时触发 PendSV。 */
static void wake_rx(void)
{
    BaseType_t woken = pdFALSE;
    if (rx_task != NULL) vTaskNotifyGiveFromISR(rx_task, &woken);  // 任务未注册时（初始化早期）静默跳过
    portYIELD_FROM_ISR(woken);  // 仅在需要时让出中断
}
/* 从指定 RX FIFO 排空所有待收帧并写入软件队列（ISR 上下文）。 */
// 参数：bus 总线索引，fifo 硬件 FIFO 号；返回：无。非法帧丢弃并计数，队列满时丢新帧并继续排空硬件。
static void capture(unsigned bus, uint32_t fifo)
{
    FDCAN_RxHeaderTypeDef hdr;
    uint8_t data[64]; /* HAL copies DLC bytes, even for frames we reject. */
    RxQueue *q = &queues[bus];
    while (HAL_FDCAN_GetRxFifoFillLevel(buses[bus], fifo) != 0U) {  // 一次中断把硬件 FIFO 排空
        if (HAL_FDCAN_GetRxMessage(buses[bus], fifo, &hdr, data) != HAL_OK) break;
        uint32_t next = (q->tail + 1U) % RX_QUEUE_SIZE;  // tail+1 取模，留一格区分空/满
        if (hdr.IdType != FDCAN_STANDARD_ID || hdr.RxFrameType != FDCAN_DATA_FRAME ||
            hdr.DataLength != FDCAN_DLC_BYTES_8) {
            can_health[bus].rx_invalid++;  // 只接受标准数据帧且 DLC=8，其余计为非法
            continue;
        }
        if (next == q->head) {
            can_health[bus].rx_overflow++; /* drop new, keep draining HW */
            continue;
        }
        q->frame[q->tail].hdr = hdr;
        memcpy(q->frame[q->tail].data, data, 8U);  // 固定复制 8 字节，与 DLC 校验一致
        q->frame[q->tail].tick_ms = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;  // 接收时间戳（ms），供上层做超时判断
        __DMB();  // 保证数据先于 tail 发布
        q->tail = next;
    }
    wake_rx();  // 排空后统一唤醒一次，减少无谓调度
}
/* RX FIFO0 中断：服务 FDCAN1（关节）与 FDCAN3（DJI 抬升），其他总线忽略。 */
// 参数：hcan 触发句柄，its 中断标志；返回：无。
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hcan, uint32_t its)
{
    int bus = bus_index(hcan);
    if (bus != 0 && bus != 2) return;
    if (its & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) can_health[bus].rx_overflow++;  // 硬件 FIFO 溢出（丢帧）单独计数
    capture((unsigned)bus, FDCAN_RX_FIFO0);  // 搬运 FIFO0 的全部帧
}
/* RX FIFO1 中断：仅 FDCAN2（双板 FD 帧）使用 FIFO1，其他句柄直接返回。 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hcan, uint32_t its)
{
    if (hcan != &hfdcan2) return;
    if (its & FDCAN_IT_RX_FIFO1_MESSAGE_LOST) can_health[1].rx_overflow++;  // FDCAN2 溢出计数
    capture(1U, FDCAN_RX_FIFO1);  // 搬运 FIFO1
}
/* 错误状态中断：检测到 bus-off 时置事件、标记阻塞并唤醒任务做恢复。 */
// 中断里不做 Stop/Start，恢复动作全部放在任务上下文。
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hcan, uint32_t its)
{
    int bus = bus_index(hcan);
    if (bus >= 0 && (its & FDCAN_IT_BUS_OFF) && (hcan->Instance->PSR & FDCAN_PSR_BO)) {
        busoff_event[bus] = 1U;  // 事件交给 BSP_CAN_HealthStep 处理
        can_health[bus].blocked = 1U;  // 阻塞发送，直到恢复完成
        wake_rx();  // 唤醒任务尽快处理
    }
}
/* 记录 RX 分发任务句柄：需在首次接收中断前调用。 */
void BSP_CAN_RxTaskInit(void)
{
    rx_task = xTaskGetCurrentTaskHandle();  // 通常由 CAN_Task 启动时调用
}
/* 任务侧分发：按总线轮转，每路最多取 RX_QUEUE_SIZE 帧，统一回调 APP_CAN_RxHandler。 */
// 参数：无；返回：无；调用上下文：CAN_Task 循环，可被中断抢占。
void BSP_CAN_ProcessRx(void)
{
    for (unsigned bus=0; bus<3; ++bus) {
        RxQueue *q = &queues[bus];
        /* Bound one visit, so a saturated bus cannot starve the others. */
        // 每路每次最多消费 32 帧，避免单路饱和饿死其他总线。
        for (unsigned n=0; n<RX_QUEUE_SIZE && q->head != q->tail; ++n) {
            __DMB();
            RxFrame frame = q->frame[q->head];
            __DMB();
            q->head = (q->head + 1U) % RX_QUEUE_SIZE;
#if (APP_ARM_ENABLE == 0)  // 整车构建的兼容路径：同一帧同时喂给 DJI/DM8009 电机对象
            for (unsigned j=0; j<4; ++j) {  // 固定前 4 个电机槽位
                if (bus == 0) DJI_Motor_Info_Update(&frame.hdr.Identifier,frame.data,&Chassis_Motor[j]);  // FDCAN1 -> 底盘 DJI 电机
                if (bus == 1) DM_Motor_Info_Update(&frame.hdr.Identifier,frame.data,&DM_8009_Motor[j]);  // FDCAN2 -> DM8009 双板电机
            }
#endif
            APP_CAN_RxHandler(buses[bus], frame.hdr.Identifier, frame.data, frame.tick_ms);  // 统一交给应用层按（总线, ID）分发
        }
    }
}
/* bus-off 恢复状态机（任务上下文，周期调用）：统计错误 -> 中止待发 -> 释放 INIT -> 等待恢复。 */
// 阶段：0 空闲 / 1 已中止发送、等 TXBRP 清零 / 2 已释放 INIT、等 BO 清除 / 3 锁定，需复位。
// 限流：60 s 窗口内最多 3 次尝试；单次恢复超时 1000 ms；失败即锁定并停止重试。
/* Single task owner: Moter_Task. ISR only records entry and blocks new TX.
 * M_CAN sets INIT on bus-off. Clear INIT once to request ISO recovery;
 * merely waiting for BO to clear can wait forever. Never Stop/Start in ISR.
 * Stop/Start does not flush TX: cancel TXBRP explicitly BEFORE release. */
void BSP_CAN_HealthStep(uint32_t now)
{
    for (unsigned i=0; i<3; ++i) {
        FDCAN_HandleTypeDef *h = buses[i];
        FDCAN_ErrorCountersTypeDef ec;
        if (HAL_FDCAN_GetErrorCounters(h, &ec) == HAL_OK) {
            can_health[i].tec = (uint8_t)ec.TxErrorCnt;  // TEC/REC 错误计数
            can_health[i].rec = (uint8_t)ec.RxErrorCnt;
            if (ec.TxErrorCnt >= 96U || ec.RxErrorCnt >= 96U) can_health[i].flags |= CAN_HEALTH_WARNING;  // >=96 进入 warning，置健康标志
            if (h->Instance->PSR & FDCAN_PSR_EP) can_health[i].flags |= CAN_HEALTH_PASSIVE;  // 错误被动（EP）标志
        }
        if (now - window_start[i] >= 60000U) { window_start[i] = now; attempts[i] = 0U; }  // 60 s 统计窗口到期，清零尝试次数
        if (recovery_phase[i] == 0U && (busoff_event[i] || (h->Instance->PSR & FDCAN_PSR_BO))) {  // 空闲态检测 bus-off 事件或硬件 BO 标志
            busoff_event[i] = 0U;
            can_health[i].blocked = 1U;
            can_health[i].flags |= CAN_HEALTH_BUSOFF;
            can_health[i].busoff_count++;  // 累计 bus-off 次数
            if (i == 0U) can_recovery_epoch++;  // FDCAN1 恢复后通知上层重发关节指令
            recovery_start[i] = now;
            if (attempts[i] >= 3U) {  // 超过尝试上限：锁定，等待人工复位
                can_health[i].flags |= CAN_HEALTH_RECOVERY_LIMIT;
                recovery_phase[i] = 3U; /* reset required */
                continue;
            }
            attempts[i]++;
            if (HAL_FDCAN_AbortTxRequest(h, h->Instance->TXBRP) != HAL_OK) {  // 先撤销所有待发请求（Stop/Start 不清 TX FIFO）
                can_health[i].flags |= CAN_HEALTH_HAL_ERROR;
                recovery_phase[i] = 3U;
                continue;
            }
            recovery_phase[i] = 1U;  // 进入等待 TXBRP 清零阶段
        }
        if (recovery_phase[i] == 1U) {
            if (h->Instance->TXBRP == 0U) {  // 待发请求已清空
                CLEAR_BIT(h->Instance->CCCR, FDCAN_CCCR_INIT);  // 清 INIT，触发 ISO 11898-1 恢复流程
                recovery_phase[i] = 2U;
            } else if (now - recovery_start[i] > 1000U) {  // 1 s 内未清空视为超时，锁定不再重试
                can_health[i].flags |= CAN_HEALTH_RECOVERY_TIMEOUT;
                recovery_phase[i] = 3U; /* fail closed; never replay pending targets */
            }
        }
        if (recovery_phase[i] == 2U) {
            if ((h->Instance->PSR & FDCAN_PSR_BO) == 0U &&  // BO 已清除且 INIT 已释放 -> 恢复成功
                (h->Instance->CCCR & FDCAN_CCCR_INIT) == 0U) {
                can_health[i].recovery_count++;
                can_health[i].blocked = 0U;  // 解除发送阻塞
                recovery_phase[i] = 0U;
            } else if (now - recovery_start[i] > 1000U) {  // 阶段 2 同样受 1 s 超时约束
                can_health[i].flags |= CAN_HEALTH_RECOVERY_TIMEOUT;
            }
        }
    }
}
