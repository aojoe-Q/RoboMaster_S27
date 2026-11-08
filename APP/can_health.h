/* can_health.h —— CAN 总线健康度统计（FDCAN1/2/3）
 *
 * 用途：为上层提供“总线是否可靠”的判据，例如 bus-off 后禁止继续发帧，
 *       或者连续丢帧时降级控制。所有字段均由 BSP 层周期更新，控制层只读。
 *
 * 结构：can_health[3] 保存每路总线的计数与标志位；can_joint_tx_drop[7]
 *       单独统计 7 个关节的发送丢弃；can_recovery_epoch 用“纪元”方式
 *       通知上层“发生过一次恢复，之前的帧可能已丢失”。
 */
#ifndef CAN_HEALTH_H
#define CAN_HEALTH_H
#include <stdint.h>
#define CAN_HEALTH_BUSOFF 1U // bit0：总线进入 bus-off
#define CAN_HEALTH_WARNING 2U // bit1：错误计数越过告警阈值
#define CAN_HEALTH_PASSIVE 4U // bit2：错误被动（error passive）状态
#define CAN_HEALTH_RECOVERY_LIMIT 8U // bit3：自动恢复尝试次数已达上限
#define CAN_HEALTH_RECOVERY_TIMEOUT 16U // bit4：恢复过程超时
#define CAN_HEALTH_HAL_ERROR 32U // bit5：HAL/驱动层报错
typedef struct { // 单路总线的健康统计：flags 为上方 CAN_HEALTH_* 的按位或
    uint32_t tx_drop, rx_overflow, rx_invalid, busoff_count; // tx_drop 发送被丢弃；rx_overflow 接收溢出；rx_invalid 非法帧；busoff_count bus-off 次数
    uint32_t flags, recovery_count; // flags 见上方位定义；recovery_count 自动恢复成功次数
    uint8_t tec, rec, tx_free_min, blocked; // tec/rec 发送/接收错误计数；tx_free_min 见过的最小空闲发送槽；blocked 是否已被判阻塞
} CanBusHealth;
extern volatile CanBusHealth can_health[3]; // 下标 0..2 对应 FDCAN1/2/3
extern volatile uint32_t can_joint_tx_drop[7]; // 7 个关节各自的发送丢弃计数，下标 0..6 对应电机 ID 1..7
extern volatile uint32_t can_recovery_epoch; // 恢复纪元：每完成一次总线恢复自增，控制层用变化来感知“可能丢过帧”
void BSP_CAN_HealthStep(uint32_t now_ms); // 周期调用（传入当前毫秒时基），刷新 flags 与统计
void BSP_CAN_RxTaskInit(void); // 初始化 CAN 接收任务（由 BSP 启动流程调用一次）
void BSP_CAN_ProcessRx(void); // 在接收任务/ISR 上下文处理已收到的帧
#endif // CAN_HEALTH_H
