# 2026-11-07 评审方案实施记录

实施对象：`S27_TEST_2`。依据桌面的《S27_TEST_2_代码评审与修改方案》和《代码评审方案_技术问题清单_修正版》。源码改动已完成并进行主机测试、ARM 编译链接；未烧录，未执行机械臂实机验收。

修改前工作区干净，原始 HEAD 已备份至同级目录 `S27_TEST_2_before_review_20261107.zip`。改动保留为未提交工作区差异，便于审查。新增接口实现放在现有编译单元内，不需要手工向 Keil 增加 C 文件。

## 逐项落实

| 方案条目 | 实施结果 |
|---|---|
| P0-1 七轴输出 | `Moter_Task` 每 4 ms 取一次完整快照，循环内无延时；控制任务每 8 ms 绝对节拍发布。初始化、回零、标零中的阻塞点也发布快照。超过 200 ms 无反馈或电机反馈异常，锁存该轴 `EN_FAIL` 并停止该轴正常输出。 |
| P0-2 使能确认 | `WAIT_BOOT → SEND → OK/FAIL`，20 ms 间隔、最多 50 个尝试时隙，最后一次仍保留反馈等待窗口。要求对应 ID、新反馈计数、发送后时间戳、新鲜度和 DM 状态高半字节 `1`；HAL_OK 和 CAN ACK 均不能代替确认。 |
| P0-2 bus-off | ISR 只记录与唤醒，输出任务处理恢复；三路自动重传继续关闭。每总线每 60 s 最多三次恢复请求，超限锁存到复位。恢复先取消旧 TX 请求，再启动协议恢复；详细差异见下一节。 |
| P0-3 编码 | 统一 `motor_encoding.h`、`uint16_t float_to_uint_clamped`；DM 和 HT 调用同步。MIT 非有限参数整帧拒绝，DM 按 ID 计数；边界饱和，不回绕。浮点类型重解释改为 memcpy。 |
| P0-4 PC 限速 | 所有 PC 位置命令均经速度、加速度限制；不按主机种类绕过。现阶段七轴默认/硬速度上限 0.5 rad/s，加速度上限 1.0 rad/s²，实测 dt，单次 dt 最大 16 ms。参数位于 `arm_motion.h`，必须结合机构、负载实测标定。 |
| P0-5 发送可观测 | HAL 返回值逐总线/关节计数；FIFO 空闲不足 2 时延期，下一周期使用最新快照，轮换起始关节避免饥饿。累计计数、TEC/REC、bus-off、队列溢出通过诊断帧上报。 |
| P1-1 RX 中断 | 每总线独立 32 槽 SPSC 队列（可用 31 槽），保存接收时间，DMB 发布；通知 CAN_Task 解码。队列满丢新帧并继续排空硬件 FIFO；检查总线、帧类型、DLC、DM 内嵌 ID/状态。HAL 接收暂存区按 64 字节配置，避免异常 DLC 写越界。 |
| P1-2/P1-3 节拍、共享数据 | 绝对节拍、一致命令快照；CAN 单个反馈先私有解码再短临界区发布，USB 从输出快照读取 target；双板单帧数据短临界区发布。CAN_Task 提至 High，输出仍为唯一 Realtime。 |
| P1-4 模式消抖 | 模式和夹爪分支均经三个控制采样确认，读取同一份左右拨杆快照；遥控丢失立即退回无力模式。 |
| P1-5 带宽 | 采用 250 Hz 软件目标，保留 FDCAN1/2/3 物理拓扑及自动重传配置；实际负载、帧间差、反馈年龄待分析仪验收。 |
| P1-6 速度字段 | 选择“速度限制”语义；不复用为前馈，不改变 Kp/Kd。增加速度前馈须另行版本设计、主机同步和实机标定。 |
| P1-7 启动 | 保留 950 ms 电源等待，关节确认最长 1.5 s，去除 3.6 s 固定等待；夹爪运动判定 2 s、总搜索上限 12 s，回零增加 20 s 上限。启动、夹爪标零或回零失败均保持未就绪、无力输出，修复后复位重新初始化。 |
| P1-8 夹爪 | 力矩分支移到控制快照构造层，约 100 ms 在位置增益与 0.2 N·m 力矩之间过渡。标零仍使用独立 0.5 N·m 配置；仅用新反馈判断堵转，撤力矩后存零一次，再等待新零位反馈确认。 |
| P2-1 编码 | 新增、重写代码采用 UTF-8/ASCII；混合编码文件按字节保留未改区域。历史损坏注释未整文件转码，无法恢复的内容保留在备份；这项可选清理没有伪装为已全面完成。 |
| P2-2 模板隔离 | DM8009/底盘演示接收分支与原发送分支一致受 `APP_ARM_ENABLE==0` 隔离。保留实际模块、USB/遥控/独立总线初始化链路，未按名称盲删 DJI/HT/LK。 |
| P2-3/P2-4/P2-5 | 修复重复发送 DM8009[2] 而遗漏 [3]；修复类型双关；CAN_Task 使用 xTaskGetTickCount（CMSIS v1 API 并非“废弃”）。 |
| P2-6 栈溢出 | `configCHECK_FOR_STACK_OVERFLOW=2`，Hook 保留任务句柄/名称并停机。损坏栈上不执行阻塞打印；调试器读取 `stack_overflow_task/name`。 |
| P2-7 空循环 | 弱实现空循环由 1 ms 降到 100 ms；USB 初始化原位置保留，真实 INS/Control/Detect 强实现未降频。CAN 空循环已替换为队列消费/通知等待。 |

## 对方案示例作出的必要修正

1. **M_CAN bus-off 并不是只等待 BO 清零。** 进入 bus-off 时硬件会设置 `CCCR.INIT`，软件需要清除 INIT 才能请求协议恢复。当前实现仅在任务中、完成旧 TX 请求取消后清除一次 INIT，再等待 `PSR.BO/CCCR.INIT` 退出，不在 BO=1 时反复 Stop/Start。文档两版关于“完全无需操作 INIT”的说法不适合作为本硬件实现依据。
2. **HAL Stop/Start 不等于清空 TX FIFO。** 本地 HAL 的 Stop/Start 只操作控制状态；使用 `HAL_FDCAN_AbortTxRequest(TXBRP)` 并检查 TXBRP 已清空。若取消请求 1 s 内无法完成，锁存故障并禁止恢复输出，避免旧目标重放。取消请求在实际 H723 bus-off 状态下的完成时序必须上板验证；失败时保持禁止输出，不会假定已经清空。
3. **恢复不自动续跑。** 运行中某轴失败后不会被偶然反馈自动重新启用；需要拨杆重新进入无力模式产生一次明确重试请求。整条 FDCAN1 bus-off 后额外锁存 `bus_inhibit`，清故障并确认无力快照后才能重试，返回 PC 模式仍需新的 USB ENABLE。初始化失败需要复位。
4. **FIFO 压力不排队重放历史轨迹。** 延后后使用下一周期的最新完整快照；使能尝试也受时隙上限约束。若长期无法接受命令，计数可见且使能最终失败。
5. **速度限制与加速度限制的边界。** 正常 PC 目标改变受两者约束；主机突然降低速度上限时按加速度上限制动到新的巡航限值。STOP、失联、反馈失效仍沿用锁存实测位置的保持行为，属于故障动作，不承诺这一瞬间满足普通轨迹加速度上限。固件静态位置限位仍最后执行。

## USB 兼容扩展

帧版本继续 `0x02`，STATE 138 B、POSITION_CMD 64 B 不变。POSITION_CMD 的速度数组是 little-endian signed int32，单位 μrad/s：0 用固件默认，正值只收紧速度上限，负值拒绝整条命令并返回 BAD_VALUE。旧主机发全 0 仍能使用。

新增 STATE.flags：bit1 曾发生 TX 失败，bit2 CAN 健康异常，bit3 CAN RX 溢出，bit4 DM 使能/启动失败；bit0 仍为 ARM_READY。fault_code 新增 `0x0010 CAN_BUS`、`0x0020 DM_ENABLE`。feedback_valid 同时要求新鲜且电机状态为已使能。

新增 MCU→PC `0x08 DIAGNOSTIC`，每 500 ms 交替发送一个页面，各页面约 1 Hz。旧主机可按帧长跳过未知类型；本工程 Python 解码器和监视工具已支持。每页开头为 `<BBI`：schema=1、page、RTOS timestamp_ms。

| page | payload 长度 | 后续记录 |
|---|---|---|
| 0 | 90 B | 三总线，每条 `<6I4B`：tx_drop、rx_overflow、rx_invalid、busoff_count、flags、recovery_count、TEC、REC、tx_free_min、blocked |
| 1 | 160 B | 七电机，每条 `<5I2B`：invalid_command、tx_drop、offline_skip、fifo_defer、first_enable_feedback_ms、enable_state、attempts |

state=0/1/2/3 分别为 WAIT_BOOT/SEND/OK/FAIL；first_enable_feedback_ms 表示本次启动首次确认使能的时刻，不等于物理上电时刻。计数为 uint32，允许回绕，累计至复位；CAN flags 位定义见 `can_health.h`，WARNING 与 ERROR_PASSIVE 分开。

只读采集：

```powershell
python tools/arm_host_monitor.py --port COM7 --csv arm_state.csv --health-jsonl can_health.jsonl
```

ROS2 现有全零速度字段无需升级即可兼容；MoveIt/ros2_control 的关节速度、加速度约束应与固件校准结果一致。外部 `ros_test_8` 工程不在本次修改范围内。

## 验证与产物

```powershell
python tools/selftest/run_selftest.py
python tools/selftest/run_control_selftest.py
python tools/selftest/armclang_check.py
python tools/selftest/full_arm_build.py
```

- USB C/Python 双向回归：会话、CRC、过期、重放、使能/撤防、反馈看门狗、限速字段、两页诊断的 32 位计数和字段顺序。
- 控制测试直接编译实际 `DM_motor.c`、`Moter_task.c`、`bsp_can.c`：编码边界/NaN/Inf、速度/加速度/反向阶跃、使能新反馈、50 次上限、单轴离线、显式恢复、FIFO 公平延期、bus-off 限次/取消失败、队列溢出/异常 DLC。
- 完整 ARM 构建使用当前 Keil 工程记录参数，重新编译 107 个编译单元并链接，产物 `tools/selftest/build/full_arm/project.axf` 和 `project.hex`；编译/链接日志在同目录。实际工具链是 Arm Compiler 6.21，原方案写的 6.19 与当前工程不一致。
- 主机假 HAL 测试验证软件分支，不能证明物理总线恢复、机械行为或调度抖动。未修改 MPU、Cache、DMA section 或 scatter 布局。

实机待验收：连续 30 次上电（含电机延迟上电）、夹爪空载标零、单轴断开/重新上线、无 ACK 与 bus-off 故障注入、FIFO/队列压力、250 Hz 最坏负载和帧间差、反馈年龄 <20 ms、实际轨迹速度/加速度/跟踪误差。每项需记录时间戳、总线分析仪数据及健康日志；不能用通过编译替代验收。


### ??????

- USB ???0 failures?STATE ??????????????? Python ?????
- ?????encoding / motion / enable / retry / fifo / bus / queue ??????????? `-Wall -Wextra -Werror`?
- ?????????50 Hz ???? 48.4 Hz?100 Hz ???? 96.8 Hz????????????????????? CAN ????
- ?? ARM ???107 ???0 failed?0 compiler warnings??????Code 156148 B?RO-data 2500 B?RW-data 960 B?ZI-data 133800 B?
- ?? `armclang_check.py` ??? `-Wall -Wextra` ??????????/????????????0 warnings?? Keil ??????????????????????
