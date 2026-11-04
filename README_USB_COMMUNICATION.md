# DM-MC02 机械臂 USB CDC 协议与实现说明（v2）

文档修订：2026-11-02。适用：`S27_TEST_2` 固件与 `ros_test_8/src/dm_mc02_hardware`。
协议版本仍为 `0x02`；本次仅整理文档，不表示固件升级或台架验收通过。

本文用于协议开发、故障定位和测试；Ubuntu 操作顺序见 [实机联调手册](../ros_test_8/UBUNTU_SETUP.md)。
以下“当前实现”按本地源码核对。代码修改后必须重新核对本文；建议和待办不代表已实现能力。

## 1. 文件与排查入口

| 位置 | 职责 |
|---|---|
| [APP/arm_usb_protocol.h](APP/arm_usb_protocol.h)、对应 .c | 帧、字段、CRC、逐字节序列化，不依赖 C 结构体填充 |
| [APP/Arm_USB_task.h](APP/Arm_USB_task.h)、对应 .c | 10 ms USB 任务、超时常量、会话、校验、ACK/STATE/FAULT |
| [APP/ring_buffer.h](APP/ring_buffer.h)、对应 .c | USB 中断写、USB 任务读的环形缓冲 |
| [USB_DEVICE/App/usbd_cdc_if.c](USB_DEVICE/App/usbd_cdc_if.c) | CDC 初始化/结束、RX 入队、TX 提交 |
| [Components/Device/Src/MiniPC.c](Components/Device/Src/MiniPC.c) | USB 连接、发送和忙状态 |
| [APP/Compat/app_can.c](APP/Compat/app_can.c) | CAN 反馈及 update_tick/update_count |
| [APP/Arm_L_task.c](APP/Arm_L_task.c)、对应 .h | PC 模式、保持目标、静态限位及遥控模式 |
| [tools/arm_usb_protocol.py](tools/arm_usb_protocol.py) | Python 编解码 |
| [tools/arm_host_monitor.py](tools/arm_host_monitor.py) | 只读观察 STATE/ACK/FAULT，保存 CSV |
| [tools/arm_host_control.py](tools/arm_host_control.py) | ENABLE、绝对位置目标、退出时 STOP |
| [tools/selftest](tools/selftest) | 主机自测、发送频率、ARMCLANG 检查 |

数据路径：USB RX → 环形缓冲 → Arm_USB_Task → PC 目标缓存 → Arm_PC_Control → 电机发送任务。
STATE 的反馈来自 CAN 电机反馈，不是“命令已执行”的证明。STATE 位置与 POSITION_CMD 目标都是逻辑坐标 P = p + b（见 APP/multi_turn_restore.h）；`MT_RESTORE_ENABLE=0` 时 b=0，与原始电机角度相同。

## 2. 帧格式和单位

所有多字节数小端；有符号整数为 int32。

| 帧内偏移 | 字节数 | 字段 |
|---:|---:|---|
| 0 | 1 | SOF=0xA5 |
| 1 | 1 | version=0x02 |
| 2 | 1 | type |
| 3 | 1 | sequence；各发送方独立计数，uint8 回绕 |
| 4 | 2 | payload_len，最大 160 |
| 6 | 4 | timestamp_ms，发送方时钟 |
| 10 | N | payload |
| 10+N | 2 | CRC16，低字节在前 |

总长 `12 + payload_len`，最大 172 B。CRC16-CCITT-FALSE：poly=0x1021、init=0xFFFF、无反射、xorout=0；覆盖 CRC 前全部字节。校验向量 `123456789 → 0x29B1`。

线上的角度/角速度分别为微弧度、微弧度/秒（rad × 1,000,000），力矩为 mN·m（N·m × 1,000）。Python monitor 已转换为 rad、rad/s、N·m。

| 数组索引 | 电机 | ROS 名称 | ROS 位置单位 |
|---:|---|---|---|
| 0–5 | J1–J6 | joint1–joint6 | rad |
| 6 | Grip | joint8 | m，经夹爪映射 |

线上第 7 项仍是夹爪电机角度，不能把 ROS joint8 的米数直接写入该项。Python 控制工具使用编号 1–7，夹爪是 `--joint 7=...`，不是 8。

## 3. 消息与负载

| type | 方向 | payload 字节数 | 说明 |
|---|---|---:|---|
| 0x01 STATE | MCU→PC | 138 | USB 任务目标 100 Hz；忙时可能跳过发送 |
| 0x02 POSITION_CMD | PC→MCU | 64 | 7 个电机绝对目标 |
| 0x03 ENABLE | PC→MCU | 4 | 使能/失能 |
| 0x04 STOP | PC→MCU | 0 | 撤销 PC 使能和目标 |
| 0x05 HEARTBEAT | PC→MCU | 0 | 维持链路活动，不延长位置命令有效期 |
| 0x06 ACK | MCU→PC | 4 | 请求类型、序号、结果 |
| 0x07 FAULT | MCU→PC | 4 | 故障位变化通知；持续状态以 STATE 为准 |

### 3.1 POSITION_CMD

以下偏移均相对 payload 起点。

| 偏移 | 类型 | 字段 |
|---:|---|---|
| 0 | u32 | command_timestamp_ms |
| 4 | u16 | valid_time_ms |
| 6 | u8 | enable；发送方只使用 0/1 |
| 7 | u8 | reserved，填 0 |
| 8 | i32[7] | position |
| 36 | i32[7] | velocity_limit，保留，全部填 0 |

当前固件按以下规则处理：

1. 长度必须为 64 B；任一目标超出静态限位，整帧返回 OUT_OF_RANGE，保留上一目标（直到其过期或被撤防）。
2. 必须先成功 ENABLE；POSITION_CMD.enable=1 本身不能使能。
3. enable=0 清除当前目标有效位，但不等同于 ENABLE(0)/STOP；仍需满足当前使能和时间戳检查。
4. `valid_time_ms` 实际采用钳位：0 或大于 300 → 300 ms；1–9 → 10 ms；10–300 保持原值。不是越界就 BAD_VALUE。
5. command_timestamp_ms 使用主机单调毫秒时钟；连续已使能时段内必须更新，等于或早于上一值返回 STALE。比较使用 32 位差值，允许正常 uint32 回绕；不要用墙钟或重发旧时间戳。
6. 每次成功 ENABLE 会重置命令时间基线；USB 新会话必须重新 ENABLE。
7. 排队时间按 `mcu_ms-host_ms` 相对历史最小差估计，从有效期扣除；排队时间 ≥ 有效期返回 STALE。基线最多每秒跟随 1 ms，以容纳时钟漂移。首帧建立基线，不能据此检测所有绝对传输延迟。

### 3.2 ENABLE / STOP / HEARTBEAT

ENABLE：`enable u8, reserved u8[3]`。保留字段填 0。

- ENABLE(1)：要求当前会话有效、arm_mode=10、Arm_L_Init 已完成、全部 7 个关节反馈年龄 ≤100 ms。
- 成功后清除故障位、清除旧目标和命令时间基线；等待新 POSITION_CMD。
- ENABLE(0) 与 STOP 均撤防、清除目标有效位，但不清除已锁存的故障位。
- HEARTBEAT 只刷新链路活动；已有位置命令仍会到期。
- STOP/HEARTBEAT 的发送负载必须为 0；当前处理分支未单独校验这两类的负载长度，不能据此依赖非标准帧。

### 3.3 STATE

| payload 偏移 | 类型 | 字段及含义 |
|---:|---|---|
| 0 | u32 | timestamp_ms，MCU tick |
| 4 | u8 | arm_mode，PC 模式为 10 |
| 5 | u8 | system_state，见第 4 节 |
| 6 | u16 | fault_code，位掩码 |
| 8 | u8 | flags，bit0=Arm_L_Init 完成 |
| 9 | u8 | feedback_valid，bit i=该电机反馈年龄 ≤100 ms |
| 10 | u16 | reserved |
| 12 | u16[7] | feedback_age_ms，0xFFFF=从未收到，最大正常值饱和到 0xFFFE |
| 26 | i32[7] | position，电机反馈 |
| 54 | i32[7] | velocity，电机反馈 |
| 82 | i32[7] | torque，电机反馈 |
| 110 | i32[7] | target_position，控制输出 send_set |

位置、速度、力矩在同一临界区复制；这不意味着 7 台电机的 CAN 采样发生在同一时刻。
全部有效的掩码为 `0x7F`。当前阈值是 **100 ms**，旧文档的 50 ms 已过时。
判断可用性要同时观察 flags、feedback_valid、feedback_age_ms 和 STATE 接收新鲜度。

### 3.4 ACK / FAULT

ACK：`acked_type u8, acked_sequence u8, result u8, reserved u8`。

ENABLE、STOP、未知类型及版本错误会排队 ACK；POSITION_CMD/HEARTBEAT 正常接受时不 ACK，拒绝时 ACK。CRC 错误/无法成帧的数据丢弃并重新同步，不应等待其 ACK。ACK 队列长度 8，溢出可能丢弃旧应答；主机必须处理超时。

| result | 名称 | 定位方向 |
|---:|---|---|
| 0 | OK | 请求被接受，不代表电机到位 |
| 1 | BAD_LENGTH | ENABLE/POSITION_CMD 负载大小 |
| 2 | BAD_VERSION | 协议版本 |
| 3 | NOT_ENABLED | 未使能或使能会话不匹配 |
| 4 | OUT_OF_RANGE | 任一电机目标越界 |
| 5 | UNKNOWN_TYPE | 消息类型 |
| 6 | BAD_VALUE | 例如 ENABLE 值不是 0/1 |
| 7 | STALE | 时间戳未更新或排队过期 |
| 8 | NOT_READY | 初始化或反馈条件未满足 |
| 9 | NO_SESSION | MCU 内部结果，不发出 |
| 10 / 0x0A | WRONG_MODE | ENABLE 时 arm_mode 不为 10 |

FAULT：`fault_code u16, system_state u8, reserved u8`。

| fault_code 位 | 名称 | 当前效果 |
|---|---|---|
| 0x0001 | LINK_TIMEOUT | 已使能时链路 300 ms 无活动，撤防 |
| 0x0002 | USB_DISCONNECT | 已使能时会话结束、替换或挂起，撤防 |
| 0x0004 | RX_OVERFLOW | 接收缓冲丢字节，记录故障；当前不单独撤防 |
| 0x0008 | FEEDBACK_LOST | 已使能时任一 CAN 反馈超时，撤防 |

故障位累积到成功 ENABLE(1)，或固件重新初始化。并非任意非零 fault_code 都等于 system_state=FAULT，尤其 RX_OVERFLOW。

## 4. 状态、会话和停止边界

### 4.1 固件状态与 ROS 状态分开读

| 固件 STATE.system_state / ROS 日志 sys | 意义 |
|---:|---|
| 0 IDLE | PC 未使能，且无导致 FAULT 的锁存位 |
| 1 ENABLED | 已使能，无新鲜有效目标 |
| 2 RUNNING | 已使能且目标有效；不保证电机运动/到位 |
| 3 FAULT | 已撤防，锁存 LINK_TIMEOUT、USB_DISCONNECT 或 FEEDBACK_LOST |

ROS 日志 `State` 则是 ArmLink 状态：**0 IDLE、1 WAIT_ENABLE_ACK、2 WAIT_BASELINE、3 ARMED、4 FAULT**。
正常流是 ROS `0→1→2→3`，固件 `0→1→2`；不能把 ROS State=2 读成 ARMED。

### 4.2 会话隔离与发送保护

CDC_Init_HS 开始会话，CDC_DeInit_HS 结束会话，USB 中断递增会话编号。无打开会话时 RX 字节直接丢弃。
使能绑定会话编号；目标读取函数同步检查，旧目标在会话变化后立即不可用，不必等下次 USB 任务。

任务丢弃旧会话字节，保留新会话起点之后的数据；若会话在解析期间变化，旧帧不提交。特殊竞争导致新会话字节已被解析消耗时，主机需处理 ENABLE 超时并重试。
USB 挂起即使未销毁 CDC 类，也会因不再 CONFIGURED 而使目标无效。

CDC_Transmit_HS 在 PRIMASK 临界区检查设备状态、类指针、TxState 并提交，只排队不等待完成，避免复位中断在检查与使用之间清空指针。

### 4.3 三类时限

| 层级 | 当前值 | 到期后的结果 |
|---|---|---|
| 固件 CAN 有效性/ENABLE/watchdog | 100 ms | 已使能时撤防并置 FEEDBACK_LOST；目标读取立即失败 |
| 固件单条命令 | 10–300 ms，ROS 默认发 100 ms | 目标失效；若心跳继续，仍可保持 ENABLED |
| 固件链路活动 | 300 ms | 已使能时撤防并置 LINK_TIMEOUT |
| ROS STATE 接收超时 | 默认 100 ms，加载参数时上限钳为 100 | 等待/ARMED 中撤防；位置置 NaN |

当前固件通过 CRC 并送入处理器的当前会话帧，即使返回部分拒绝结果，也会刷新链路活动。
因此不能把“没发合法运动命令”直接等同于“300 ms 后必然 LINK_TIMEOUT”；命令新鲜度另有期限。

### 4.4 PC 模式和机械行为

DT7 左上+右上进入 mode=10，覆盖该位置原取水动作。离开时 Arm_USB_PC_Disarm 撤销 PC 使能，回到 PC 模式须重新 ENABLE。
PC 主任务约 8 ms 一次；有效目标写入 J1–J6/Grip.send_set，使用正常 Kp/Kd，并同步斜坡状态。

仍在 PC 模式时，STOP/命令过期/断链/反馈丢失会转入保持逻辑，锁存 ArmData 中的位置并施加静态限位。反馈丢失时该值可能已陈旧，不能当作可靠的物理位置闭环保证。
**退出 PC 模式后执行新拨杆模式，可能运动；遥控失联/ARM_POWERLESS 可能失力下垂。STOP 不是全局急停。**

PC 模式和 ROS 插件当前都不执行 J1/J2 碰撞区限位。人工避让、模型碰撞检查和静态限位是不同层次，不能互相替代。

## 5. 静态限位和映射

数值来自 Arm_L_task.h；ROS Xacro 当前与其一致。单位为 MCU rad。

| 电机 | 最小 | 最大 |
|---|---:|---:|
| J1 | -2.96054893 | 2.81695309 |
| J2 | 0 | 1.7732357 |
| J3 | 0 | 2.39008179 |
| J4 | -4.3 | 4.3 |
| J5 | -1.29562798 | 1.23497334 |
| J6 | -2.25160637 | 2.22337666 |
| Grip | 0 | 0.7 |

joint1–6：`q_ros = sign * (q_mcu - offset)`，sign 只使用 ±1；反向转换 `q_mcu = sign*q_ros + offset`。
ROS 可用区间应取“转换后的 MCU 区间”与 URDF/MoveIt 限位的交集，负 sign 时需重排上下界。

夹爪线性映射：

~~~text
t = (motor_rad - grip_mcu_closed) / (grip_mcu_open - grip_mcu_closed)
joint8_m = grip_q_close_m + t * (grip_q_open_m - grip_q_close_m)
~~~

两端值必须独立测量；不能把已映射的 joint8 反馈当作原始电机角度或独立标定。
当前 check_limits 要求 mcu_closed ≤ motor_rad ≤ mcu_open；若实际闭合/张开方向相反，需先修正映射/限位实现，不能直接交换参数后使能。

## 6. ROS 当前实现与后续工作

| 项目 | 当前行为 | 后续验收目标 |
|---|---|---|
| 激活 | on_activate 仅遥测；allow_enable=1 且有位置接口被占用后才尝试 ENABLE | 无运动控制器时不使能 |
| 门控 | mode=10、非 SYS_FAULT、ready、全反馈有效 | 非 PC 模式本地不发 ENABLE，不一定收到 WRONG_MODE |
| ENABLE | 等 ACK，超时 2 s；拒绝进入 ROS FAULT | 增强 ACK 序号匹配、诊断和有界重试 |
| 基准帧 | ACK 后先发当前位置，再按控制器目标发送 | 实测首次激活无突跳，不能仅凭首帧保证 |
| 发送 | 控制循环 100 Hz，位置帧间隔至少 20 ms，约 50 Hz；valid_time=100 ms | 检查抖动、短写和过期 |
| 空闲 | 当前 ArmLink 未主动发 HEARTBEAT；ARMED 持续发位置 | 心跳策略是开发项，不能当已实现 |
| 日志 | 每约 5 s 输出 State/mode/sys/计数；disarm reason 被丢弃 | 输出具体原因、ACK 结果和反馈年龄 |
| 无效反馈 | 阻止/撤销使能；仍可能输出上次数字值，仅 STATE 超时将位置置 NaN | 补充逐关节有效性诊断 |
| 故障恢复 | ROS 预检查拒绝 SYS_FAULT；MCU 锁存位又需成功 ENABLE 才清 | 设计明确恢复流程；不能承诺停/启控制器即恢复 |
| 串口 | 当前无完整自动重连流程 | 重连须重新建链和使能，禁止旧命令恢复 |

单帧步长限制由 ROS 插件执行，默认每关节 0.01 rad/20 ms（约 0.5 rad/s 的平均步长约束），不是固件速度/加速度规划。
Moter_Task 每轮等待合计约 17 ms，各电机发送错开，单电机约 59 Hz 以下；100 Hz STATE 不等于 100 Hz 同步执行。
不同电机型号的 P/V/T 量程仍需核对；当前公共力矩量程为 ±54 N·m。

## 7. 测试与证据

一次只能有一个串口所有者：ROS、monitor、control 三者轮流运行。另开 monitor 会与 ROS 争读数据。
以下命令从 S27_TEST_2 根目录执行；Ubuntu 用 python3，Windows 可用 python。monitor/control 需 pyserial。

~~~bash
# 离线；run_selftest 需要 gcc，armclang_check 需要 Keil/ARMCLANG 环境
python3 tools/selftest/run_selftest.py
python3 tools/selftest/host_rate_check.py
python3 tools/arm_host_control.py --dry-run --joint 1=0.1

# Ubuntu 只读采集；先停止 ROS
python3 tools/arm_host_monitor.py --list
python3 tools/arm_host_monitor.py --port /dev/ttyACM0 --verbose --duration 30 --csv state.csv
~~~

Windows 编译检查：

~~~powershell
python tools/selftest/armclang_check.py
~~~

control 工具的 --joint 是**绝对 MCU rad**，--duration 是重复发送该目标的时间，不是插值运动时间；不指定关节时保持刚读取的反馈。
首次持位测试可在已支撑、反馈正常且 mode=10 的台架执行：

~~~bash
python3 tools/arm_host_control.py --port /dev/ttyACM0 --duration 2
~~~

该命令仍会使能并发位置帧，结束发送 STOP；不是纯只读。--dry-run 不打开串口，也不能验证实机位置。
不要用 --force 绕过首次点动的幅度检查。

| 用例 | 环境/方法 | 验收 |
|---|---|---|
| CRC、长度、版本、拆包粘包 | 主机自测/协议注入 | 错帧不改变目标；可解析的版本错误 ACK=2 |
| WRONG_MODE / NOT_READY | 协议注入或自测 | ENABLE 分别 ACK=10/8；CLI 有本地门控，不一定发出请求 |
| STALE | 自测，旧时间戳和超期排队 | ACK=7，目标不覆盖 |
| 有效期钳位 | 自测 0、1、9、10、300、301 | 按第 3.1 节，不误报 BAD_VALUE |
| 越界 | 优先离线，完整 7 项且只越一项 | ACK=4，保留旧目标；不会仅因此置固件 FAULT |
| STOP | 有支撑台架，在 PC 模式 | 撤防并进入保持逻辑；记录实际停止响应 |
| 仅命令过期 | 停 POSITION_CMD、继续心跳 | 目标失效但不直接链路 FAULT |
| LINK_TIMEOUT | 已使能，停止全部主机发送 | 300 ms 门限后下一处理周期撤防 |
| CAN 反馈丢失 | 自测，再受控台架故障注入 | >100 ms 后目标无效、撤防、fault 位 0x0008 |
| USB/会话变化 | 受控台架拔线、重连、挂起 | 旧目标失效；重连需新 ENABLE |
| 模式退出/再进入 | 台架，事先确认目标拨杆模式 | 旧使能不恢复；不要求退出后一直保持位置 |
| RX 溢出 | 离线注入 | 记录 0x0004；不误判为已自动撤防 |

以上为测试要求，不是本次已执行结果。原文历史“编译通过/自测通过”没有附运行记录，不能代表当前版本已通过。
每项保存：日期、固件/ROS 版本、工具命令、日志、预期/实际、通过/失败/未执行及操作者。

## 8. 维护约定

协议字段、单位、数组顺序或状态变更时同步核对固件编解码、Python 工具、ROS 编解码、测试及两份文档。
超时/门控/恢复变更即使不改协议版本，也必须更新行为说明和验收矩阵。
新增功能标记为“待实现”，有源码和测试证据后再改为“当前实现”。

| 日期 | 变更 | 证据/剩余工作 |
|---|---|---|
| 2026-11-02 | 文档与本地源码对齐：100 ms 反馈阈值、状态编号、钳位、STOP 边界、ROS 实现差距 | 静态核对；Ubuntu 构建及台架测试待执行 |

