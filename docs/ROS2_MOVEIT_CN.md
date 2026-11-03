本文基于 2026-11-01 的工程状态。

# 0. 结论摘要

推荐路线：在现有 `ros2_control` 链路中实现 `SystemInterface` 硬件插件，替换 `mock_components/GenericSystem`。MoveIt 的 `FollowJointTrajectory`、控制器名称和关节接口可基本保持，改动集中在硬件层，便于保持规划配置一致；独立桥接节点作为回退方案，适合先做协议联调但需自行实现 action、状态发布和更多时序逻辑。实机上线前必须确认零位/方向、Grip 单位、软限位，并显式确认 `ARM_USB_PC_APPLY_ENABLE=1`。

# 1. 总体架构

仿真工程的 `test_robot.ros2_control.xacro` 当前声明 `mock_components/GenericSystem`；`ros2_controllers.yaml` 以 100 Hz 启动 `arm_controller`、`gripper_controller` 和 `joint_state_broadcaster`；`test_robot.launch.xml` 先启动 `ros2_control_node`，再启动三个 spawner，最后 include `move_group`。实机应保持该依赖顺序。

```text
┌──────────────────────── Ubuntu ROS 2 进程边界 ────────────────────────┐
│ move_group + RViz                                                    │
│   └─ FollowJointTrajectory 目标（arm 6 DOF / gripper joint8）         │
│       ↓                                                              │
│ controller_manager (100 Hz) → joint_trajectory_controller             │
│       ↓ position command/state interfaces                             │
│ DM-MC02 SystemInterface 插件（推荐）                                  │
│       ├─ write/read：USB CDC，PC_CMD ≥20 ms / ARM_DATA 50 Hz           │
│       └─ joint_state_broadcaster → /joint_states                      │
└──────────────────────────────┬────────────────────────────────────────┘
                               │ USB CDC（帧头+payload+CRC16）
┌──────────────────────── STM32H723 固件进程边界 ───────────────────────┐
│ USB 接收/解析任务 → PC_CMD/PC_ENABLE/ESTOP → ArmData.send_set         │
│ Arm_L/Motor 任务 → 电机与反馈 → ARM_DATA(50 Hz)、HEARTBEAT(1 Hz)、     │
│                                    PC_STATE(1 Hz)、ACK                 │
└───────────────────────────────────────────────────────────────────────┘
```

| 路线 | 代码/配置落点（只描述，不执行） | MoveIt 配置变化 | 调试与实时性 | 结论 |
|---|---|---|---|---|
| `ros2_control` `SystemInterface` | 新建 hardware plugin；把 `test_robot.ros2_control.xacro` 的 `<plugin>` 改为该插件；实现 `read/write/activate/deactivate` | 控制器名、joint 名和 `FollowJointTrajectory` 可保持；仅替换 hardware plugin 参数 | 路径短、接口标准、100 Hz 控制循环易测；USB 抖动需在插件内隔离 | **推荐** |
| 独立桥接节点 | 新建 ROS 2 node，实现 `FollowJointTrajectory` action、USB 编解码、`/joint_states` 发布；MoveIt controllers 指向该 action | 需改 `moveit_controllers.yaml` 的 action 命名/命名空间；可不启动 ros2_control controller | 初期可独立抓包，故障定位直观；需自建轨迹仲裁、取消、超时和实时节拍 | 协议摸底或插件尚未稳定时的过渡方案 |

推荐插件只承诺 position command/state interface，保留现有 `arm_controller`/`gripper_controller`。插件内部使用非阻塞 USB I/O 和有界缓存；USB 线程/回调不得阻塞 `read()`/`write()`。待确认：实际 USB 设备节点、串口权限、固件当前分支是否已切换到 `arm_usb_comm.*` 协议。

# 2. 字段与单位映射表

帧内下标均从 0 开始，`ARM_DATA.joint[i]` 与 `PC_CMD.position[i]` 使用同一顺序。角度链路统一保留 `float`，插件边界做限幅和零位换算。

| 仿真关节 | 帧内下标 | 实机关节/ArmData | 仿真单位与 URDF 限位 | 固件当前限位（`Arm_L_task.h`） | 零位偏差初值 |
|---|---:|---|---|---|---:|
| `joint1` | 0 | J1 | rad，[-2.09, 2.09] | rad，[-2.96055, 2.81695] | 0.0 rad |
| `joint2` | 1 | J2 | rad，[0, 1.89] | rad，[0, 1.77324] | 0.0 rad |
| `joint3` | 2 | J3 | rad，[0, 3.0543] | rad，[0, 2.39008] | 0.1 rad |
| `joint4` | 3 | J4 | rad，[-2.09, 2.09] | rad，[-4.3, 4.3] | 0.0 rad |
| `joint5` | 4 | J5 | rad，[-2.09, 2.09] | rad，[-1.29563, 1.23497] | 0.0 rad |
| `joint6` | 5 | J6 | rad，[-2.09, 2.09] | rad，[-2.25161, 2.22338] | 0.0 rad |
| `joint8` | 6 | Grip | prismatic，m，[-0.000523, 0.009530] | 工程宏为 [0, 0.7]；语义待确认 | 0.0（需标定） |

方向、编码器正方向和零位不能仅凭文件名确定。插件/桥接节点应配置 `sign[i]`、`offset[i]`，并使用 `q_ros = sign[i] * (q_mcu - offset[i])`、`q_mcu = sign[i] * q_ros + offset[i]`；上线前用单关节小步验证 `sign`，不要直接把仿真上限当作实机上限。固件限位优先，ROS `joint_limits.yaml` 先收紧到实测安全值。

**Grip 单独处理。** 仿真 `joint8` 是米制滑块，实机 `Grip` 可能是 0..1 归一化量，也可能沿用弧度语义；待确认：`ArmData.Grip.position/send_set` 的最终单位及开合方向。若确认实机为归一化值，可定义：

```text
u = clamp((q_sim - q_close_m) / (q_open_m - q_close_m), 0, 1)
q_cmd = clamp(u, 0, 1)                 # 固件定义为 0..1 时
# 若固件量程为 [0, 0.7]，q_cmd = 0.7 * u
```

其中 `q_close_m`、`q_open_m` 必须由实机标定；若方向相反交换两者。修改落点：插件的映射配置/转换函数；若走桥接节点，改桥接节点的 `joint8 ↔ Grip` 转换，不改 MoveIt 规划组名称。`joint9..joint16` 为 mimic/passive 结构，不下发为独立电机。

# 3. 轨迹下发链路

`FollowJointTrajectory` 的轨迹点常见为 10 ms 间隔，USB 协议建议以 20 ms（50 Hz）为最小发送周期。不要把多个历史点突发补发；用单调主机时钟建立发送栅格，在每个 `t0+n*20 ms` 处对相邻轨迹点做线性插值（或使用已验证的三次插值），最后一个点发送一次并保持，直到 action 成功或取消。

| 项目 | 建议 |
|---|---|
| `joint_count` | 始终填 7；arm 轨迹的 Grip 位置取当前反馈，gripper action 只更新下标 6 |
| `position[7]` | J1..J6 为 rad；Grip 按第 2 节转换；每帧先做 ROS 限位、固件限位和最大步长检查 |
| `command_ms` | `steady_clock`/`time.monotonic()` 的毫秒低 32 位；每帧递增，按无符号回绕比较，不使用墙上时间 |
| `valid_time_ms` | 初始 100 ms（固件允许范围 10..1000 ms）；发送周期 20 ms，连续丢帧仍能在约 100 ms 内失效；实测后可收紧到 60 ms |
| `flags` | 正常轨迹帧带 `ENABLE`；撤销/停机发 `ESTOP`，随后发 `PC_ENABLE(0)`；不要同时置 ENABLE 与 ESTOP |
| ACK | 按 `acked_id/result/acked_seq` 关联；`BAD_LENGTH`、`BAD_VALUE` 立即终止 action 并进入安全停机 |

固件命令过期后，`command_valid` 应变为 0，后续目标不得继续驱动；预期由应用层保持当前安全目标或停在当前位置，具体停机动作待确认：用 `PC_STATE` 的 `command_age_ms`、`command_valid` 与 `ARM_DATA.send_set` 现场核对。上位机掉线、USB 写失败、ACK 连续丢失或轨迹调度延迟超过 `valid_time_ms` 时，桥接层立即停止发送新目标，尽力发送一次 `PC_CMD(ESTOP)` 和至少两次 `PC_ENABLE(0)`；物理急停仍是最终措施。

```text
# 伪代码：20 ms 轨迹采样与超时处理
next = monotonic_ms()
while action_active:
    now = monotonic_ms()
    if now < next: sleep(next - now); continue
    q = interpolate(traj, elapsed=now - start)
    q = clamp_and_map(q)                 # 含 Grip 转换
    send_pc_cmd(q, flags=ENABLE, command_ms=now & 0xffffffff,
                valid_time_ms=100, joint_count=7)
    next += 20
    if next < now: next = now + 20        # 不补发突发帧
if cancel_or_io_error:
    send_pc_cmd(last_safe_q, flags=ESTOP, command_ms=monotonic_ms(), valid_time_ms=100)
    send_pc_enable(0)
```

需要改动时的明确落点（只写方案）：

| 文件 | 改成什么样 |
|---|---|
| `test_robot_moveit_config/config/test_robot.ros2_control.xacro` | 将 `mock_components/GenericSystem` 替换为新 `SystemInterface` 插件，并增加 USB 设备、映射表、周期参数 |
| 新建 `ros2_control` hardware plugin 的 `.hpp/.cpp` | 实现 `on_init/read/write/on_activate/on_deactivate`；`write()` 只发送采样后的 PC_CMD，`read()` 消费 ARM_DATA |
| 若采用桥接路线：新建 bridge node | 实现 `FollowJointTrajectory` action、20 ms 采样器、ACK/超时/ESTOP和 `/joint_states` |
| `test_robot_moveit_config/config/moveit_controllers.yaml`（仅桥接路线） | 将控制器 action 名称/命名空间改为 bridge 提供的 action；插件路线保持现状 |
| 固件 `APP/arm_usb_comm.h/c`（仅需启用驱动时） | 在发布版本配置中显式把 `ARM_USB_PC_APPLY_ENABLE` 设为 1，并保留超时/限幅/ESTOP检查；当前文件在工程主目录的对应实现待确认 |

# 4. 状态回传链路

`ARM_DATA` 每 20 ms 到达一次，解码 7 个 joint 的 `position`、`send_set` 等字段，写入 hardware plugin 的 state interfaces；`joint_state_broadcaster` 再发布 `/joint_states`。`joint1..joint6` 直接按 rad 发布，`joint8` 先做 Grip 逆映射后发布为仿真要求的米制值。若 `position` 非有限值、`joint_count != 7`、CRC 错或帧长度不是 198 字节，整帧丢弃并递增错误计数。

**时间戳。** `/joint_states.header.stamp` 使用主机收到并完成 CRC 校验的 ROS 时间（`node->now()`），保证与 TF、MoveIt 使用同一时钟；保留帧内 `tick_ms` 作为诊断字段并用相邻 tick 计算设备侧周期/延迟。只有在已建立 MCU tick 到 ROS clock 的偏移估计后，才可改用换算后的 tick 时间。时间回绕用无符号差值处理。

**丢帧策略。** 控制反馈不做插值：丢帧时保留最后一个有效位置，同时发布 `state_age_ms`/诊断；超过 100 ms（或待确认的安全阈值）将关节状态标为 stale，插件停止接受新轨迹。可在 RViz 可视化层对不超过一个周期的样本做线性插值，但不能把插值值写回 hardware state，也不能用于安全判定。由于 ARM_DATA 没有独立 velocity 字段，`JointState.velocity` 默认留空；若确需速度，使用 tick 差分并加限幅/低通，待验证后再开放。

| 来源 | 频率 | 处理 | ROS 暴露 |
|---|---:|---|---|
| ARM_DATA（198 B） | 50 Hz | CRC、长度、joint_count、有限值、Grip 逆映射 | `/joint_states`；帧率/年龄/seq gap 诊断 |
| HEARTBEAT（8 B） | 1 Hz | 校验 `status/arm_mode/uptime_s` | `diagnostic_msgs/DiagnosticArray` |
| PC_STATE（16 B） | 1 Hz | 读取 `enabled/command_valid/last_result/command_age_ms/rx_frames/rx_errors` | 诊断键值与告警等级 |
| ACK（4 B） | 按命令 | 关联 `acked_id + acked_seq` | action result、错误日志 |

`PC_STATE.rx_errors` 应按累计计数计算窗口增量：0 为 OK；增量非零或 ARM_DATA 帧率低于 45 Hz 为 WARN；持续 1 s 无有效 ARM_DATA、`command_valid=0`、或错误持续增长为 ERROR。诊断中同时发布最近 `last_result`、`command_age_ms`、最近 tick、CRC/重同步计数，避免只显示“串口断开”。

```text
# 伪代码：ARM_DATA -> joint_states
for frame in usb_frames:
    if frame.id != ARM_DATA or len(frame) != 198 or not crc_ok(frame):
        rx_errors += 1; continue
    host_stamp = ros_now()
    q = decode_float_positions(frame.payload)
    q[6] = grip_to_joint8(q[6])
    if any_nonfinite(q): rx_errors += 1; continue
    state_interfaces.set(q)
    publish_joint_states(names, q, stamp=host_stamp)
    age_ms = host_mono_ms() - last_valid_host_ms
```

需要改动时：插件的 `read()` 负责解码和 state interface；若需独立诊断，新增 `dm_mc02_diagnostics` 节点订阅内部统计/PC_STATE 并发布 `DiagnosticArray`。桥接路线则在 bridge node 内完成同样逻辑；不要修改 MoveIt 的 joint 名称。

# 5. 安全设计与联锁

| 联锁 | 触发条件 | 动作 | 恢复要求 |
|---|---|---|---|
| `APPLY_ENABLE` | 固件编译宏为 0/1 | 为 0 时允许遥测、解析、ACK，但不得驱动；上线前逐项确认改为 1 的固件版本 | 记录固件版本和宏值，重新执行只读遥测 |
| PC 使能 | `PC_ENABLE(1)` 且反馈有效 | 只允许在所有关节状态新鲜、软限位参数已加载时进入 armed | `PC_ENABLE(0)`、ESTOP 或 USB 重连后重新握手 |
| 命令过期 | `command_age_ms > valid_time_ms` 或 `command_valid=0` | 固件停止接受新目标；主机停止发送并发 ESTOP/撤防 | 清除故障，重新发送当前反馈位置，再使能 |
| ESTOP | 物理急停、软件急停、通信异常 | 主机发 `PC_CMD(flags=ESTOP)`；随后 `PC_ENABLE(0)`；物理回路切断驱动 | 只能人工确认现场安全后复位急停 |
| 软限位 | 目标超出 ROS/固件限位 | 主机拒绝该帧并结束 action；不静默截断跨越大步 | 修正轨迹或映射，重新规划 |
| 速度/步长 | `abs(q[k]-q[k-1])/dt` 超过实测上限 | 降采样频率不变，缩放轨迹或拒绝发送；Grip 单独按 m/s/归一化量检查 | 以更低倍率重试 |
| 反馈丢失 | ARM_DATA 缺帧、CRC 错、年龄超阈值 | plugin 置 stale、停止新命令；主机发送 ESTOP/撤防 | 恢复稳定 50 Hz 后从只读阶段重来 |

**`ARM_USB_PC_APPLY_ENABLE` 是上线闸门。** 题设确认默认值为 0；收到 `PC_CMD` 时固件只解析、保存并 ACK。上线检查表必须包含：读取构建配置/版本记录；确认该宏确为 1；确认 `PC_STATE.enabled=0` 后再做一次无动作 PC_CMD；观察 `ARM_DATA.send_set` 不变；最后才允许 `PC_ENABLE(1)`。若工程实际使用 `APP/Arm_USB_task.c` 而不是备份目录的 `arm_usb_comm.c`，应在对应实现中完成同等闸门，文件和分支待确认。

**上电顺序：**

1. 机械臂固定、工作区清空、物理急停保持按下，限位/方向标签可见。
2. 先启动固件和 USB CDC，再连接主机；只读接收 ARM_DATA/HEARTBEAT/PC_STATE，确认 50/1/1 Hz 和 CRC 正常。
3. ROS 只启动 `robot_state_publisher`、hardware plugin 和状态广播；controller 在未使能前不得发目标。
4. 逐关节核对当前反馈、零位、限位、方向；设置低速度/低加速度缩放。
5. 松开物理急停后发送 `PC_ENABLE(1)`，仍从单关节 ±小角度开始；任何异常立即按物理急停。

物理急停按钮必须独立于 ROS/USB，能切断驱动使能或触发硬件安全输入；软件 ESTOP 只作为补充。恢复不能自动重启轨迹，必须重新读取状态、重新规划并由操作者确认。

需要改动时：在 hardware plugin/bridge 中实现限位、步长、年龄 watchdog 和 ESTOP 状态机；在 `APP/arm_usb_comm.h/c`（或当前 `Arm_USB_task.*` 对应实现）保留固件侧超时、限位和 `APPLY_ENABLE` 闸门；在 `ros2_controllers.yaml`/`controller_manager.yaml` 将控制器启动后置或保持 inactive，具体参数名待确认。

# 6. 分阶段验证计划

每阶段只在上一阶段通过、急停可触达、记录文件已归档后进入。任何失败都回到“只读遥测”，不跨阶段带故障运行。

| 阶段 | 进入条件 | 操作与观察指标 | 失败判据 |
|---|---|---|---|
| 1. 只读遥测 | 固件启动、USB 可枚举；`APPLY_ENABLE` 状态已记录但不使能 | 运行 monitor；连续 ≥5 min 收 ARM_DATA 50±2 Hz、HEARTBEAT 1 Hz、PC_STATE 1 Hz；CRC/重同步/`rx_errors`=0；7 个反馈有限且年龄稳定 | 任一帧长/CRC 错误持续增长、频率低于 45 Hz、joint 顺序/单位不一致、USB 重连后未恢复 |
| 2. 空载点动（单关节 ±小角度） | 阶段 1 通过；机械空载、物理急停有效；`APPLY_ENABLE=1` 已核验 | 仅 J1..J6，每次 ±0.02 rad 或更小；20 ms PC_CMD，`valid_time_ms=100`；观察方向、零位、停止延迟、`command_valid` | 方向反、越过软限位、目标/反馈发散、停止后仍运动、年龄/错误异常 |
| 3. 单关节轨迹 | 阶段 2 通过且该关节映射已标定 | 发送短轨迹（≤0.2 rad，低速）；检查 action result、ACK 序列、轨迹跟随误差和 20 ms 发送抖动 | ACK 非 OK、轨迹时间轴乱序、误差超预设阈值、超时未停或 action 取消无效 |
| 4. 六关节联动 | 六个单关节均通过；碰撞区清空；MoveIt 规划组 `arm` 仅含 joint1..joint6 | 低倍率规划到已验证姿态；记录每关节位置/速度、规划与实测时间、USB 帧率；确认无自碰/越限 | 任一关节超限、联动误差明显放大、控制器 abort、通信错误增长或急停链路不可靠 |
| 5. 夹爪 | Grip 单位/方向已由标定确认；`joint8` 映射测试通过 | 先开/合小行程，再执行 gripper action；记录开合端点、重复性、速度（仿真 0.02 m/s 仅作参考） | 0/1 与机械端点相反、卡滞、端点冲击、单位误用导致大步运动 |
| 6. 完整抓取 | 1–5 全通过；物体、工装和恢复姿态已定义；现场负责人确认 | 低速执行：到预抓取位→闭合→抬升→放置→打开；全程记录 ARM_DATA、PC_STATE、ACK、ROS action 日志 | 物体滑落、碰撞/过载、任一 watchdog/ESTOP 误触发且无法恢复、日志缺失 |

建议每阶段保存：主机单调时间、帧 seq/tick、`rx_errors`、`command_age_ms`、目标与反馈、action 状态和人工观察。阈值（跟随误差、停止距离、允许延迟）在无硬件前标为“待确认”，由空载测量后写入测试记录，不在配置中擅自放宽。

# 7. 风险清单与回滚

| 风险 | 现象 | 原因 | 处置 |
|---|---|---|---|
| 协议版本/帧布局不一致 | 全部 ACK `BAD_LENGTH` 或 CRC 错 | 主目录 `arm_usb_protocol.*` 与备份 `arm_usb_comm.*` 不是同一版；头长度、CRC 或 payload 不同 | 固定使用题设 12 字节头/0x8408 CRC/198 字节 ARM_DATA；锁定固件与上位机版本，先做离线帧互解 |
| `APPLY_ENABLE` 未打开 | ACK=OK、PC_STATE 正常但电机不动 | 默认宏为 0，只解析和存储 | 记录构建宏；需要上线时只在受控固件分支改为 1，重新执行阶段 1–2 |
| 零位/方向错误 | 点动方向相反、回零偏差或规划姿态镜像 | 编码器正方向、`offset` 未标定；J3 初始偏差等 | 立即 ESTOP；在映射配置中改 `sign/offset`，不改 MoveIt 名称；重新单关节验证 |
| Grip 单位误用 | 夹爪大步冲击或完全不动 | `joint8` 米制与 Grip 0..1/0..0.7 语义混用 | 禁止自动上线；确认单位后只改转换函数并做端点标定 |
| 轨迹发送抖动/突发 | action abort、跟随误差周期性增大 | 10 ms 点未降采样、USB 写阻塞或落后补发 | 使用 20 ms 单调栅格、非阻塞 I/O；落后时跳到当前栅格，不补发 |
| 命令过期未停 | `command_valid=0` 但目标继续变化 | 主机忽略 `valid_time_ms`、固件应用层接线错误 | 物理急停；检查 `ARM_USB_PC_APPLY_ENABLE`、超时分支和 `ArmData.send_set`；未验证前不得联动 |
| 遥测丢帧/状态陈旧 | `/joint_states` 卡住或 MoveIt 误判当前位置 | USB 拥塞、CRC 错、读取线程阻塞 | plugin 标记 stale、停止新轨迹；查看 `rx_errors`/seq gap，修复 I/O 后回到只读 |
| ROS 控制器启动竞态 | action 找不到、controller inactive 或状态全零 | controller_manager、spawner、MoveIt 启动顺序错误 | 按 `ros2_control_node → spawner → move_group` 顺序；保持控制器 inactive 直到遥测通过 |
| 实机限位与 URDF 不同 | 规划可达但固件拒绝/夹死 | 仿真 URDF 上限比固件宽或零位不同 | 使用实测更小限位覆盖 `joint_limits.yaml` 和插件限位；重新规划 |

**回滚步骤：**

1. 主机立即发送 `PC_CMD(ESTOP)`、`PC_ENABLE(0)`，必要时按物理急停；保存最后 10 s 日志。
2. 停止 MoveIt action 与 controller spawner，只保留 monitor；确认 `command_valid=0`、ARM_DATA 仍可读。
3. 将 `test_robot.ros2_control.xacro` 的 hardware plugin 恢复为 `mock_components/GenericSystem`（插件路线），或把 `moveit_controllers.yaml` 恢复为原 `arm_controller/gripper_controller` action（桥接路线）；这些是待实施的文件修改，不在本次文档任务中执行。
4. 固件侧恢复 `ARM_USB_PC_APPLY_ENABLE=0` 的已知安全构建，重新执行只读遥测；若协议版本不明，停止实机下发。
5. 只有根因、修复和复测记录齐全，才允许从阶段 1 重新进入。

# 8. 附录：协议帧速查表、参考代码片段

## 8.1 帧速查表

统一格式：`0xA5` + 12 字节 header + payload + CRC16。header 字段均 little-endian：`id(u8) | ver(u8) | rsvd(u8) | len(u16) | seq(u16) | tick_ms(u32)`；CRC16 小端存放，覆盖 `SOF..payload`，初值 `0xFFFF`，LSB-first 多项式 `0x8408`，表首项 `0x0000/0x1189`。

| ID | 方向 | payload | 整帧长度 | 关键字段 |
|---:|---|---:|---:|---|
| `0x01 ARM_DATA` | MCU→PC，50 Hz | 184 B | 198 B | `arm_mode,u8 joint_count,u16 rsvd`；7×`position/send_set/rc_sum/key_ctrl/kp/kd`（float）；`total_angle,pitch,roll` |
| `0x02 HEARTBEAT` | MCU→PC，1 Hz | 8 B | 22 B | `status,arm_mode,rsvd,uptime_s` |
| `0x10 PC_CMD` | PC→MCU，≥20 ms 周期 | 36 B | 50 B | `command_ms(u32),valid_time_ms(u16),flags,u8 joint_count,position[7](float)` |
| `0x11 PC_ENABLE` | PC→MCU | 1 B | 15 B | `0` 撤防，`1` 使能 |
| `0x20 ACK` | MCU→PC | 4 B | 18 B | `acked_id,result,acked_seq(u16)`；result：0 OK、1 BAD_LENGTH、2 BAD_VALUE |
| `0x21 PC_STATE` | MCU→PC，1 Hz | 16 B | 30 B | `enabled,command_valid,pc_flags,last_result,command_age_ms,rx_frames,rx_errors` |

PC_CMD flags：`0x01 ENABLE`、`0x02 DISABLE`、`0x04 ESTOP`。ACK 必须按 `acked_id + acked_seq` 关联；未知帧、CRC 错或不完整帧不应触发运动。

## 8.2 CRC 与发送伪代码

```text
crc16(data):
    crc = 0xffff
    for b in data:
        crc ^= b
        for i in 0..7:
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return crc & 0xffff

send(id, payload):
    header = le(0xa5, id, ver, 0, len(payload), seq++, tick_ms())
    body = header + payload
    return body + le(crc16(body))
```

## 8.3 `SystemInterface` 伪代码

```text
on_activate():
    open_usb(); wait_arm_data(); send_pc_enable(0)
read():
    while frame_available(): decode_and_update_state(frame)
    if state_age_ms > state_timeout: mark_stale(); return ERROR
write():
    q = command_interfaces[:]          # 7 个 position
    q = clamp_and_map(q)
    if !armed: return OK
    if now_ms - last_tx_ms >= 20:
        send_pc_cmd(q, ENABLE, now_ms, 100)
on_deactivate():
    send_pc_cmd(last_safe_q, ESTOP, now_ms, 100)
    send_pc_enable(0); close_usb()
```

上述片段仅为接口行为示意；实际线程、锁、USB API、错误码和参数名待按 ROS 2 发行版与工程构建系统确认。当前 Windows 副本中已读到的固件主目录文件名为 `APP/arm_usb_protocol.*`、`APP/Arm_USB_task.*`，题目指定的 `APP/arm_usb_comm.*`、`ARM_USB_COMM_CN.md` 在 `_backup_arm_usb_comm_20261031_152120` 下；合入前需确认哪个分支才是实机固件基线。
