# 最新修改复查（第二轮）

对象：2026-11-17 最新工作区，已包含 USB 自测自动读取标定限位的修改，以及本轮控制逻辑调整。此报告替代上一轮复查结论用于描述当前状态。

结论：USB 和输出层自测通过，但完整固件链接失败，且遥控目标基准、夹爪力矩模式、重试授权仍有问题。本轮只检查和制作离线复现产物，未修改固件或正式测试源码、未烧录、未控制实机。

## 本次验证

| 项目 | 结果 |
| --- | --- |
| `python tools/selftest/run_selftest.py` | 全部通过；C 检查 0 失败，Python 协议交叉验证通过 |
| `python tools/selftest/run_control_selftest.py` | 7 个场景全部通过 |
| `python tools/selftest/full_arm_build.py` | 107 个编译单元通过；链接出现 4 个未定义符号错误 |
| 当前模式/输出函数主机复现 | 确认遥控首拍跳零、夹爪模式无力矩、故障自动请求重试、故障检测前已有带增益指令发出 |

USB 与输出层自测不构建真实 `Arm_L_task.c` 控制状态机，故它们通过与完整固件链接失败、模式逻辑有问题并不矛盾。

## 当前问题

### 1. [P1] 注释预设函数后没有清除调用，导致完整固件无法链接

位置：`APP/Arm_L_task.c:360`、`:363`、`:366`、`:369`；被注释的定义在 `:827`、`:883`、`:934`、`:991`。

主循环仍调用 TAKE_WATER_1、PASS_WATER_1、ARM_RESET_1、TAKE_Grip_1，但四个函数实现已全部注释。链接器实际输出四条 L6218E Undefined symbol。

建议：若决定取消这些动作，应同步修改模式选择和分派，将对应拨杆组合明确映射到保留的模式；若仍需要这些动作，应恢复函数并重新标定参数。不要仅添加空函数掩盖链接错误，否则切换到这些模式可能继续沿用旧的目标和增益。

### 2. [P1] 遥控位置基准只改了入口，输出仍用旧基准，首拍直接跳零

位置：`APP/Arm_L_task.c:341`、`:1268`，其余关节有相同结构。

当前进入遥控时：`RC_Sum_Value = position - Initial_Total_position`。

当前输出仍为：`send_set = zero_position + RC_Sum_Value`。

例如无力模式下 J2 在 -1.5 rad，Initial_Total_position 也为 -1.5。进入遥控后累加器为 0，但输出用 zero_position=0 相加，于是目标变为 0。使用当前真实模式入口和遥控函数复现，首拍 J2 目标为 0.000000，相对实测跳变 1.5 rad，约 85.94°。

建议：入口、目标计算、累加器限位及外部偏置必须使用同一基准。若选 Initial_Total_position，则最终输出也应使用这个基准，并检查夹爪限位仍减 grip_zero_position 等遗漏；不能只更改入口赋值。

### 3. [P1] DT7 接收又回到原始配置，完整帧仍无法解析

位置：`BSP/Src/bsp_uart.c:77`、`:97`、`:100`、`:131`、`:151`；`Components/Device/Inc/Remote_Control.h:31`。

本轮去掉了 +1，避免了上一轮“17 字节误算为 18”的问题，但没有建立正确的完整帧接收路径：RxXferSize=18、NDTR=18、双缓冲开启；完整 18 字节传输后 NDTR 自动重装为 18，当前 HAL 的 IDLE 条件要求 `0 < NDTR < RxXferSize`，因而不会调用接收回调。部分帧的回调 Size 为 1～17，又不满足 Size==18 的解析条件。

现有直接寄存器初始化仍没有注册并启用 DMA 完成处理路径。注释“Size < DataLength 时才处理”与下方实际 Size==18 判断也不一致。

建议：实际扩容接收缓冲并统一 NDTR/RxXferSize/长度判断，或完整实现双缓冲 DMA 完成事件。仅修改 RxXferSize 或注释无法解决。

### 4. [P1] 重试入口虽然可达，但故障本身会自动授权重试

位置：`APP/Arm_L_task.c:325`、`:333`；`APP/Moter_task.c:271`。

控制任务先在 bus_inhibit 时强制 ArmMode=ARM_POWERLESS，随后将这个变化视为无力模式上升沿并请求重试。没有检查是否为操作者主动切换遥控拨杆。

使用当前主循环复现：保持遥控模式选择不变，仅在第二拍置 bus_inhibit，就调用了 1 次 Arm_Output_RequestRetry。总线恢复后，如果请求被接受，将清除 bus_inhibit 并重新握手；原遥控/预设选择可继续生效，没有经过明确的人工重新授权。

建议：将“故障强制无力”与“真实遥控开关进入无力”分开，恢复授权依据真实输入或独立显式确认。

### 5. [P2] 重试请求早于无力快照发布，存在被提前消费而丢失的竞态

位置：`APP/Arm_L_task.c:335`；`APP/Moter_task.c:273`。

新增调用发生在 Arm_Powerless 和 Arm_PrepareAndPublish 之前。输出任务优先级更高，如果在这个窗口抢占，会读到上一拍带 Kp/Kd 的快照。输出层先清 retry_requested，再校验快照是否全零；校验失败会丢失这次请求。控制任务随后发布无力快照，但上一模式已更新，不能自动补发。

主机探针记录到请求发出时，已发布快照的 J2 Kp 仍非零。实际丢失与否取决于 RTOS 抢占时刻，未在实机测量。

建议：先发布确定的无力快照，再请求重试，或者把请求与对应快照以一致方式提交。主循环末尾原来的重试分支仍保留，但由于 prev_ArmMode 已更新，仍无法作为补救。

### 6. [P2] 排除无力模式叠加力矩，也把正常夹爪力矩模式屏蔽了

位置：`APP/Arm_L_task.c:289`、`:799`。

GetArmMode 中，右上+左中会设置 grip_torque_mode=1，但该组合返回的 ArmMode 是 ARM_POWERLESS。新增条件要求 ArmMode!=ARM_POWERLESS 才叠加夹爪力矩，导致正常夹爪力矩开关也失效。

当前真实 GetArmMode 和发布函数复现：mode=5（ARM_POWERLESS），torque_mode=1，blend=1，最终输出夹爪力矩仍为 0。

建议：单独表达“遥控在线且明确请求夹爪力矩”和“失联/故障无力”，或者新增独立夹爪模式。遥控失联必须立即归零，但正常力矩模式应保持可达。

### 7. [P2] 循环中才处理本拍故障，无法撤回已发给前面关节的控制帧

位置：`APP/Moter_task.c:312`、`:318`、`:341`。

新增分支能把发现故障之后的关节指令清零，但遍历顺序在轮转，排在故障关节之前的正常关节可能已经发出了带增益帧。

复现：使能完成后发布 Kp=20 的快照，使 J1 状态变为 0。本拍 J1 进入 EN_FAIL，但检测它之前已经发出 3 个非零 Kp 帧；下一拍才全部为零。

建议：第一轮遍历只更新所有关节的故障状态；第二步统一决定整臂输出是否受限；最后独立执行发送。不能依赖发送循环中途才清零来实现同拍整臂一致性。

## 本次已确认改善

- USB 限位自动同步仍正常，所有 USB 自测通过，无需再手动同步测试桩。
- 正常失联进入 ARM_POWERLESS 时，夹爪实际发布力矩已为 0，旧的 0.184 N·m 残余力矩问题在这一路径已消除；但如上所述，引入了正常力矩模式失效。
- J1～J6 上电自动覆盖零点的调用已注释，日常启动不再无条件发送这六个关节的存零命令。夹爪堵转标零仍保留。
- 重试条件不再永远不可达，但授权来源和请求时序仍需修正。
- 已有 EN_FAIL 的情况下，输出层会清零正常轴；新增故障的整拍一致性仍待修复。

## 证据与范围

链接日志：`tools/selftest/build/full_arm/link.log`。

隔离复现：`tools/selftest/build/audit/recheck_latest.py` 和 `recheck_latest.log`。从当前源码提取真实 GetArmMode、Arm_L_Task、遥控控制、无力和发布函数，硬件/RTOS 用假实现。由于当前生产工程四个预设符号缺失，仅在探针中对它们提供空实现以隔离验证其他路径；这不表示真实固件能够链接，也没有更改生产代码。

```text
RC entry from -1.5 rad: first J2 target=0.000000
Grip switch: mode=5 torque_mode=1 blend=1.00 output_torque=0.000000
Lost remote: output_torque=0.000000
Bus fault without switch change: retry_calls=1, request_before_zero=1
```

输出发送次序探针：`tools/selftest/build/audit/recheck_order.c`，链接本次复制的真实 Moter_task/DM_motor/bsp_can 与项目现有 HAL 假实现，记录每一帧而非只检查末帧：

```text
New J1 fault: state=3, frames already sent with nonzero kp=3
Next cycle: frames with nonzero kp=0
```

本次是离线审查。DT7 硬件收帧、电机真实力矩、运动和故障恢复尚未在实机验证。
