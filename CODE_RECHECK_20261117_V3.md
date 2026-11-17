# 最新修改复查（第三轮）

检查对象：2026-11-17 12:42～12:44 修改后的固件源码。本次仅审查、离线构建及复现；未修改生产源码、未烧录或控制硬件。本报告用于描述当前状态，前两轮报告中的部分缺陷已修复。

## 结论及验证

完整工程已恢复编译链接，USB 和控制自测全部通过。上一轮的 J2 首拍跳零、夹爪力矩模式失效、故障发送顺序、四个预设函数缺失造成的链接错误已解决。但 UART 接收和故障重试仍有问题，夹爪位置模式还遗漏了一处基准同步。

| 验证项 | 本次结果 |
| --- | --- |
| 全量 ARM 编译链接 | 107 个编译单元通过，0 编译警告；链接成功，1 条原有 L6314W 警告 |
| USB 自测 | C 检查 0 失败，Python 协议/状态/诊断交叉验证通过 |
| 控制自测 | encoding / motion / enable / retry / fifo / bus / queue 共 7 项通过 |
| J2 模式切换探针 | 实测 -1.5 rad，摇杆居中切入遥控，首拍目标保持 -1.5 rad |
| 夹爪力矩模式探针 | 右上+左中且 blend=1，输出 0.2 N·m；遥控失联后输出 0 |
| 新增单轴故障探针 | 本拍与下一拍发送的非零 Kp 帧数均为 0 |

构建大小：Code=140640，RO-data=2664，RW-data=428，ZI-data=118708 字节。L6314W 是未使用 `.AXI_SRAM` 段的既有警告。

## 仍需处理的问题

### 1. [P1] UART5 完整帧解析了错误缓冲，残帧也会被接受

位置：`BSP/Src/bsp_uart.c:77`、`:97`、`:119`、`:131`、`:151`、`:154`；长度计算参见 `Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c:2388` 及 `:2425`。

当前 RxXferSize=19，真实 DMA 单缓冲大小与 NDTR 仍为 18，双缓冲开启，接收入口改为接受 `0 < Size < 18`。

当 18 字节正常帧写满 M0 后，硬件已自动将 CT 切到 M1，并将 NDTR 重装为 18。HAL 的 IDLE 回调长度为 19-18=1。新的长度条件接受 Size=1，但处理函数根据 CT=1 解析的是 M1，刚刚完成的帧实际在 M0。它还把 CT 手动切回 M0，因此正常连续完整帧可能不断覆写 M0，同时解析未更新的 M1。

此外，只有 10 字节到达时，NDTR=8，HAL 报 Size=11，也会通过新的条件。SBUS_TO_RC 始终读满 18 字节，并直接刷新 online_cnt/rc_lost，可能将残帧与旧字节拼接成遥控数据。

使用当前真实 USER_USART5_RxHandler 配合模拟 DMA 状态复现：

```text
18-byte M0 complete: Size=1, parser_calls=1, parsed_buffer=M1 (expected M0)
10-byte partial frame: Size=11, parser_calls=1 (expected 0)
```

建议：区分协议帧长和真实 DMA 容量。若使用 IDLE 解析固定 18 字节帧，可以将真实 M0/M1 缓冲容量扩到大于 18，并让 NDTR、RxXferSize、重装计数统一使用该容量，回调按实际 18 字节检查。另一种方案是完整实现 DMA 双缓冲完成事件，准确选择已完成缓冲。不能仅调整 RxXferSize 和 Size 接受条件。

同一初始化函数也用于 USART1，虚增 RxXferSize 的做法同样导致其长度报告多 1，需要一并校准。

### 2. [P1] 总线故障强制进入无力，仍被误当作人工重试授权

位置：`APP/Arm_L_task.c:324`、`:333`。

bus_inhibit 先把 ArmMode 强制设为 ARM_POWERLESS，随后模式边沿分支直接调用 Arm_Output_RequestRetry。它没有区分真实遥控拨杆选择变化和故障强制切换。

当前主循环探针再次确认：保持遥控模式选择不变，仅在第二拍触发 bus_inhibit，重试调用次数仍为 1。总线恢复后若请求被接受，输出层清 bus_inhibit 并重新握手，原遥控选择可能恢复生效；这与接口声明的 explicit POWERLESS transition only 不一致。

建议：单独跟踪真实遥控模式选择，要求有效遥控输入上的明确无力操作来重新授权。不能只观察已经被故障逻辑改写的 ArmData.ArmMode。

### 3. [P2] 重试请求仍在无力快照发布之前，可能被高优先级任务提前消费

位置：`APP/Arm_L_task.c:334`；`APP/Moter_task.c:273`。

请求发出时还没有运行本拍 Arm_Powerless 和 Arm_PrepareAndPublish，已发布快照仍可能有非零 Kp/Kd。输出任务优先级更高，若此时抢占，它先清 retry_requested，再用旧快照做全零校验；校验失败便丢弃请求。随后控制任务发布无力快照，也不会再次产生模式边沿。

探针确认请求发出时旧快照的 J2 Kp 仍非零；实际请求丢失取决于抢占时刻，并未在实机测量。末尾保留的第二个重试条件因 prev_ArmMode 已更新而仍不可达，无法补救。

建议：先完成并发布无力快照，再提交与本次人工模式转换对应的重试请求，或将二者一致提交。

### 4. [P2] 夹爪目标改用 Initial_Total_position，但累加器限位仍减零点，导致不能正常张开

位置：`APP/Arm_L_task.c:1246`、`:1267`。

J1～J6 的主要目标基准已统一，夹爪最终目标也改成 `Initial_Total_position + RC_Sum_Value + Key_Control_Value`。但夹爪累加器仍被限制到 `Grip_Min/Max - grip_zero_position - Key_Control_Value`。

以夹爪实际位置 -6 rad 为例：无力模式把 Initial_Total_position 设为 -6；切入遥控时 RC_Sum_Value 为 0。正向摇杆希望张开，累加量刚变成正值，就被旧上限 `Grip_Max_position - grip_zero_position = 0` 钳回 0，因此最终目标始终等于 -6，无法向 0 张开。

当前真实模式入口和遥控函数复现：

```text
Grip opening from -6 rad, 100 positive-stick cycles: target=-6.000000 sum=0.000000
```

建议：夹爪累加器上下界也应减去夹爪自己的 Initial_Total_position 及实际叠加偏置，使它与最终目标公式一致。

## 已解决与功能状态说明

- 四个预设函数不再被调用，相关分支执行 Arm_Powerless，链接错误已消除。含义是预设动作目前被禁用，并非原预设位姿已经重新标定；遥测的模式值仍可能显示原预设模式。
- J2 实测 -1.5 rad 切入遥控的首拍目标保持 -1.5 rad，上一轮 1.5 rad 跳零已消除。
- 夹爪力矩模式恢复输出；遥控失联时不再保留旧混合力矩。
- 输出层已先检查全部反馈，再统一清零、最后发送，本轮检测到单轴故障时未发送带 Kp 指令。
- J1～J6 上电无条件存零调用仍停用，夹爪堵转标零仍保留。
- USB 测试桩仍自动读取 Arm_L_task.h 标定限位，自测保持通过。

## 复现与验证范围

- 构建日志：`tools/selftest/build/full_arm/link.log`。
- 当前控制模式探针：`tools/selftest/build/audit/recheck_latest.py`，结果为同目录 `recheck_latest.log`。使用提取的真实模式、遥控和发布函数，硬件及 RTOS 以假实现替代。
- UART 探针：`tools/selftest/build/audit/recheck_uart.py`，结果为 `recheck_uart.log`。使用当前真实 UART5 回调，按双缓冲硬件规则模拟完整帧和残帧场景；不代表已进行实机串口测试。
- 故障发送顺序探针：`tools/selftest/build/audit/recheck_order.c`，链接当前输出/DM/BSP 源码与项目 HAL 假实现，确认首次故障周期的非零 Kp 帧数为 0。

USB/输出层自测没有完整覆盖真实控制状态机与 UART DMA，因此通过不能替代本报告列出的链路检查。建议优先修正 UART 接收和重试授权，再补齐夹爪限位基准与重试时序。
