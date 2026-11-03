# APP / Modules 合入说明（S27_TEST_1）

本目录来自 `S27_ARM_H7` 的 `User/APP` 与 `Core/Inc|Src` 电机模块，已合入
`S27_TEST_1`（COD_H7_Template 架构）。原 APP 控制逻辑保持不变，新增
`APP/Compat` 作为新旧 BSP 的适配层。

## 1. 目录与任务

| 目录/文件 | 内容 |
| --- | --- |
| `APP/` | `Arm_L_task`、`Moter_task`、`DM_motor`、`referee_usart_task`（原样保留） |
| `Modules/` | `DJI_motor`、`HT_motor`、`LK_motor`（原样保留） |
| `APP/Compat/` | CAN / UART / ramp / 类型兼容层 |

FreeRTOS 中新增任务（`Core/Src/freertos.c`）：

- `Arm_L_Task`：左臂状态机、斜坡、模式切换
- `Moter_Task`：7 个 DM 电机 MIT 周期发送
- `referee_usart_Task`：原裁判/自定义控制器解析任务
- `APP_RC_Task`：DT7 优先、双板备用的遥控仲裁，维护 `RCData`

## 2. CAN 链路（保持 APP 原映射）

| 总线 | 用途 | 说明 |
| --- | --- | --- |
| FDCAN1 | 左臂 DM 关节 0x01~0x07 | 经典 CAN，`Arm_L_Init` 使能，`Moter_Task` 发 MIT，RX 回填 `joint_L_motor[7]` |
| FDCAN2 | 双板/自定义控制器 | `0x212`/`0x213` 先填 shadow buffer（DT7 离线时才进入 `RCData`），`0x208`~`0x210`/`0x207` 填 `CC_data` |
| FDCAN3 | 抬升 3508/2006 | 兼容层保留 ID，当前 APP 未主动使用 |

CAN 发送统一走 `APP/Compat/app_can.c` 的 `canx_send_data()`，内部复用
`bsp_can.c` 的 `FDCAN1/2/3_TxFrame`，并做短临界区保护。

## 3. 遥控链路

- DT7/SBUS（`remote_ctrl`）始终优先，`APP_RC_Task` 每 10 ms 把它复制到 `RCData`。
- 只有 DT7 离线（`rc_lost` / `online_cnt` 超时）时，才使用 FDCAN2 双板数据；
  双板数据先解到 shadow buffer，不会覆盖在线 DT7。
- 两者都离线时，`RCData` 被清零，机械臂进入 `ARM_POWERLESS` 安全态。
- `VT13` 串口链路未启用（原 APP 中相关代码本来也是注释状态）。

## 4. UART 说明

`referee_usart_Task` 依赖的 `uart1_rx_process_buf` 在原工程中只有声明、没有
定义；兼容层提供了 512 字节缓冲区。当前 USART1 仍按模板配置给图传
（921600），因此该缓冲区为 0，`Custom_RC[]` 保持 0，不影响机械臂主链路。
若以后接入 C 板自定义控制器，只需把对应 UART 的 DMA 数据写入该缓冲区。

## 5. 编译开关

在 `APP/Compat/can_bsp.h` 中：

```c
#define APP_ARM_ENABLE          1   /* 1: APP 接管 FDCAN1，模板 CAN 演示任务休眠 */
#define APP_RC_FALLBACK_ENABLE  1   /* 1: 启用 DT7 优先、双板备用的仲裁 */
```

模板 `CAN_Task` 中的 DM8009/FDCAN1 演示代码已用 `APP_ARM_ENABLE` 包住；
设为 0 可恢复模板演示（其中越界下标与 FDCAN1 标识符已顺手修正）。

## 6. Keil 工程

`MDK-ARM/S27.uvprojx` 已增加：

- Groups：`Application/APP`、`Application/Modules`、`Application/Compat`
- Include Paths：`..\APP`、`..\APP\Compat`、`..\Modules`

工程原来指定 ARM Compiler `V6.22`，本机安装的是 `V6.21`，已改为 `V6.21`；
否则 Keil 会直接报 “V6.22 is not available”。

## 7. 验证状态

- 本次 DT7 优先逻辑：`app_can.c` 由 ARMCLANG 单文件编译通过，
  `armlink --via project.lnp` 链接通过（UV4 当前在开发机启动崩溃，改用命令行工具链）。
- 产物：`MDK-ARM/COD_H7_Template/project.axf` / `project.hex`
- 说明：以上为编译/链接与静态链路验证；实机电机方向、零位、CAN 终端电阻
  和拨杆方向仍需上台架确认。
