# S27_TEST_2 整改记录

## 2025 年度整改清单

### 1. ✅ 已完成：禁用 Control_Task 和 INS_Task
**文件**：`Core/Src/freertos.c`
- 注释行 61-66：删除缓冲区定义
- 注释行 98-99：删除函数声明  
- 注释行 149-155：删除任务创建代码
- 注释行 196-232：禁用函数实现（__weak）

**效果**：
- 释放 4096 字节栈空间（2 × 2048）
- 减少 2 个任务调度开销
- 保留代码便于恢复

---

### 2. ✅ 已完成：清理 LED 实现
**删除文件**（已移至 `archive/`）：
- `BSP/Src/bsp_led_dma.c` — DMA 实现（DTCM 内存配置错误）
- `BSP/Src/bsp_led_dma_standalone.c` — 独立 DMA 实现
- `BSP/led_diagnostic_example.c` — 诊断示例

**保留**：
- `BSP/Src/bsp_led.c` — 标准轮询实现（当前编译单元）

**理由**：
- bsp_led_dma*.c 错误地将缓冲区放在 DTCM，STM32H7 DTCM 不支持 DMA1/DMA2 访问
- 当前产品采用轮询方案足以满足需求（20 ms 呼吸灯，低优先级任务）
- 若后续需要 DMA，在 scatter 文件更新后重新实现

---

### 3. ✅ 已完成：删除空模块
**删除文件**（已移至 `archive/`）：
- `APP/Arm_R_task.c` — 全部注释，无功能实现
- `APP/Arm_R_task.h` — 关联头文件

**工程更新**：
- `MDK-ARM/project.uvprojx` 行 423-427：移除编译单元引用

**理由**：
- 右臂功能未实现，空占位置
- 后续需要时从 archive 恢复并完整实现

---

### 4. ✅ 已完成：收紧 CAN 硬件过滤器
**文件**：`BSP/Src/bsp_can.c` 行 63-84

**修改内容**：
```c
/* FDCAN1：达妙关节 0x01..0x07 */
filter.FilterID1 = 0x000U;
filter.FilterID2 = 0x7F8U;      // 掩码允许 0x00x..0x07x

/* FDCAN2：双板控制 0x212, 0x213 */
filter.FilterID1 = 0x212U;
filter.FilterID2 = 0x7FEU;      // 精确掩码

/* FDCAN3：DJI 抬升电机 0x201..0x204 */
filter.FilterID1 = 0x200U;
filter.FilterID2 = 0x7F8U;      // 掩码允许 0x20x..0x20x
```

**效果**：
- 硬件层面拒绝不相关 ID，减少软件队列压力
- 32 帧队列满的风险大幅降低
- 新增诊断指标更准确反映实际流量

---

### 5. ✅ 已完成：规范 DMA 内存配置
**文件**：`MDK-ARM/COD_H7_Template/COD_H7_Template.sct`

**改进**：
- 添加明确的 `.AXI_SRAM_DMA` section 用于 DMA 缓冲区
- 添加注释说明 DTCM 不可用于 DMA1/DMA2
- 保留区域分隔，防止 DMA 和通用变量混杂

**建议后续**：
- 所有图传发送缓冲区需添加 `__attribute__((section(".AXI_SRAM_DMA")))` 标记
- Image_Transmission.h 中的发送结构体需统一标记
- 构建过程中可添加脚本验证关键 DMA 符号地址是否在允许范围内

---

## 当前状态

| 项 | 状态 | 备注 |
|---|------|------|
| INS_Task / Control_Task | ✅ 禁用 | 堆栈节省 4KB |
| LED 实现 | ✅ 统一为轮询 | bsp_led.c 保留，DMA 版本存档 |
| 空模块 Arm_R_task | ✅ 删除 | 已移至 archive/ |
| CAN 过滤器 | ✅ 收紧 | 三路总线精确过滤配置 |
| DMA 内存布局 | ✅ 规范 | scatter 文件已更新，DTCM 警告已标注 |
| **UART1 二重占用** | ⏸️ 待处理 | 需业务决策：使用图传还是裁判系统 |
| **代码体积** | ⏸️ 延后 | 功能稳定后再拆分大文件 |

---

## 后续优先级

1. **高**：固化 UART1 owner，选择图传或裁判系统之一
2. **中**：验证 CAN 过滤器在实际总线上的效果（丢帧计数）
3. **中**：逐步为图传缓冲区添加 DMA section 标记
4. **低**：机械臂和 USB 任务拆分（待功能稳定）

---

## 验证方式

- 编译：`MDK-ARM/project.uvprojx` 应无错误
- 查看 map 文件：确认 DMA 符号地址 ≥ 0x24000000（AXI SRAM）
- 运行诊断：观察 CAN 队列溢出计数是否下降
- 监控：INS_Task / Control_Task 禁用后 CPU 利用率变化
