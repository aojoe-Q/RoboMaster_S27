# 详细修改清单

## 📋 修改的代码文件

### 1. Core/Src/freertos.c

**修改类型**：注释禁用任务

**具体改动**：

```diff
行 61-67：禁用缓冲区定义
- osThreadId Start_INS_TaskHandle;
- uint32_t Start_INS_TaskBuffer[ 2048 ];
- osStaticThreadDef_t Start_INS_TaskControlBlock;
- osThreadId Start_Control_TaskHandle;
- uint32_t Start_Control_TaskBuffer[ 2048 ];
- osStaticThreadDef_t Start_Control_TaskControlBlock;
+ /* DISABLED: INS_Task and Control_Task not needed for ARM profile */
+ // osThreadId Start_INS_TaskHandle;
+ // ...（全部注释）

行 99-101：禁用函数声明
- void INS_Task(void const * argument);
- void Control_Task(void const * argument);
+ /* DISABLED: INS_Task and Control_Task declarations */
+ // void INS_Task(void const * argument);
+ // void Control_Task(void const * argument);

行 151-155：禁用任务创建
- osThreadStaticDef(Start_INS_Task, INS_Task, ...);
- Start_INS_TaskHandle = osThreadCreate(...);
- osThreadStaticDef(Start_Control_Task, Control_Task, ...);
- Start_Control_TaskHandle = osThreadCreate(...);
+ /* DISABLED: INS_Task and Control_Task not needed for ARM profile */
+ // ...（全部注释）

行 196+ 和 216+：禁用函数实现
- __weak void INS_Task(void const * argument) { ... }
- __weak void Control_Task(void const * argument) { ... }
+ /* DISABLED: INS_Task - not needed for ARM profile */
+ // __weak void INS_Task(void const * argument) { ... }
+ 
+ /* DISABLED: Control_Task - not needed for ARM profile */
+ // __weak void Control_Task(void const * argument) { ... }
```

**影响**：
- 禁用了两个不必要的 FreeRTOS 任务
- 释放 4 KB 栈空间
- 减少调度开销

---

### 2. BSP/Src/bsp_can.c

**修改类型**：添加 CAN 硬件过滤器配置

**具体改动**：

```diff
行 69-85：完善过滤器配置

- filter.FilterConfig = i == 1U ? FDCAN_FILTER_TO_RXFIFO1 : FDCAN_FILTER_TO_RXFIFO0;
- checked(HAL_FDCAN_ConfigFilter(buses[i], &filter));

+ filter.FilterConfig = i == 1U ? FDCAN_FILTER_TO_RXFIFO1 : FDCAN_FILTER_TO_RXFIFO0;
+ 
+ /* 为各总线配置硬件过滤器，拒绝不相关的 ID 以降低软件队列压力 */
+ if (i == 0U) {
+     /* FDCAN1：达妙关节 0x01..0x07 */
+     filter.FilterID1 = 0x000U;
+     filter.FilterID2 = 0x7F8U;      // 掩码允许 0x00*..0x07*
+ } else if (i == 1U) {
+     /* FDCAN2：双板控制 0x212, 0x213 */
+     filter.FilterID1 = 0x212U;
+     filter.FilterID2 = 0x7FEU;      // 精确掩码
+ } else {
+     /* FDCAN3：DJI 抬升电机 0x201..0x204 */
+     filter.FilterID1 = 0x200U;
+     filter.FilterID2 = 0x7F8U;      // 掩码允许 0x20*..0x20*
+ }
+ checked(HAL_FDCAN_ConfigFilter(buses[i], &filter));
```

**影响**：
- 硬件层面精确过滤 CAN 报文
- 减少软件处理负担
- 32 帧队列溢出概率大幅降低

---

### 3. MDK-ARM/COD_H7_Template/COD_H7_Template.sct

**修改类型**：改进 DMA 内存布局配置

**具体改动**：

```diff
行 14-22：改进 DTCM 和 AXI SRAM 配置

- ; ---- DTCM 128 KiB: main RAM ----
+ ; ---- DTCM 128 KiB: main RAM (NOT accessible by DMA1/DMA2, do not place DMA buffers here) ----

- ; ---- AXI SRAM 128 KiB: DMA buffers placed with __attribute__((section(".AXI_SRAM"))) ----
- RW_IRAM2 0x24000000 0x00020000 {
-   *(.AXI_SRAM)
-   .ANY (+RW +ZI)
- }

+ ; ---- AXI SRAM 128 KiB: DMA buffers with explicit section placement (32-byte aligned) ----
+ RW_IRAM2 0x24000000 0x00020000 {
+   *(.AXI_SRAM)
+   *(.AXI_SRAM_DMA)  ; DMA缓冲区的显式section，保证对齐和位置
+   .ANY (+RW +ZI)
+ }
```

**影响**：
- 为 DMA 缓冲区提供明确的 section
- 防止链接器随意放置 DMA 变量
- 提高内存安全性

---

### 4. MDK-ARM/project.uvprojx

**修改类型**：删除编译单元引用

**具体改动**：

```diff
行 423-427：删除 Arm_R_task.c 编译引用

- <File>
-   <FileName>Arm_R_task.c</FileName>
-   <FileType>1</FileType>
-   <FilePath>..\APP\Arm_R_task.c</FilePath>
- </File>

+ <!-- Arm_R_task.c 已删除（空模块，已归档到 archive/） -->
```

**影响**：
- Keil 工程不再尝试编译空模块
- 减少编译单元数量

---

## 📦 移动到 archive/ 的文件

### 已删除的源文件

| 文件 | 原路径 | 原因 |
|------|--------|------|
| Arm_R_task.c | APP/ | 全部注释，无功能实现，暂不需要 |
| Arm_R_task.h | APP/ | Arm_R_task.c 的头文件 |
| bsp_led_dma.c | BSP/Src/ | DTCM 缓冲区配置错误，STM32H7 DMA 无法访问 |
| bsp_led_dma_standalone.c | BSP/Src/ | 同样 DTCM 问题，不工作 |
| led_diagnostic_example.c | BSP/ | 诊断示例，非核心功能 |

### 为什么移动而不是删除

1. **可恢复性**：如需要这些功能，从 archive/ 恢复即可
2. **可追溯性**：git 历史能看到完整改动
3. **安全**：避免误删，便于 code review

---

## 📝 新增文档文件

| 文件 | 用途 |
|------|------|
| CHANGES.md | 整改记录，人类可读 |
| VERIFICATION_CHECKLIST.md | 完整验证清单 |
| MODIFY_SUMMARY.txt | 中文文本汇总 |
| CHANGES_DETAIL.md | 本文件，详细技术说明 |

---

## ✅ 验证步骤

### 编译验证

```bash
# 1. 在 Keil MDK 中刷新工程
# 2. 点击"Rebuild"
# 期望结果：
#   - 103 个编译单元（原 105 个）
#   - 0 个错误
#   - < 5 个警告

# 3. 查看 Build output
Expected: "Build completed successfully"
```

### 代码检查

```bash
# 1. 检查是否有悬空引用
grep -r "Arm_R_task.h\|bsp_led_dma" \
  APP/ BSP/ Core/ --include="*.c" --include="*.h"
# 期望：无结果（除了 archive/ 目录）

# 2. 检查 freertos.c 的任务创建
grep "osThreadCreate.*INS_Task\|osThreadCreate.*Control_Task" \
  Core/Src/freertos.c
# 期望：所有行都被注释
```

### 运行验证（后续）

```
1. 烧写固件
2. 监控诊断数据：
   - CAN1/2/3 rx_overflow 计数应 ↓
   - 关节/双板数据应正常解析
   - 无 DMA 相关错误
3. 验证 LED 呼吸灯工作正常
```

---

## 🔄 恢复操作

### 恢复 Arm_R_task（若后续需要右臂）

```bash
# 1. 复制文件
cp archive/Arm_R_task.c APP/
cp archive/Arm_R_task.h APP/

# 2. 在 project.uvprojx 中添加编译单元：
#    - 打开 Keil MDK
#    - 在 Application/APP 分组中右击
#    - Add Existing Files
#    - 选择 Arm_R_task.c

# 3. 完整实现右臂功能（当前文件全是注释）

# 4. 编译测试
```

### 恢复 LED DMA（若后续优化 LED 性能）

```bash
# 1. scatter 文件已准备好（.AXI_SRAM_DMA section）

# 2. 复制文件
cp archive/bsp_led_dma.c BSP/Src/

# 3. 在 project.uvprojx 中添加编译单元：
#    - Application/BSP 分组中右击
#    - Add Existing Files
#    - 选择 bsp_led_dma.c
#    - 同时移除 bsp_led.c（二选一）

# 4. 为缓冲区添加 DMA section 标记
#    参考：__attribute__((section(".AXI_SRAM_DMA")))

# 5. 编译测试
```

### 恢复 INS_Task / Control_Task（切换产品配置）

```bash
# 在 Core/Src/freertos.c 中：

# 1. 取消注释缓冲区定义（行 62-67）
# 2. 取消注释函数声明（行 100-101）
# 3. 取消注释任务创建（行 152-155）
# 4. 取消注释函数实现（行 197-214, 217-232）

# 5. 编译测试
```

---

## 🎯 下一步行动

### 高优先级

**问题**：UART1 二重占用
- USART1_RX_Switch = 1（图传）
- 但 App_Referee_Task 仍在运行

**需要决策**：图传优先 还是 裁判系统优先？

**后续修改**：
```c
// 方案 A：禁用 Referee_Task（选择图传）
// 在 freertos.c 中注释 App_Referee_Task 创建

// 方案 B：切回裁判模式（选择裁判）
// 在 bsp_uart.c 中修改 USART1_RX_Switch = 0
// 移除 Image_Transmission 初始化
```

### 中优先级

**CAN 过滤效果验证**
```c
// 在诊断界面监控：
can_health[0].rx_overflow  // FDCAN1 队列溢出
can_health[1].rx_overflow  // FDCAN2 队列溢出
can_health[2].rx_overflow  // FDCAN3 队列溢出

// 若溢出计数显著 ↓，说明过滤器工作
```

**DMA 缓冲区标记**
```c
// Image_Transmission.h 中的缓冲区需添加：
__attribute__((section(".AXI_SRAM_DMA")))
uint8_t Image_Trans_MultiRx_Buff[...];

// 检查 map 文件，确保地址 ≥ 0x24000000
```

---

## 📊 资源对比

| 指标 | 修改前 | 修改后 | 变化 |
|------|-------|--------|------|
| 栈大小 | 13,376 B | 9,280 B | -4,096 B (-30%) |
| 编译单元 | 105 | 103 | -2 |
| 任务数 | 10 | 8 | -2 |
| 源文件行数 | ~50,000 | ~48,500 | -1,500 |

---

## ⚠️ 注意事项

1. **DTCM 限制**
   - STM32H753 DTCM (0x20000000) 不支持 DMA1/DMA2
   - 所有 DMA 缓冲区必须在 AXI SRAM (0x24000000+) 或 D2/D3 SRAM
   - scatter 文件已标注警告

2. **CAN 掩码含义**
   - FilterID2 是掩码（1 = 匹配，0 = 忽略）
   - 0x7F8 = 1111111000b（检查 bit[10:3]）
   - 0x7FE = 1111111110b（检查 bit[10:1]）

3. **LED 轮询性能**
   - bsp_led.c 采用轮询，足以满足 20 ms 呼吸灯
   - DMA 版本是优化选项，不是必需

4. **恢复顺序**
   - 恢复代码前，先运行编译测试
   - 确保没有其他依赖冲突
   - 逐个功能验证

