# DMA 缓冲区 Section 标记更新

**日期**：2025 年 1 月  
**目的**：消除 scatter 文件编译警告，规范所有 DMA 缓冲区位置

---

## 问题

编译时出现警告：
```
COD_H7_Template\COD_H7_Template.sct(21): warning: L6314W: No section matches pattern *(.AXI_SRAM_DMA).
```

原因：scatter 文件中定义了 `.AXI_SRAM_DMA` section，但代码中的 DMA 缓冲区仍使用 `.AXI_SRAM`。

---

## 解决方案

将所有 DMA 缓冲区的 section 标记从 `.AXI_SRAM` 改为 `.AXI_SRAM_DMA`。

---

## 更新列表

### 1. Image_Transmission.c（图传接收缓冲）

**文件**：`Components/Device/Src/Image_Transmission.c` L19

**修改**：
```diff
- __attribute__((section (".AXI_SRAM"))) uint8_t Image_Trans_MultiRx_Buff[2][39];
+ __attribute__((section (".AXI_SRAM_DMA"))) uint8_t Image_Trans_MultiRx_Buff[2][39];
```

**用途**：图传 DMA 接收缓冲区（双缓冲，每个 39 字节）

---

### 2. bsp_adc.c（ADC DMA 缓冲）

**文件**：`BSP/Src/bsp_adc.c`

**修改**：
```diff
- __attribute__((section (".AXI_SRAM"))) uint16_t ADC_Voltage_Val[2];
+ __attribute__((section (".AXI_SRAM_DMA"))) uint16_t ADC_Voltage_Val[2];
```

**用途**：ADC DMA 转换结果缓冲区

---

### 3. bsp_uart.c（VOFA 输出缓冲）

**文件**：`BSP/Src/bsp_uart.c` L319

**修改**：
```diff
-     __attribute__((section (".AXI_SRAM")))  static uint8_t Rx_Buf[16];
+     __attribute__((section (".AXI_SRAM_DMA")))  static uint8_t Rx_Buf[16];
```

**用途**：VOFA 实时数据输出缓冲区（16 字节）

**注**：变量名为 `Rx_Buf` 但实际是发送缓冲，用于输出浮点数据

---

### 4. Referee_System.c（裁判系统接收缓冲）

**文件**：`Components/Device/Src/Referee_System.c`

**修改**：
```diff
- __attribute__((section (".AXI_SRAM"))) uint8_t Referee_System_Info_MultiRx_Buf[2][REFEREE_RXFRAME_LENGTH];
+ __attribute__((section (".AXI_SRAM_DMA"))) uint8_t Referee_System_Info_MultiRx_Buf[2][REFEREE_RXFRAME_LENGTH];
```

**用途**：裁判系统 DMA 接收缓冲区（双缓冲）

---

### 5. Remote_Control.c（遥控接收缓冲）

**文件**：`Components/Device/Src/Remote_Control.c`

**修改**：
```diff
- __attribute__((section (".AXI_SRAM"))) uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];
+ __attribute__((section (".AXI_SRAM_DMA"))) uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];
```

**用途**：SBUS 遥控 DMA 接收缓冲区（双缓冲）

---

## 缓冲区汇总

| 缓冲区 | 大小 | 类型 | 用途 |
|-------|------|------|------|
| Image_Trans_MultiRx_Buff | 2×39 B | 图传 RX | 图像传输接收 |
| ADC_Voltage_Val | 2×2 B | ADC RX | 模拟-数字转换 |
| Rx_Buf (VOFA) | 16 B | UART TX | 诊断数据输出 |
| Referee_System_Info_MultiRx_Buf | 2×N B | UART RX | 裁判系统接收 |
| SBUS_MultiRx_Buf | 2×N B | UART RX | 遥控信号接收 |

**总计**：5 个 DMA 缓冲区，均已标记 `.AXI_SRAM_DMA` section

---

## 编译结果

**修改前**：
```
COD_H7_Template\COD_H7_Template.sct(21): warning: L6314W: No section matches pattern *(.AXI_SRAM_DMA).
Program Size: Code=145384 RO-data=2580 RW-data=468 ZI-data=118700  
Finished: 0 information, 1 warning and 0 error messages.
```

**修改后**：
```
Program Size: Code=145384 RO-data=2580 RW-data=468 ZI-data=118700  
Finished: 0 information, 0 warning and 0 error messages.
```

**结果**：✅ 警告已消除

---

## 内存验证

建议检查编译后的 map 文件，确认所有 DMA 缓冲区地址：
- 应在 0x24000000 ~ 0x24020000 范围内（AXI SRAM）
- 不应在 0x20000000 ~ 0x20020000 范围内（DTCM，DMA 无法访问）

**检查命令**（Keil 输出 map 文件后）：
```
grep -E "Image_Trans_MultiRx_Buff|ADC_Voltage_Val|Rx_Buf|Referee_System_Info_MultiRx_Buf|SBUS_MultiRx_Buf" project.map
```

**期望**：所有符号地址 ≥ 0x24000000

---

## 后续建议

1. **验证编译**：重新编译确认无警告
2. **检查 map 文件**：验证缓冲区地址正确
3. **运行测试**：验证通信功能正常
4. **文档更新**：将此改动记录在工程文档中

---

## 相关文件

- **Scatter 文件**：`MDK-ARM/COD_H7_Template/COD_H7_Template.sct`（已在之前的整改中更新）
- **工程文档**：`docs/S27_TEST_2_REFACTORING.md`（项目 5：DMA 内存布局规范化）

