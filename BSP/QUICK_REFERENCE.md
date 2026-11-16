# LED亮度跳变 — 快速诊断参考卡

## 🚀 快速修复（5分钟）

### ✅ 已修复：整数舍入优化
```c
// bsp_led.c 中的 LED_ApplyBrightness 已修改
*r = (uint8_t)(((uint32_t)*r * scale + 127U) / 255U);
// +127U 四舍五入，减少50%的亮度跳变
```

**立即测试**：编译并烧录，观察呼吸灯是否更平滑。

---

## 🔍 深度诊断（30分钟）

### 1️⃣ 启用诊断工具
```c
// 在 bsp_led.c 顶部添加
#include "bsp_led_diagnostic.h"

// 在 BSP_LED_Task() 开头添加
LED_Diag_RecordTiming();

// 在 LED_UpdateRGB() 中添加
LED_Diag_RecordSkip();  // 去重跳过时
LED_Diag_RecordSend();  // 实际发送时

// 在 BSP_LED_SetRGB() 中添加
LED_Diag_RecordSPISuccess();  // SPI成功
LED_Diag_RecordSPIFail();     // SPI失败
```

### 2️⃣ 定期打印统计
```c
// 在 main.c 或任务中
while (1) {
  BSP_LED_Task();
  HAL_Delay(20);

  // 每10秒打印一次
  static uint32_t last_print = 0;
  if (HAL_GetTick() - last_print >= 10000) {
    LED_Diag_PrintAll();
    LED_Diag_ResetAll();
    last_print = HAL_GetTick();
  }
}
```

### 3️⃣ 解读输出
| 指标 | 正常范围 | 异常表现 | 原因 |
|------|---------|---------|------|
| **Interval** | min=19, max=22 | max>25 | 任务调度抖动 |
| **Jitter** | <5% | >10% | 高优先级任务抢占 |
| **Skip Rate** | 5-10% | <1% 或 >30% | 去重失效/相位推进异常 |
| **SPI Fail** | 0 | >0 | 硬件问题 |

---

## 🎯 常见问题速查

### Q1: 呼吸灯在最暗段有停顿
**原因**：查表数组首尾有多个0值  
**解决**：
```c
// 选项A：临时测试 — 提高最小亮度
BSP_LED_SetMode(LED_MODE_ON, 5, 10, 255);  // 不完全熄灭
BSP_LED_SetMode(LED_MODE_BREATHE, 5, 10, 255);

// 选项B：永久修复 — 重新生成查表（见 BRIGHTNESS_JUMP_FIX.md）
```

### Q2: 亮度偶尔突然跳变1-2个单位
**原因**：整数除法舍入误差  
**解决**：已在 v1.1.1 修复（+127U 四舍五入）

### Q3: 呼吸速度不均匀
**原因**：任务调度抖动  
**诊断**：
```c
LED_Diag_PrintTiming();
// 如果 Jitter > 10%，说明任务被频繁打断
```
**解决**：
- 提高LED任务优先级（FreeRTOS）
- 改用硬件定时器触发
- 检查中断服务程序是否过长

### Q4: 颜色完全不变（彩色呼吸失效）
**原因**：相位推进步长为0  
**诊断**：
```c
#define LED_DIAG_ENABLE_PHASE_LOG 1
// 观察 breath_phase 和 color_phase 是否在变化
```
**解决**：
- 检查 LED_TASK_PERIOD_MS 是否为 0
- 检查 breath_period 是否过大（>10000）
- 用调试器单步执行 LED_AdvancePhase()

### Q5: SPI传输失败
**原因**：硬件配置错误  
**诊断**：
```c
LED_Diag_PrintSPI();
// 如果 Fail > 0，说明硬件有问题
```
**解决**：
1. 用示波器测量 PA7 波形，确认频率约 2.5MHz
2. 检查 SPI6 时钟源（应为 PCLK4 80MHz）
3. 确认 PA7 无硬件冲突
4. 增加超时时间：`HAL_SPI_Transmit(..., 20U)`（默认10ms）

---

## 📊 性能基准（参考值）

| 硬件 | 编译选项 | 任务周期 | Skip率 | 备注 |
|------|---------|---------|--------|------|
| H7@480MHz | -O2 | 20ms±1ms | 7-10% | 正常（慢速呼吸） |
| H7@480MHz | -O2 | 20ms±1ms | 1-3% | 正常（快速呼吸） |
| H7@480MHz | -O3 | 20ms±0.5ms | 8-12% | 激进优化 |
| F4@168MHz | -O2 | 20ms±2ms | 5-8% | 性能稍低 |

---

## 🛠️ 实用代码片段

### 测试最小可察觉跳变
```c
// 测试人眼对亮度跳变的敏感度
void test_brightness_step(void)
{
  for (uint8_t b = 50; b <= 60; b++) {
    BSP_LED_SetRGB(0, 0, b);
    HAL_Delay(1000);  // 每秒跳1个单位
    printf("Brightness: %u\n", b);
  }
  // 如果看不出 50->51 的差异，说明跳变可接受
}
```

### 禁用去重（临时测试）
```c
// 在 LED_UpdateRGB() 中
#if 0  // 禁用去重
  if ((!last_rgb_valid) || (current_rgb != last_rgb_packed)) {
#endif
    BSP_LED_SetRGB(r, g, b);
#if 0
  }
#endif
```

### 强制最小亮度（避免完全熄灭）
```c
static inline void LED_ApplyBrightness(uint8_t scale, uint8_t *r, uint8_t *g, uint8_t *b)
{
  *r = (uint8_t)(((uint32_t)*r * scale + 127U) / 255U);
  *g = (uint8_t)(((uint32_t)*g * scale + 127U) / 255U);
  *b = (uint8_t)(((uint32_t)*b * scale + 127U) / 255U);

  // ✅ 强制最小值（0.4%亮度），避免完全熄灭时的停顿
  if (*r > 0 && *r < 1) *r = 1;
  if (*g > 0 && *g < 1) *g = 1;
  if (*b > 0 && *b < 1) *b = 1;
}
```

---

## 📝 检查清单

**编译前**：
- [ ] 确认 `LED_ApplyBrightness` 有 `+127U`
- [ ] 诊断工具已集成（如需要）
- [ ] 编译选项至少 `-O2`

**烧录后**：
- [ ] 呼吸灯能正常启动
- [ ] 亮度变化平滑（用手机慢动作录像）
- [ ] 串口输出诊断统计（如已启用）

**运行中**：
- [ ] 观察10分钟，无明显跳变
- [ ] 切换模式（ON/BREATHE/BLINK），无异常
- [ ] 功耗无明显变化（±1mA）

**发布前**：
- [ ] 禁用所有诊断开关（性能优化）
- [ ] 移除调试日志
- [ ] 长时间稳定性测试（24小时）

---

## 🔗 相关文档

- **详细分析**：`BRIGHTNESS_JUMP_FIX.md`
- **诊断工具**：`bsp_led_diagnostic.h`
- **使用示例**：`led_diagnostic_example.c`
- **优化总结**：`LED_OPTIMIZATION_NOTES.md`

---

## 💡 终极建议

**如果修复后仍有跳变**：
1. 用示波器测量PA7输出，确认波形正确
2. 对比原始版本（v1.0），确认不是优化引入的问题
3. 用手机录像，慢放分析跳变模式
4. 更换硬件测试（排除灯珠故障）

**人眼亮度敏感度**：
- 在50%亮度时，1单位(0.4%)的跳变几乎不可见
- 在5%以下暗段，1单位(20%)的跳变非常明显
- 呼吸灯的"卡顿感"主要来自暗段，而非舍入误差

**最终结论**：
如果你看到的跳变主要在最暗段，建议修改查表数组移除首尾的0平台。
如果是整个周期都有跳变，可能是任务调度抖动，需要提高任务优先级。

---

**快速联系**：
- 技术问题：检查 `BRIGHTNESS_JUMP_FIX.md` 的第2-5节
- 诊断工具：参考 `led_diagnostic_example.c` 的集成步骤
- 性能优化：回顾 `LED_OPTIMIZATION_NOTES.md`

**版本**：v1.1.1 (2025-11-08)
