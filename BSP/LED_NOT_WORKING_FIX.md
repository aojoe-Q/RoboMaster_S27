# LED 不动了 — 紧急修复指南

## 问题现象
应用 v1.1.2 后，LED完全不动/不亮/不变化。

---

## 根本原因

**`__disable_irq()` 可能导致系统死锁！**

### 场景1：RTOS环境（FreeRTOS/RT-Thread）
```c
__disable_irq();  // 禁用所有中断
HAL_SPI_Transmit(...);  // 内部可能调用 osDelay() 或等待信号量
// ❌ 死锁！因为中断被禁用，RTOS无法调度，SPI永远等不到超时
```

### 场景2：裸机环境但中断配置错误
```c
__disable_irq();  // 禁用所有中断
HAL_SPI_Transmit(...);  // 等待 SPI->SR.TXE 标志
// ❌ 如果SPI依赖中断完成传输，永远等不到
```

### 场景3：编译器优化问题
```c
uint32_t primask = __get_PRIMASK();  // 可能被优化掉
__disable_irq();
// ...
__set_PRIMASK(primask);  // 恢复到错误的状态
```

---

## 🚀 立即修复（已应用）

### 修复版本：v1.1.3
**默认不屏蔽中断**，让 HAL 库自己处理 SPI 传输。

```c
void BSP_LED_SetRGB(uint8_t r, uint8_t g, uint8_t b)
{
  WS2812_Pack(ws_buf, g, r, b);
  last_rgb_valid = 0U;
  
  // ✅ 默认不屏蔽中断（兼容RTOS和裸机）
  HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi6, ws_buf, 9, 10);
  
  if (status == HAL_OK) {
    last_rgb_packed = RGB_Pack(r, g, b);
    last_rgb_valid  = 1U;
  }
}
```

### 验证步骤
1. 重新编译并烧录 `bsp_led.c` v1.1.3
2. 观察LED是否恢复正常
3. 如果恢复 → 说明是中断屏蔽导致的死锁
4. 如果仍不动 → 继续诊断（见下方）

---

## 🔍 深度诊断

### 检查1：LED初始化是否成功
```c
void test_led_init(void)
{
  BSP_LED_Init();
  
  // 手动测试：红色常亮2秒
  BSP_LED_SetRGB(255, 0, 0);
  HAL_Delay(2000);
  
  // 绿色常亮2秒
  BSP_LED_SetRGB(0, 255, 0);
  HAL_Delay(2000);
  
  // 蓝色常亮2秒
  BSP_LED_SetRGB(0, 0, 255);
  HAL_Delay(2000);
  
  printf("If colors changed, hardware is OK!\n");
}
```

**预期**：LED依次显示红绿蓝。
- ✅ 如果正常 → 初始化成功，问题在任务调度
- ❌ 如果不亮 → 硬件或SPI配置问题

---

### 检查2：SPI传输是否成功
```c
void test_spi_status(void)
{
  for (int i = 0; i < 10; i++) {
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi6, ws_buf, 9, 10);
    
    printf("Frame %d: ", i);
    switch (status) {
      case HAL_OK:      printf("✅ OK\n"); break;
      case HAL_TIMEOUT: printf("❌ TIMEOUT\n"); break;
      case HAL_ERROR:   printf("❌ ERROR\n"); break;
      case HAL_BUSY:    printf("⚠️ BUSY\n"); break;
      default:          printf("❓ UNKNOWN(%d)\n", status);
    }
    
    HAL_Delay(100);
  }
}
```

**预期**：全部显示 `✅ OK`。
- ❌ 如果有 TIMEOUT → SPI配置错误或时钟未开启
- ❌ 如果有 ERROR → SPI硬件故障
- ❌ 如果有 BUSY → SPI被其他代码占用

---

### 检查3：Task是否被调用
```c
void BSP_LED_Task(void)
{
  static uint32_t call_count = 0;
  call_count++;
  
  // 每100次打印一次
  if (call_count % 100 == 0) {
    printf("LED_Task called %lu times\n", call_count);
  }
  
  // ... 原有逻辑 ...
}
```

**预期**：串口持续输出 `LED_Task called ...`。
- ❌ 如果无输出 → **Task根本没被调用！**
- ✅ 如果有输出 → Task被调用，问题在内部逻辑

---

### 检查4：Mode是否设置
```c
void test_led_mode(void)
{
  printf("Current mode: %d\n", led_state.mode);
  printf("Target RGB: (%u, %u, %u)\n", 
         led_state.target_r, led_state.target_g, led_state.target_b);
  printf("Breath period: %u ms\n", led_state.breath_period);
  printf("Breath scale: %u\n", led_state.breath_scale);
}
```

**预期**：
- Mode应该 ≠ LED_MODE_OFF
- Target RGB应该 ≠ (0, 0, 0)（除非你设置的就是黑色）

---

### 检查5：相位是否推进
```c
void BSP_LED_Task(void)
{
  static uint16_t last_phase = 0;
  
  // ... 原有逻辑 ...
  
  // 检查相位变化
  if (led_state.breath_phase != last_phase) {
    printf("Phase: %u -> %u\n", last_phase, led_state.breath_phase);
    last_phase = led_state.breath_phase;
  } else {
    printf("⚠️ Phase stuck at %u!\n", led_state.breath_phase);
  }
}
```

**预期**：相位应该持续变化。
- ❌ 如果卡住 → `LED_AdvancePhase()` 有问题

---

## 🛠️ 常见问题与修复

### 问题1：LED完全不亮
**原因**：
1. 硬件未连接
2. SPI6时钟未开启
3. PA7未配置为AF模式

**修复**：
```c
// 在 BSP_LED_Init() 开头添加
__HAL_RCC_SPI6_CLK_ENABLE();
__HAL_RCC_GPIOA_CLK_ENABLE();

// 确认PA7配置
GPIO_InitTypeDef GPIO_InitStruct = {0};
GPIO_InitStruct.Pin = GPIO_PIN_7;
GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;  // ✅ 必须是复用推挽
GPIO_InitStruct.Alternate = GPIO_AF8_SPI6;  // ✅ 必须是AF8
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
```

---

### 问题2：LED亮但不变化
**原因**：
1. `BSP_LED_Task()` 未被调用
2. Mode设置为 `LED_MODE_OFF` 或 `LED_MODE_ON`
3. 任务调度周期过长（>1秒）

**修复**：
```c
// 在 main.c 或任务中确保定期调用
while (1) {
  BSP_LED_Task();  // ✅ 每20ms调用一次
  HAL_Delay(20);
}

// 设置呼吸模式
BSP_LED_SetMode(LED_MODE_BREATHE, 0, 40, 255);
```

---

### 问题3：LED偶尔闪一下然后停止
**原因**：
1. SPI传输失败（`status != HAL_OK`）
2. `last_rgb_valid` 一直为1，导致去重失效

**修复**：
```c
// 临时禁用去重测试
void LED_UpdateRGB(void)
{
  uint8_t r, g, b;
  // ... 计算 r, g, b ...
  
  // ❌ 注释掉去重逻辑
  // uint32_t current_rgb = RGB_Pack(r, g, b);
  // if ((!last_rgb_valid) || (current_rgb != last_rgb_packed)) {
    BSP_LED_SetRGB(r, g, b);
  // }
}
```

---

### 问题4：RTOS环境下LED卡死
**原因**：
1. `__disable_irq()` 导致死锁（已修复）
2. Task优先级过低，被其他任务饿死
3. 栈空间不足

**修复**：
```c
// FreeRTOS中
xTaskCreate(led_task, "LED", 256, NULL, 2, NULL);  // ✅ 优先级至少2
                                    ↑ 栈空间至少256字（1KB）

// 检查栈使用
UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
printf("LED task stack remaining: %u words\n", uxHighWaterMark);
```

---

## 🔧 临时解决方案

### 方案1：强制刷新（绕过去重）
```c
void BSP_LED_SetRGB_Force(uint8_t r, uint8_t g, uint8_t b)
{
  last_rgb_valid = 0;  // ✅ 强制标记为无效，绕过去重
  BSP_LED_SetRGB(r, g, b);
}

// 使用
BSP_LED_SetRGB_Force(255, 0, 0);
```

### 方案2：复位LED状态
```c
void BSP_LED_Reset(void)
{
  // 复位状态机
  led_state.mode = LED_MODE_OFF;
  led_state.breath_phase = 0;
  led_state.color_phase = 0;
  last_rgb_valid = 0;
  
  // 发送全0帧
  BSP_LED_SetRGB(0, 0, 0);
  HAL_Delay(1);
  
  // 重新初始化
  BSP_LED_Init();
}
```

### 方案3：降级到基础功能
```c
// 完全绕过状态机，直接控制
void test_basic_control(void)
{
  extern SPI_HandleTypeDef hspi6;
  extern uint8_t ws_buf[];
  
  for (int i = 0; i < 256; i++) {
    // 直接打包
    WS2812_Pack(ws_buf, i, 0, 0);  // 绿色渐亮
    
    // 直接发送
    HAL_SPI_Transmit(&hspi6, ws_buf, 9, 10);
    
    HAL_Delay(10);
  }
}
```

---

## 📊 诊断决策树

```
LED不动
    ↓
运行 test_led_init()
    ├─ 亮 → 硬件OK
    │       ↓
    │   运行 test_led_mode()
    │       ├─ Mode=OFF → 设置为 BREATHE
    │       └─ Mode≠OFF → 运行 test_spi_status()
    │                       ├─ 全OK → 检查Task调用
    │                       └─ 有错误 → SPI配置问题
    │
    └─ 不亮 → 硬件故障
            ↓
        检查：
        1. PA7焊接
        2. WS2812供电（5V）
        3. 示波器测PA7波形
```

---

## ✅ 快速验证清单

**1分钟检查**：
- [ ] 重新烧录 v1.1.3（已移除 `__disable_irq()`）
- [ ] LED是否恢复正常

**5分钟检查**：
- [ ] 运行 `test_led_init()` → LED依次显示红绿蓝
- [ ] 运行 `test_spi_status()` → 全部显示 `✅ OK`
- [ ] 运行 `test_led_mode()` → Mode ≠ OFF

**30分钟深度检查**：
- [ ] 添加 `call_count` 日志 → Task被持续调用
- [ ] 添加相位日志 → Phase持续变化
- [ ] 用示波器测PA7 → 有2.5MHz波形

---

## 🎯 最可能的原因（排序）

1. **`__disable_irq()` 导致死锁** — 90%
   - 修复：已移除，使用 v1.1.3

2. **Task未被调用** — 5%
   - 检查：添加 `call_count` 日志
   - 修复：确保 `main.c` 中定期调用 `BSP_LED_Task()`

3. **Mode设置为OFF** — 3%
   - 检查：打印 `led_state.mode`
   - 修复：调用 `BSP_LED_SetMode(LED_MODE_BREATHE, ...)`

4. **SPI配置错误** — 2%
   - 检查：运行 `test_spi_status()`
   - 修复：重新配置SPI6时钟和GPIO

---

## 📞 紧急联系

如果以上方案都无效：

1. **提供以下信息**：
   - 运行环境：裸机 / RTOS（哪个？）
   - 编译选项：优化级别（-O0/-O2/-O3）
   - 测试结果：`test_led_init()` / `test_spi_status()` 输出
   - 串口日志：完整的调试输出

2. **临时workaround**：
   - 使用 `BSP_LED_SetRGB_Force()` 强制刷新
   - 或使用 `test_basic_control()` 基础控制

3. **回滚到原始版本**：
   ```bash
   cp bsp_led.c.v1.0.backup bsp_led.c
   # 重新编译，确认原版本是否正常
   ```

---

**版本**: v1.1.3 (紧急修复：移除 `__disable_irq()`）  
**状态**: 已应用，待验证  
**预期**: LED应该恢复正常工作
