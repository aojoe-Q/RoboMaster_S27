# bsp_led.c 优化验证清单

## 编译验证

### 1. 基础编译检查
```bash
# 在 Makefile 所在目录执行
make clean
make -j8 2>&1 | tee build.log

# 检查警告
grep -i "warning" build.log
grep -i "error" build.log
```

**预期结果**：
- ✅ 编译成功，无 error
- ✅ 无 warning（或仅有已知的无关警告）
- ✅ bsp_led.c 的编译输出无异常

---

### 2. 编译选项确认
```bash
# 确认优化级别
grep "^CFLAGS" Makefile

# 推荐配置
# -O2: 平衡优化
# -O3: 激进优化（可能增大代码体积）
# -Os: 代码体积优化
```

**推荐设置**：
```makefile
CFLAGS += -O2
CFLAGS += -Wall -Wextra
CFLAGS += -ffunction-sections -fdata-sections
LDFLAGS += -Wl,--gc-sections
```

---

### 3. 代码体积对比
```bash
# 编译前
ls -lh build/*.elf > size_before.txt
arm-none-eabi-size build/*.elf >> size_before.txt

# 编译后（替换优化后的 bsp_led.c）
make clean && make
ls -lh build/*.elf > size_after.txt
arm-none-eabi-size build/*.elf >> size_after.txt

# 对比
diff size_before.txt size_after.txt
```

**预期变化**：
- text 段（Flash）：±50 字节（内联函数影响）
- data 段（初始化数据）：无变化
- bss 段（未初始化数据）：-4 字节（合并 RGB 状态变量）

---

## 功能验证

### 4. LED基础功能
| 测试项 | 操作 | 预期结果 | 实测 |
|--------|------|----------|------|
| 初始化 | `BSP_LED_Init()` | LED熄灭 | [ ] |
| 红色常亮 | `SetMode(LED_MODE_ON, 255, 0, 0)` | 纯红色 | [ ] |
| 绿色常亮 | `SetMode(LED_MODE_ON, 0, 255, 0)` | 纯绿色 | [ ] |
| 蓝色常亮 | `SetMode(LED_MODE_ON, 0, 0, 255)` | 纯蓝色 | [ ] |
| 白色常亮 | `SetMode(LED_MODE_ON, 255, 255, 255)` | 白色 | [ ] |
| 熄灭 | `SetMode(LED_MODE_OFF, 0, 0, 0)` | 完全熄灭 | [ ] |

---

### 5. 呼吸灯验证
| 测试项 | 配置 | 预期结果 | 实测 |
|--------|------|----------|------|
| 蓝色呼吸 | `SetMode(LED_MODE_BREATHE, 0, 40, 255)` + 默认周期 | 1.5秒周期，亮暗平滑 | [ ] |
| 快速呼吸 | `SetBreathPeriod(500)` | 0.5秒周期 | [ ] |
| 慢速呼吸 | `SetBreathPeriod(5000)` | 5秒周期 | [ ] |
| 边界钳位 | `SetBreathPeriod(100)` | 自动钳位到200ms | [ ] |
| 边界钳位 | `SetBreathPeriod(20000)` | 自动钳位到10000ms | [ ] |

**验证要点**：
- 呼吸曲线平滑，无阶跃感
- 暗段不应完全熄灭（gamma曲线效果）
- 周期准确（用秒表计时10个周期，误差<±0.5秒）

---

### 6. 彩色呼吸验证
| 测试项 | 配置 | 预期结果 | 实测 |
|--------|------|----------|------|
| 默认彩虹呼吸 | `SetMode(LED_MODE_BREATHE_COLOR, ...)` | 色相缓慢变化，7秒一圈 | [ ] |
| 快速变色 | `SetColorPeriod(2000)` | 2秒变完一圈 | [ ] |
| 慢速变色 | `SetColorPeriod(15000)` | 15秒变完一圈 | [ ] |
| 淡彩模式 | `SetSaturation(150)` | 颜色淡化，接近白光 | [ ] |
| 纯色模式 | `SetSaturation(255)` | 纯色，饱和度高 | [ ] |

**验证要点**：
- 色相变化顺序：红 → 黄 → 绿 → 青 → 蓝 → 洋红 → 红
- 亮度呼吸和色相变化独立（不应同步）
- 颜色过渡平滑，无跳变

---

### 7. 闪烁模式验证
| 测试项 | 配置 | 预期结果 | 实测 |
|--------|------|----------|------|
| 红色闪烁 | `SetMode(LED_MODE_BLINK, 255, 0, 0)` | 150ms亮/150ms灭，交替 | [ ] |
| 频率准确性 | 用示波器测量 | 周期300ms（3.33Hz） | [ ] |
| 长时间稳定性 | 观察10分钟 | 频率不漂移 | [ ] |

---

### 8. 实时模式切换
```c
// 测试代码示例
void test_mode_switching(void)
{
  BSP_LED_SetMode(LED_MODE_ON, 255, 0, 0);
  HAL_Delay(1000);
  
  BSP_LED_SetMode(LED_MODE_BREATHE, 0, 255, 0);
  HAL_Delay(3000);
  
  BSP_LED_SetMode(LED_MODE_BREATHE_COLOR, 0, 0, 0);
  HAL_Delay(5000);
  
  BSP_LED_SetMode(LED_MODE_BLINK, 0, 0, 255);
  HAL_Delay(3000);
  
  BSP_LED_SetMode(LED_MODE_OFF, 0, 0, 0);
}
```

**预期结果**：
- ✅ 模式切换无延迟
- ✅ 无闪烁或异常颜色
- ✅ 从呼吸切到其他模式时，相位重置

---

## 性能验证

### 9. 执行时间测量（可选）

#### 方法1：GPIO翻转 + 示波器
```c
// 在 BSP_LED_Task 前后插入
void BSP_LED_Task(void)
{
  HAL_GPIO_WritePin(TEST_PORT, TEST_PIN, GPIO_PIN_SET);
  
  // ... 原有代码 ...
  
  HAL_GPIO_WritePin(TEST_PORT, TEST_PIN, GPIO_PIN_RESET);
}
```
**测量**：用示波器测量高电平宽度

**预期**：
- 优化前：~10-15us（BREATHE_COLOR模式）
- 优化后：~9-13us
- 提升：约8-12%

---

#### 方法2：DWT计数器（H7支持）
```c
// 启用 DWT 计数器（在 main.c 初始化时调用）
void DWT_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// 在测试代码中
uint32_t start = DWT->CYCCNT;
BSP_LED_Task();
uint32_t cycles = DWT->CYCCNT - start;

// 周期数 -> 时间
float us = (float)cycles / 480.0f;  // 假设 480MHz
printf("LED Task: %.2f us (%lu cycles)\n", us, cycles);
```

**预期**：
- BREATHE模式：~6000-8000 周期（12-16us）
- BREATHE_COLOR模式：~8000-10000 周期（16-20us）

---

### 10. 去重功能验证
```c
// 测试 LED_UpdateRGB 的去重逻辑
void test_dedup(void)
{
  BSP_LED_SetRGB(100, 50, 200);  // 首次发送
  
  uint32_t start = DWT->CYCCNT;
  BSP_LED_SetRGB(100, 50, 200);  // 重复颜色，应跳过
  uint32_t cycles_skipped = DWT->CYCCNT - start;
  
  BSP_LED_SetRGB(101, 50, 200);  // 不同颜色，应发送
  uint32_t cycles_sent = DWT->CYCCNT - start;
  
  // 验证：跳过时的周期数应远小于发送时
  assert(cycles_skipped < 100);       // 应<0.2us（纯比较）
  assert(cycles_sent > 5000);         // 应>10us（SPI传输）
}
```

---

## 兼容性验证

### 11. 向后兼容测试
| 测试项 | 说明 | 验证 |
|--------|------|------|
| API 不变 | 所有公开函数签名未改变 | [ ] |
| 行为一致 | 相同输入得到相同输出 | [ ] |
| 边界条件 | 参数钳位行为不变 | [ ] |
| 错误处理 | SPI 失败后的重试机制正常 | [ ] |

---

### 12. 并发安全测试（如适用）
```c
// 如果从多个任务调用（不推荐，但需验证）
void test_concurrent_access(void)
{
  // 任务1：20ms周期调用 BSP_LED_Task
  // 任务2：随机时刻调用 BSP_LED_SetMode
  // 预期：无死锁、无数据竞争
}
```

**注意**：
- 当前实现**非线程安全**（文档已说明）
- 若需支持多任务，需添加互斥锁

---

## 回归测试

### 13. 已知边界条件
| 场景 | 输入 | 预期行为 | 实测 |
|------|------|----------|------|
| 零饱和度 | `SetSaturation(0)` | 彩色呼吸变为白色呼吸 | [ ] |
| 零周期（非法） | `SetBreathPeriod(0)` | 钳位到200ms | [ ] |
| 极短周期 | `SetColorPeriod(500)` | 钳位到1000ms | [ ] |
| 极长周期 | `SetColorPeriod(60000)` | 钳位到30000ms | [ ] |

---

### 14. 电源测试
| 测试项 | 方法 | 预期 | 实测 |
|--------|------|------|------|
| 功耗无变化 | 用电流表测量 | ±1mA以内 | [ ] |
| 全白最大功耗 | `SetRGB(255,255,255)` | 约60mA（单颗2812） | [ ] |
| 熄灭功耗 | `SetMode(LED_MODE_OFF)` | <1mA | [ ] |

---

### 15. 长时间稳定性
```c
// 压力测试：运行24小时
void stress_test(void)
{
  uint32_t iterations = 0;
  while (1)
  {
    BSP_LED_SetMode(LED_MODE_BREATHE_COLOR, 0, 0, 0);
    HAL_Delay(10000);  // 10秒彩色呼吸
    
    BSP_LED_SetMode(LED_MODE_BLINK, 255, 0, 0);
    HAL_Delay(5000);   // 5秒闪烁
    
    iterations++;
    if (iterations % 100 == 0)
      printf("Stress test: %lu iterations\n", iterations);
  }
}
```

**预期**：
- ✅ 无内存泄漏
- ✅ 无频率漂移
- ✅ 无颜色异常

---

## 特殊场景测试

### 16. 时钟变更测试（可选）
如果项目可能改变系统时钟：
```c
// 1. 默认 80MHz PCLK4
BSP_LED_Init();
BSP_LED_SetRGB(255, 0, 0);  // 验证红色正常

// 2. 改时钟到 100MHz（需修改 RCC 配置）
SystemClock_Config_100MHz();
BSP_LED_DeInit();  // 需添加这个函数
BSP_LED_Init();    // 重新初始化
BSP_LED_SetRGB(255, 0, 0);  // 验证仍然正常

// 预期：自动分频逻辑适配新时钟
```

---

### 17. 多颗灯级联（未来扩展）
```c
// 修改配置
#define WS2812_LED_NUM  4U  // 改为4颗

// 测试代码
void test_multiple_leds(void)
{
  // 当前实现：所有灯显示相同颜色
  BSP_LED_SetRGB(255, 0, 0);
  // 验证：4颗灯都是红色
}
```

**注意**：当前版本不支持单独控制多颗灯，需扩展 `SetRGB` 接口。

---

## 文档验证

### 18. 注释准确性
- [ ] 文件头的版本号已更新（v1.0 → v1.1）
- [ ] 优化说明清晰完整
- [ ] 所有魔法数字都有对应的宏定义
- [ ] 函数注释与实际行为一致

---

### 19. 编译日志检查
```bash
# 检查是否有隐式声明警告
grep -i "implicit declaration" build.log

# 检查是否有未使用变量
grep -i "unused" build.log

# 检查是否有类型转换警告
grep -i "conversion" build.log
```

**预期**：所有警告都应清零。

---

## 验证结果汇总

| 类别 | 通过项 | 总项数 | 通过率 |
|------|--------|--------|--------|
| 编译验证 | ___ / 3 | 3 | ___% |
| 功能验证 | ___ / 6 | 6 | ___% |
| 性能验证 | ___ / 2 | 2 | ___% |
| 兼容性验证 | ___ / 2 | 2 | ___% |
| 回归测试 | ___ / 3 | 3 | ___% |
| 特殊场景 | ___ / 2 | 2 | ___% |
| 文档验证 | ___ / 2 | 2 | ___% |
| **总计** | ___ / 20 | 20 | ___% |

---

## 问题跟踪

| 发现时间 | 问题描述 | 严重程度 | 状态 | 备注 |
|---------|---------|---------|------|------|
| | | | | |

---

## 验证结论

- [ ] **通过**：所有测试通过，可以合并到主分支
- [ ] **有条件通过**：有轻微问题，已记录在问题跟踪表
- [ ] **不通过**：存在严重问题，需修复后重新验证

---

**验证人员**：___________  
**验证日期**：___________  
**验证环境**：
- 硬件：___________
- 编译器：___________
- 优化级别：___________
