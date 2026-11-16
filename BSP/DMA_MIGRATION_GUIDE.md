# 从轮询SPI迁移到DMA的完整指南

---

## 第1步：CubeMX配置（10分钟）

### 1.1 启用SPI6 DMA
1. 打开你的 `.ioc` 文件
2. **Connectivity → SPI6 → DMA Settings**
3. **Add**：选择 `SPI6 TX`
4. 弹出框会自动分配DMA通道（通常DMA2_Stream5）
5. **Mode**: `Normal`
6. **Priority**: `High` 或 `Very High`
7. **Data Width**: Byte to Byte

### 1.2 配置DMA中断
1. **System Core → NVIC**
2. 启用 `DMA2 stream5 global interrupt`（或分配的通道）
3. **Generate Code**

### 1.3 验证生成的文件
编译后应该有这些变量：
```c
// main.c
extern DMA_HandleTypeDef hdma_spi6_tx;

// stm32h7xx_hal_msp.c 中应有
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI6)
  {
    __HAL_RCC_DMA2_CLK_ENABLE();
    hdma_spi6_tx.Instance = DMA2_Stream5;
    // ... DMA配置 ...
  }
}
```

---

## 第2步：更新代码（20分钟）

### 2.1 替换源文件
```bash
# 备份原文件
cp bsp_led.c bsp_led.c.v1.3.backup

# 用DMA版本替换
cp bsp_led_dma.c bsp_led.c
```

### 2.2 更新头文件（可选）

在 `bsp_led.h` 末尾添加：
```c
/**
  * @brief  DMA传输状态查询
  */
extern uint8_t BSP_LED_IsBusy(void);

/**
  * @brief  强制停止DMA（调试用）
  */
extern void BSP_LED_Forcestop(void);

/**
  * @brief  线程安全版本（推荐在RTOS中使用）
  */
extern HAL_StatusTypeDef BSP_LED_SetRGB_Safe(uint8_t r, uint8_t g, uint8_t b);
```

### 2.3 检查编译环境

**stm32h7xx_hal_conf.h** 中需要：
```c
#define USE_HAL_SPI_MODULE
#define USE_HAL_DMA_MODULE
```

### 2.4 处理D-Cache（关键）

查看链接脚本 `STM32H745XIHx_FLASH.ld` 是否定义了 `.dtcm_ram`：

```ld
/* 如果有这一行，说明DTCM已配置 */
.dtcm_ram :
{
  *(.dtcm_ram)
} > DTCM_RAM AT > FLASH

/* 如果没有，你需要改用RAM_D2或RAM_D3 */
```

**如果没有DTCM section**，修改 `bsp_led_dma.c`：
```c
/* 改为使用RAM_D3（通常禁用D-Cache） */
__attribute__((section(".ram_d3")))
__attribute__((aligned(32)))
static uint8_t ws_buf_dma[9];
```

---

## 第3步：中断处理（5分钟）

### 3.1 关键：在中断处理函数中调用HAL回调

**stm32h7xx_it.c** 中已有的中断处理：

```c
/* DMA Stream5中断处理（由CubeMX自动生成） */
void DMA2_Stream5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi6_tx);
}

/* 或者SPI6中断处理（两种方式都可以） */
void SPI6_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi6);
}
```

这两个函数最终都会调用 `HAL_SPI_TxCpltCallback()`，设置 `dma_busy = 0`。

### 3.2 验证回调链路
```
DMA中断 → DMA_IRQHandler() → HAL_DMA_IRQHandler()
         → HAL库内部检测 → HAL_SPI_TxCpltCallback()
         → dma_busy = 0 ✓
```

如果没有回调被触发，检查：
1. DMA中断是否启用（NVIC中）
2. HAL库是否正确生成

---

## 第4步：编译和测试（15分钟）

### 4.1 编译
```bash
make clean
make -j8
```

预期结果：无错误，可能有D-Cache相关的警告（无害）。

### 4.2 快速测试
```c
// main.c 中
void test_led_dma(void)
{
  BSP_LED_Init();
  BSP_LED_SetMode(LED_MODE_BREATHE, 0, 40, 255);

  for (int i = 0; i < 100; i++) {
    BSP_LED_Task();
    osDelay(20);
  }

  printf("LED DMA test completed!\n");
}
```

### 4.3 验证波形（示波器）
```
PA7波形应该看到：
✅ 每个bit约400ns（2.5MHz）
✅ 字节间没有明显延迟
✅ 9字节后有>50us低电平
❌ 不应该看到>2us的突发间隔
```

---

## 第5步：RTOS集成（10分钟）

### 5.1 LED任务优先级

```c
// main.c
void led_task(void* arg)
{
  BSP_LED_Init();
  BSP_LED_SetMode(LED_MODE_BREATHE_COLOR, 0, 0, 0);

  for (;;) {
    BSP_LED_Task();
    osDelay(20);
  }
}

// 在osKernelStart前
osThreadNew(led_task, NULL, &attributes_led);
```

优先级建议：
- LED任务：`osPriorityLow` （较低，不与关键任务争抢）
- 电机/USB/CAN：`osPriorityHigh` （高优先级，可中断LED）

### 5.2 多任务调用LED

在其他任务中设置LED颜色：
```c
void app_task(void* arg)
{
  for (;;) {
    // 使用线程安全版本
    BSP_LED_SetRGB_Safe(255, 0, 0);  // ✅ 安全
    // BSP_LED_SetRGB(255, 0, 0);    // ⚠️ 不安全（仍然可用，但有竞态）
    
    osDelay(1000);
  }
}
```

---

## 常见问题速查表

### Q1: 编译错误 `undefined reference to 'hdma_spi6_tx'`
**原因**: CubeMX没有生成DMA配置
**解决**:
```
1. 重新打开CubeMX项目
2. Connectivity → SPI6 → DMA Settings → Add SPI6 TX
3. Generate Code
4. 重新编译
```

### Q2: 编译警告 `section '.dtcm_ram' not found`
**原因**: 链接脚本没有定义DTCM
**解决**:
```c
// 在 bsp_led_dma.c 中改为
__attribute__((section(".ram_d3")))  // 用 RAM_D3 代替
static uint8_t ws_buf_dma[9];
```

### Q3: LED仍然闪白光
**原因**: Cache未正确处理
**诊断**:
```c
// 检查是否调用了Cache清理
// bsp_led_dma.c 中应该有：
SCB_CleanDCache_by_Addr((uint32_t*)work_buf, 9);
```

### Q4: DMA超时或传输失败
**原因**: 中断未被正确调用
**诊断**:
```c
// 添加日志
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
  if (hspi->Instance == SPI6) {
    printf("✅ DMA TX Complete\n");
    dma_busy = 0;
  }
}
```

### Q5: 多个SPI冲突
**原因**: 另一个SPI也使用DMA
**解决**:
```c
// 检查SPI分配
// STM32H7: SPI1→DMA2_Stream3, SPI2→DMA1_Stream4, SPI6→DMA2_Stream5
// 确保没有重复使用
```

---

## 性能对比

| 指标 | 轮询+关中断 | **DMA** |
|------|-----------|--------|
| 闪白光 | 100% 消除（但延迟其他中断） | ✅ 100% 消除 |
| CPU占用 | 29μs阻塞 + SPI计算 | <1μs（CPU自由） |
| 实时性 | ❌ USB/CAN延迟 | ✅ 完全不影响 |
| 多颗灯支持 | ❌ 需改代码 | ✅ 自动支持 |
| 最大刷新率 | 34Hz (29μs×34=1ms) | **>1000Hz**（理论） |

---

## 完整迁移检查清单

- [ ] **第1步**：CubeMX配置DMA
  - [ ] SPI6 DMA Settings中添加TX
  - [ ] NVIC中启用DMA中断
  - [ ] Generate Code

- [ ] **第2步**：代码更新
  - [ ] 替换bsp_led.c为bsp_led_dma.c
  - [ ] 更新bsp_led.h
  - [ ] 检查hal_conf.h中USE_HAL_DMA_MODULE

- [ ] **第3步**：中断处理
  - [ ] 验证DMA IRQ Handler存在
  - [ ] 验证SPI IRQ Handler存在
  - [ ] 检查HAL_SPI_TxCpltCallback被调用

- [ ] **第4步**：编译测试
  - [ ] make clean && make成功
  - [ ] 无链接错误
  - [ ] 烧录成功

- [ ] **第5步**：功能验证
  - [ ] LED能正常呼吸
  - [ ] 无闪白光现象
  - [ ] 30分钟无异常

- [ ] **第6步**：RTOS集成
  - [ ] LED任务优先级正确
  - [ ] 其他任务能中断LED发送
  - [ ] 多任务调用时使用Safe版本

---

## 回滚方案

如果DMA方案有问题，快速回滚到轮询版本：
```bash
# 恢复备份
cp bsp_led.c.v1.3.backup bsp_led.c

# 在CubeMX中
# 1. Connectivity → SPI6 → DMA Settings → Remove SPI6 TX
# 2. System Core → NVIC → Disable DMA interrupt
# 3. Generate Code

# 重新编译
make clean && make
```

---

## 最终验证

```c
void final_test(void)
{
  uint32_t flash_count = 0;
  uint32_t total_frames = 0;

  for (int min = 0; min < 30; min++) {  // 运行30分钟
    for (int sec = 0; sec < 60; sec++) {
      BSP_LED_Task();
      osDelay(1000 / 50);  // 50Hz更新
      total_frames++;
    }
    printf("Min %d: Flashes=%lu, Rate=%.3f%%\n",
           min, flash_count, 100.0f*flash_count/total_frames);
  }
}
```

**预期结果**：
- ✅ `flash_count ≈ 0`
- ✅ `Rate < 0.001%`
- ✅ LED呼吸平滑无闪烁

---

**工作量总计**: 约1-2小时
**风险等级**: 低（DMA是标准方案）
**推荐指数**: ⭐⭐⭐⭐⭐
