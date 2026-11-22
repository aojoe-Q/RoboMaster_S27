/**
 * @file    led_diagnostic_example.c
 * @brief   LED诊断工具使用示例
 * @note    将代码片段集成到你的项目中
 */

/* ========================================================================== */
/* 步骤1：在 bsp_led.c 中插入诊断钩子 */
/* ========================================================================== */

// 在 bsp_led.c 的顶部添加
#include "bsp_led_diagnostic.h"

// 修改 BSP_LED_Task() 函数
void BSP_LED_Task(void)
{
  // ✅ 添加：记录调用时间戳
  LED_Diag_RecordTiming();

  switch (led_mode)
  {
    // ... 原有代码 ...

    case LED_MODE_BREATHE:
    {
      uint8_t scale = LED_Breath_Scale();
      uint8_t r8 = led_r, g8 = led_g, b8 = led_b;

      // ✅ 添加：记录相位和亮度（可选，高频输出）
      LED_Diag_LogPhase(breath_phase, 0, scale);

      LED_ApplyBrightness(scale, &r8, &g8, &b8);
      LED_UpdateRGB(r8, g8, b8);
      break;
    }

    // ... 其他 case ...
  }
}

// 修改 LED_UpdateRGB() 函数
static void LED_UpdateRGB(uint8_t r, uint8_t g, uint8_t b)
{
  uint32_t current_rgb = RGB_Pack(r, g, b);

  if ((!last_rgb_valid) || (current_rgb != last_rgb_packed))
  {
    // ✅ 添加：记录发送
    LED_Diag_RecordSend();
    BSP_LED_SetRGB(r, g, b);
  }
  else
  {
    // ✅ 添加：记录跳过
    LED_Diag_RecordSkip();
  }
}

// 修改 BSP_LED_SetRGB() 函数
void BSP_LED_SetRGB(uint8_t r, uint8_t g, uint8_t b)
{
  WS2812_Pack(ws_buf, g, r, b);

  last_rgb_valid = 0U;
  HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi6, ws_buf, (uint16_t)WS2812_BUF_SIZE, 10U);

  if (status == HAL_OK)
  {
    // ✅ 添加：记录SPI成功
    LED_Diag_RecordSPISuccess();

    last_rgb_packed = RGB_Pack(r, g, b);
    last_rgb_valid  = 1U;
  }
  else
  {
    // ✅ 添加：记录SPI失败
    LED_Diag_RecordSPIFail();
  }
}

/* ========================================================================== */
/* 步骤2：在 main.c 中添加定期打印 */
/* ========================================================================== */

#include "bsp_led_diagnostic.h"

void test_led_with_diagnostics(void)
{
  BSP_LED_Init();
  BSP_LED_SetMode(LED_MODE_BREATHE, 0, 40, 255);
  BSP_LED_SetBreathPeriod(1500);

  uint32_t last_print = HAL_GetTick();

  while (1)
  {
    BSP_LED_Task();
    HAL_Delay(20);  // 20ms 周期

    // 每10秒打印一次统计
    uint32_t now = HAL_GetTick();
    if (now - last_print >= 10000)
    {
      LED_Diag_PrintAll();
      LED_Diag_ResetAll();  // 重置计数器，观察下一个10秒窗口
      last_print = now;
    }
  }
}

/* ========================================================================== */
/* 步骤3：在 FreeRTOS 任务中使用 */
/* ========================================================================== */

void LED_Task(void *argument)
{
  BSP_LED_Init();
  BSP_LED_SetMode(LED_MODE_BREATHE_COLOR, 0, 0, 0);

  uint32_t last_print = 0;

  for (;;)
  {
    BSP_LED_Task();

    // 每500帧（10秒）打印一次
    if (++last_print >= 500)
    {
      LED_Diag_PrintAll();
      LED_Diag_ResetAll();
      last_print = 0;
    }

    osDelay(20);  // 20ms 周期
  }
}

/* ========================================================================== */
/* 步骤4：解读输出 */
/* ========================================================================== */

/*
正常输出示例：
========== LED Diagnostics ==========
[LED Timing] Calls: 500, Interval: min=19, avg=20, max=21 ms, Jitter: 2.4%
[LED Dedup] Skip: 35, Send: 465, Skip Rate: 7.0%
[LED SPI] Success: 465, Fail: 0 (0.000%)
=====================================

解读：
✅ Timing: 平均20ms，抖动2.4%（正常，FreeRTOS调度开销）
✅ Dedup: Skip率7%（正常，呼吸慢速变化时会有重复帧）
✅ SPI: 无失败（正常）


异常输出示例1：任务调度抖动大
========== LED Diagnostics ==========
[LED Timing] Calls: 500, Interval: min=15, avg=20, max=38 ms, Jitter: 18.2%
[LED Dedup] Skip: 42, Send: 458, Skip Rate: 8.4%
[LED SPI] Success: 458, Fail: 0 (0.000%)
=====================================

问题：
❌ max=38ms 远超期望的22ms，说明任务被长时间阻塞
❌ Jitter 18.2% 过高

可能原因：
- 其他高优先级任务抢占CPU
- 中断处理时间过长
- FreeRTOS时间片设置不当

解决：
- 提高LED任务优先级
- 检查中断服务程序（ISR）是否有耗时操作
- 使用硬件定时器触发LED任务


异常输出示例2：SPI传输失败
========== LED Diagnostics ==========
[LED Timing] Calls: 500, Interval: min=19, avg=20, max=21 ms, Jitter: 2.1%
[LED Dedup] Skip: 28, Send: 472, Skip Rate: 5.6%
[LED SPI] Success: 467, Fail: 5 (1.059%)
=====================================

问题：
❌ SPI失败5次，失败率1%

可能原因：
- SPI时钟配置错误（频率不在 2.1~3.1MHz 范围）
- PA7引脚有硬件冲突（被其他外设占用）
- 电磁干扰（EMI）
- 供电不稳定

解决：
- 用示波器测量PA7的波形，确认SPI时序
- 检查原理图，确认PA7无复用
- 添加去耦电容
- 降低SPI传输超时时间（当前10ms可能太短）


异常输出示例3：去重率异常高
========== LED Diagnostics ==========
[LED Timing] Calls: 500, Interval: min=19, avg=20, max=21 ms, Jitter: 2.3%
[LED Dedup] Skip: 485, Send: 15, Skip Rate: 97.0%
[LED SPI] Success: 15, Fail: 0 (0.000%)
=====================================

问题：
❌ Skip率97%，说明几乎所有帧都被去重（RGB值不变）

可能原因：
- 呼吸周期设置过长（例如 60000ms）
- 相位推进步长太小
- LED_AdvancePhase() 计算有bug

解决：
- 检查 breath_period 和 color_period 的值
- 添加相位日志：#define LED_DIAG_ENABLE_PHASE_LOG 1
- 如果phase一直不变，说明推进逻辑有问题


异常输出示例4：去重率为0
========== LED Diagnostics ==========
[LED Timing] Calls: 500, Interval: min=19, avg=20, max=21 ms, Jitter: 2.2%
[LED Dedup] Skip: 0, Send: 500, Skip Rate: 0.0%
[LED SPI] Success: 500, Fail: 0 (0.000%)
=====================================

问题：
❌ Skip率0%，说明每一帧的RGB都不同（去重失效）

可能原因：
- RGB_Pack() 函数有bug
- last_rgb_packed 没有正确保存
- 编译器优化问题（volatile 关键字缺失）

解决：
- 检查 RGB_Pack() 和 LED_UpdateRGB() 的实现
- 用调试器单步执行，查看 last_rgb_packed 的值
- 尝试添加 volatile：static volatile uint32_t last_rgb_packed;

*/

/* ========================================================================== */
/* 步骤5：压力测试 */
/* ========================================================================== */

void stress_test_with_diagnostics(void)
{
  BSP_LED_Init();

  printf("Starting LED stress test...\n");
  printf("Test 1: Fast breathing (200ms period)\n");
  BSP_LED_SetMode(LED_MODE_BREATHE, 255, 0, 0);
  BSP_LED_SetBreathPeriod(200);

  for (int i = 0; i < 50; i++) {  // 10秒（50次 * 200ms）
    BSP_LED_Task();
    HAL_Delay(20);
  }
  LED_Diag_PrintAll();
  LED_Diag_ResetAll();

  printf("\nTest 2: Slow breathing (10000ms period)\n");
  BSP_LED_SetBreathPeriod(10000);

  for (int i = 0; i < 500; i++) {  // 10秒
    BSP_LED_Task();
    HAL_Delay(20);
  }
  LED_Diag_PrintAll();
  LED_Diag_ResetAll();

  printf("\nTest 3: Color breathing\n");
  BSP_LED_SetMode(LED_MODE_BREATHE_COLOR, 0, 0, 0);

  for (int i = 0; i < 500; i++) {  // 10秒
    BSP_LED_Task();
    HAL_Delay(20);
  }
  LED_Diag_PrintAll();
  LED_Diag_ResetAll();

  printf("\nStress test completed!\n");
}

/* ========================================================================== */
/* 步骤6：发布前清理 */
/* ========================================================================== */

/*
在 bsp_led_diagnostic.h 中，将所有开关改为 0：

#define LED_DIAG_ENABLE_TIMING      0
#define LED_DIAG_ENABLE_DEDUP_STAT  0
#define LED_DIAG_ENABLE_SPI_STAT    0
#define LED_DIAG_ENABLE_PHASE_LOG   0

编译器会优化掉所有诊断代码（inline函数展开后为空操作），
对性能和代码体积零影响。
*/
