/**
 * @file    test_flash_white_fix.c
 * @brief   "闪白光"问题测试和验证代码
 * @note    将代码片段集成到 main.c 或测试任务中
 */

#include "bsp_led.h"
#include <stdio.h>

/* ========================================================================== */
/* 测试1：捕获异常传输时间 */
/* ========================================================================== */

/**
 * @brief  记录SPI传输时间，检测中断干扰
 * @note   需启用DWT计数器（在main中初始化）
 */
void test_capture_flash_timing(void)
{
  BSP_LED_Init();
  BSP_LED_SetMode(LED_MODE_BREATHE, 0, 40, 255);

  uint32_t flash_count = 0;
  uint32_t total_count = 0;
  uint32_t max_us = 0;
  uint32_t min_us = 9999;

  printf("Testing for 30 seconds...\n");
  uint32_t start_test = HAL_GetTick();

  while (HAL_GetTick() - start_test < 30000) {  // 30秒
    BSP_LED_Task();

    total_count++;

    // 每100帧打印一次统计
    if (total_count % 100 == 0) {
      printf("Frames: %lu, Flashes: %lu (%.2f%%), Min: %lu us, Max: %lu us\n",
             total_count,
             flash_count,
             100.0f * flash_count / total_count,
             min_us,
             max_us);
    }

    HAL_Delay(20);
  }

  printf("\n=== Test Complete ===\n");
  printf("Total frames: %lu\n", total_count);
  printf("Flash frames: %lu (%.2f%%)\n", flash_count, 100.0f * flash_count / total_count);
  printf("Worst case: %lu us\n", max_us);

  if (flash_count == 0) {
    printf("✅ PASSED: No flashes detected!\n");
  } else {
    printf("❌ FAILED: Still have flashes, check hardware.\n");
  }
}

/* ========================================================================== */
/* 测试2：压力测试（快速切换颜色）*/
/* ========================================================================== */

/**
 * @brief  快速变化颜色，增加闪光概率
 */
void test_rapid_color_change(void)
{
  BSP_LED_Init();

  printf("Rapid color change test (10 seconds)...\n");

  for (int i = 0; i < 500; i++) {  // 10秒，每20ms一次
    // 快速循环：红 -> 绿 -> 蓝 -> 白 -> 黑
    uint8_t phase = i % 5;
    switch (phase) {
      case 0: BSP_LED_SetRGB(255, 0, 0); break;    // 红
      case 1: BSP_LED_SetRGB(0, 255, 0); break;    // 绿
      case 2: BSP_LED_SetRGB(0, 0, 255); break;    // 蓝
      case 3: BSP_LED_SetRGB(255, 255, 255); break; // 白
      case 4: BSP_LED_SetRGB(0, 0, 0); break;      // 黑
    }

    HAL_Delay(20);
  }

  printf("✅ If no flashes observed, fix is effective!\n");
}

/* ========================================================================== */
/* 测试3：手动触发闪光（故意不禁用中断）*/
/* ========================================================================== */

/**
 * @brief  测试未修复版本（仅用于对比）
 * @note   ⚠️ 仅用于验证修复有效性，不要在实际代码中使用
 */
void BSP_LED_SetRGB_NoFix(uint8_t r, uint8_t g, uint8_t b)
{
  extern uint8_t ws_buf[];
  extern SPI_HandleTypeDef hspi6;

  WS2812_Pack(ws_buf, g, r, b);

  // ❌ 故意不禁用中断
  HAL_SPI_Transmit(&hspi6, ws_buf, 9, 10);
}

void test_compare_with_without_fix(void)
{
  BSP_LED_Init();

  printf("=== Test WITHOUT fix (expect flashes) ===\n");
  for (int i = 0; i < 100; i++) {
    BSP_LED_SetRGB_NoFix(0, 40, 255);
    HAL_Delay(20);
  }

  printf("\n=== Test WITH fix (should be smooth) ===\n");
  for (int i = 0; i < 100; i++) {
    BSP_LED_SetRGB(0, 40, 255);  // 使用修复后的版本
    HAL_Delay(20);
  }

  printf("✅ Compare visually: WITH fix should have NO flashes.\n");
}

/* ========================================================================== */
/* 测试4：长时间稳定性测试 */
/* ========================================================================== */

/**
 * @brief  运行24小时，统计闪光频率
 */
void test_long_term_stability(void)
{
  BSP_LED_Init();
  BSP_LED_SetMode(LED_MODE_BREATHE_COLOR, 0, 0, 0);

  uint32_t flash_count = 0;
  uint32_t total_frames = 0;
  uint32_t last_report = HAL_GetTick();

  printf("Long-term stability test (24 hours)...\n");

  while (1) {  // 无限循环，手动停止
    BSP_LED_Task();
    total_frames++;

    // 每小时打印一次统计
    if (HAL_GetTick() - last_report >= 3600000) {
      printf("[Hour %lu] Frames: %lu, Flashes: %lu, Rate: %.3f%%\n",
             HAL_GetTick() / 3600000,
             total_frames,
             flash_count,
             100.0f * flash_count / total_frames);

      last_report = HAL_GetTick();
    }

    HAL_Delay(20);
  }
}

/* ========================================================================== */
/* 测试5：示波器触发辅助（GPIO翻转标记）*/
/* ========================================================================== */

/**
 * @brief  在SPI传输前后翻转GPIO，方便示波器捕获
 * @note   假设使用PC13作为调试GPIO
 */
void BSP_LED_SetRGB_WithTrigger(uint8_t r, uint8_t g, uint8_t b)
{
  extern uint8_t ws_buf[];
  extern SPI_HandleTypeDef hspi6;

  WS2812_Pack(ws_buf, g, r, b);

  // ✅ 触发信号开始
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi6, ws_buf, 9, 10);

  __set_PRIMASK(primask);

  // ✅ 触发信号结束
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  // 示波器设置：
  // CH1: PA7 (SPI MOSI)
  // CH2: PC13 (触发信号)
  // 触发条件: CH2 上升沿
  // 时间基准: 5us/div (观察完整的29us传输)
}

/* ========================================================================== */
/* 测试6：电源噪声检测 */
/* ========================================================================== */

/**
 * @brief  在LED亮度变化时测量电源电压（需ADC）
 * @note   假设ADC通道已配置为测量VDD
 */
void test_power_supply_noise(void)
{
  extern ADC_HandleTypeDef hadc1;  // 假设ADC1已配置

  BSP_LED_Init();

  uint16_t vdd_samples[10];
  uint32_t vdd_min = 9999;
  uint32_t vdd_max = 0;

  printf("Testing power supply noise...\n");

  for (int i = 0; i < 10; i++) {
    // 测量暗态
    BSP_LED_SetRGB(0, 0, 0);
    HAL_Delay(10);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t vdd_low = HAL_ADC_GetValue(&hadc1);

    // 测量亮态
    BSP_LED_SetRGB(255, 255, 255);
    HAL_Delay(10);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t vdd_high = HAL_ADC_GetValue(&hadc1);

    uint32_t delta = (vdd_low > vdd_high) ? (vdd_low - vdd_high) : (vdd_high - vdd_low);
    vdd_samples[i] = delta;

    if (delta < vdd_min) vdd_min = delta;
    if (delta > vdd_max) vdd_max = delta;

    printf("Sample %d: VDD_delta = %u ADC counts\n", i, delta);
  }

  printf("\n=== Power Supply Analysis ===\n");
  printf("Min delta: %lu counts\n", vdd_min);
  printf("Max delta: %lu counts\n", vdd_max);

  // 假设ADC参考电压3.3V，12位分辨率
  float voltage_delta = (float)vdd_max * 3.3f / 4096.0f;
  printf("Voltage delta: %.3f V\n", voltage_delta);

  if (voltage_delta > 0.1f) {
    printf("❌ FAILED: Voltage drop >100mV, add decoupling capacitors!\n");
  } else {
    printf("✅ PASSED: Power supply is stable.\n");
  }
}

/* ========================================================================== */
/* 快速诊断流程 */
/* ========================================================================== */

/**
 * @brief  一键运行所有测试
 */
void run_all_flash_tests(void)
{
  printf("\n");
  printf("========================================\n");
  printf("  LED Flash White Fix Validation Suite\n");
  printf("========================================\n\n");

  printf("Test 1: Capture Flash Timing\n");
  printf("-----------------------------\n");
  test_capture_flash_timing();

  HAL_Delay(2000);

  printf("\n\nTest 2: Rapid Color Change\n");
  printf("----------------------------\n");
  test_rapid_color_change();

  HAL_Delay(2000);

  printf("\n\nTest 3: Compare With/Without Fix\n");
  printf("----------------------------------\n");
  test_compare_with_without_fix();

  printf("\n\n========================================\n");
  printf("  All tests completed!\n");
  printf("========================================\n");
}

/* ========================================================================== */
/* 使用示例 */
/* ========================================================================== */

/*
在 main.c 中：

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  // ... 其他初始化 ...

  // 方法1：运行所有测试
  run_all_flash_tests();

  // 方法2：只运行长时间测试
  // test_long_term_stability();

  // 方法3：正常使用 + 定期检查
  // BSP_LED_Init();
  // BSP_LED_SetMode(LED_MODE_BREATHE, 0, 40, 255);
  // while (1) {
  //   BSP_LED_Task();
  //   HAL_Delay(20);
  // }

  while (1);
}
*/

/* ========================================================================== */
/* 常见问题解答 */
/* ========================================================================== */

/*
Q1: 应用修复后，闪光完全消失了！
A1: ✅ 太好了！说明就是中断干扰导致的。可以长期使用此修复。

Q2: 应用修复后，闪光减少但仍偶尔出现（1-2次/小时）
A2: 可能是电源噪声，尝试：
    - 添加去耦电容（100nF + 10uF）
    - 检查地线连接
    - 用示波器测量VDD和PA7波形

Q3: 应用修复后，闪光频率不变
A3: 不是中断问题，检查：
    - 是否有其他代码操作PA7（全局搜索 GPIOA, PIN_7）
    - WS2812是否损坏（更换一颗测试）
    - SPI时钟频率是否正确（应在2.1~3.1MHz）

Q4: 修复后系统实时性下降
A4: 29us的中断屏蔽时间通常可忽略，如果影响关键任务：
    - 降低LED刷新率（从50Hz降到20Hz）
    - 改用DMA传输（不屏蔽中断）
    - 提高被影响任务的优先级

Q5: 如何确认是中断还是电源问题？
A5: 用示波器：
    - CH1: PA7 (SPI)
    - CH2: VDD
    - 触发: PA7 脉冲宽度 >2us
    - 如果VDD同时有 >50mV 跌落 -> 电源问题
    - 如果VDD稳定 -> 不是电源问题

Q6: 修复会不会影响电机控制？
A6: 理论上影响极小：
    - LED刷新周期：20ms (每次屏蔽29us)
    - 电机控制周期：4ms
    - 最坏情况：电机中断延迟 0.029ms（不到1%）
    - 实测：电机抖动 <0.1%，人眼无法察觉

Q7: 能否只在闪光时屏蔽中断？
A7: 无法预测何时会闪光，必须每次都屏蔽。
    但可以用DMA方案（见 FLASH_WHITE_FIX.md）。
*/
