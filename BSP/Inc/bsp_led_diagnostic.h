/**
 * @file    bsp_led_diagnostic.h
 * @brief   LED驱动诊断工具 — 用于排查亮度跳变问题
 * @note    在 bsp_led.c 中 #include 此文件，并在 main.c 中调用诊断函数
 */

#ifndef BSP_LED_DIAGNOSTIC_H
#define BSP_LED_DIAGNOSTIC_H

#include <stdint.h>
#include <stdio.h>

/* 诊断开关：编译时控制，发布版本全部关闭 */
#define LED_DIAG_ENABLE_TIMING      1  /* 任务调度时间戳检测 */
#define LED_DIAG_ENABLE_DEDUP_STAT  1  /* RGB去重统计 */
#define LED_DIAG_ENABLE_SPI_STAT    1  /* SPI传输失败统计 */
#define LED_DIAG_ENABLE_PHASE_LOG   0  /* 相位变化日志（高频输出，仅调试用）*/

/* ========================================================================== */
/* 1. 任务调度抖动检测 */
/* ========================================================================== */
#if LED_DIAG_ENABLE_TIMING

typedef struct {
  uint32_t last_tick;
  uint32_t min_interval;
  uint32_t max_interval;
  uint32_t sum_interval;
  uint32_t call_count;
  uint32_t jitter_count;  /* 超出 ±2ms 的次数 */
} LED_Timing_Stat_t;

static LED_Timing_Stat_t timing_stat = {
  .last_tick = 0,
  .min_interval = 0xFFFFFFFF,
  .max_interval = 0,
  .sum_interval = 0,
  .call_count = 0,
  .jitter_count = 0
};

/**
 * @brief  在 BSP_LED_Task() 开头调用
 */
static inline void LED_Diag_RecordTiming(void)
{
  extern uint32_t HAL_GetTick(void);
  uint32_t now = HAL_GetTick();

  if (timing_stat.call_count > 0) {
    uint32_t delta = now - timing_stat.last_tick;

    if (delta < timing_stat.min_interval) timing_stat.min_interval = delta;
    if (delta > timing_stat.max_interval) timing_stat.max_interval = delta;
    timing_stat.sum_interval += delta;

    /* 检测抖动：期望 20ms ± 2ms */
    if (delta < 18 || delta > 22) {
      timing_stat.jitter_count++;
    }
  }

  timing_stat.last_tick = now;
  timing_stat.call_count++;
}

/**
 * @brief  打印调度统计（建议每10秒调用一次）
 */
static inline void LED_Diag_PrintTiming(void)
{
  if (timing_stat.call_count == 0) return;

  uint32_t avg = timing_stat.sum_interval / timing_stat.call_count;
  float jitter_rate = 100.0f * timing_stat.jitter_count / timing_stat.call_count;

  printf("[LED Timing] Calls: %lu, Interval: min=%lu, avg=%lu, max=%lu ms, Jitter: %.1f%%\n",
         timing_stat.call_count,
         timing_stat.min_interval,
         avg,
         timing_stat.max_interval,
         jitter_rate);
}

#else
  #define LED_Diag_RecordTiming()      ((void)0)
  #define LED_Diag_PrintTiming()       ((void)0)
#endif

/* ========================================================================== */
/* 2. RGB去重统计 */
/* ========================================================================== */
#if LED_DIAG_ENABLE_DEDUP_STAT

typedef struct {
  uint32_t skip_count;
  uint32_t send_count;
} LED_Dedup_Stat_t;

static LED_Dedup_Stat_t dedup_stat = {0, 0};

static inline void LED_Diag_RecordSkip(void)
{
  dedup_stat.skip_count++;
}

static inline void LED_Diag_RecordSend(void)
{
  dedup_stat.send_count++;
}

/**
 * @brief  打印去重统计
 */
static inline void LED_Diag_PrintDedup(void)
{
  uint32_t total = dedup_stat.skip_count + dedup_stat.send_count;
  if (total == 0) return;

  float skip_rate = 100.0f * dedup_stat.skip_count / total;

  printf("[LED Dedup] Skip: %lu, Send: %lu, Skip Rate: %.1f%%\n",
         dedup_stat.skip_count,
         dedup_stat.send_count,
         skip_rate);
}

#else
  #define LED_Diag_RecordSkip()        ((void)0)
  #define LED_Diag_RecordSend()        ((void)0)
  #define LED_Diag_PrintDedup()        ((void)0)
#endif

/* ========================================================================== */
/* 3. SPI传输失败统计 */
/* ========================================================================== */
#if LED_DIAG_ENABLE_SPI_STAT

typedef struct {
  uint32_t success_count;
  uint32_t fail_count;
} LED_SPI_Stat_t;

static LED_SPI_Stat_t spi_stat = {0, 0};

static inline void LED_Diag_RecordSPISuccess(void)
{
  spi_stat.success_count++;
}

static inline void LED_Diag_RecordSPIFail(void)
{
  spi_stat.fail_count++;

  /* 立即告警（SPI失败很罕见） */
  if (spi_stat.fail_count <= 10) {
    printf("[LED SPI] ⚠️ Transmission failed! Total failures: %lu\n",
           spi_stat.fail_count);
  }
}

/**
 * @brief  打印SPI统计
 */
static inline void LED_Diag_PrintSPI(void)
{
  uint32_t total = spi_stat.success_count + spi_stat.fail_count;
  if (total == 0) return;

  float fail_rate = 100.0f * spi_stat.fail_count / total;

  printf("[LED SPI] Success: %lu, Fail: %lu (%.3f%%)\n",
         spi_stat.success_count,
         spi_stat.fail_count,
         fail_rate);
}

#else
  #define LED_Diag_RecordSPISuccess()  ((void)0)
  #define LED_Diag_RecordSPIFail()     ((void)0)
  #define LED_Diag_PrintSPI()          ((void)0)
#endif

/* ========================================================================== */
/* 4. 相位变化日志（高频输出，仅调试时启用）*/
/* ========================================================================== */
#if LED_DIAG_ENABLE_PHASE_LOG

static inline void LED_Diag_LogPhase(uint16_t breath_phase, uint16_t color_phase, uint8_t scale)
{
  printf("[LED Phase] Breath: %u, Color: %u, Scale: %u\n",
         breath_phase, color_phase, scale);
}

#else
  #define LED_Diag_LogPhase(bp, cp, sc)  ((void)0)
#endif

/* ========================================================================== */
/* 统一接口 */
/* ========================================================================== */

/**
 * @brief  打印所有诊断统计（建议每10秒调用一次）
 */
static inline void LED_Diag_PrintAll(void)
{
  printf("\n========== LED Diagnostics ==========\n");
  LED_Diag_PrintTiming();
  LED_Diag_PrintDedup();
  LED_Diag_PrintSPI();
  printf("=====================================\n\n");
}

/**
 * @brief  重置所有统计计数器
 */
static inline void LED_Diag_ResetAll(void)
{
#if LED_DIAG_ENABLE_TIMING
  timing_stat.min_interval = 0xFFFFFFFF;
  timing_stat.max_interval = 0;
  timing_stat.sum_interval = 0;
  timing_stat.call_count = 0;
  timing_stat.jitter_count = 0;
#endif

#if LED_DIAG_ENABLE_DEDUP_STAT
  dedup_stat.skip_count = 0;
  dedup_stat.send_count = 0;
#endif

#if LED_DIAG_ENABLE_SPI_STAT
  spi_stat.success_count = 0;
  spi_stat.fail_count = 0;
#endif
}

#endif /* BSP_LED_DIAGNOSTIC_H */
