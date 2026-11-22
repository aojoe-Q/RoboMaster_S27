/**
 * @file    bsp_led_dma_standalone.c
 * @brief   WS2812 LED驱动 - 独立DMA实现（与轮询版本共存）
 * @author  Claude Opus 5.5
 * @date    2025-11-08
 *
 * @note
 *  - 完全独立，不修改原有代码
 *  - 无需替换 bsp_led.c
 *  - 直接替换 BSP_LED_Init、BSP_LED_Task、BSP_LED_SetRGB 调用即可
 *  - 所有原有功能完全保留
 */

#include "main.h"
#include "cmsis_os2.h"
#include <string.h>

/* ========================================================================== */
/* 配置 */
/* ========================================================================== */

#define WS2812_LED_NUM        1U
#define WS2812_BYTES_PER_LED  9U
#define WS2812_BUF_SIZE       (WS2812_LED_NUM * WS2812_BYTES_PER_LED)

/* ========================================================================== */
/* DMA缓冲区 - 处理D-Cache */
/* ========================================================================== */

/* 如果编译错误 ".dtcm_ram not found"，改为 ".ram_d3" */
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  __attribute__((section(".dtcm_ram")))
  __attribute__((aligned(32)))
  static uint8_t ws_buf_dma[WS2812_BUF_SIZE];

  __attribute__((section(".dtcm_ram")))
  __attribute__((aligned(32)))
  static uint8_t ws_buf_work[WS2812_BUF_SIZE];
#else
  __attribute__((aligned(32)))
  static uint8_t ws_buf_dma[WS2812_BUF_SIZE];

  __attribute__((aligned(32)))
  static uint8_t ws_buf_work[WS2812_BUF_SIZE];
#endif

/* DMA状态 */
static volatile uint8_t dma_busy = 0;
static osMutexId_t led_dma_mutex = NULL;

/* SPI和DMA句柄（由CubeMX生成） */
extern SPI_HandleTypeDef hspi6;
extern DMA_HandleTypeDef hdma_spi6_tx;

/* LED状态 */
static LED_Mode_t led_mode      = LED_MODE_BREATHE;
static uint8_t    led_r         = LED_DEFAULT_R;
static uint8_t    led_g         = LED_DEFAULT_G;
static uint8_t    led_b         = LED_DEFAULT_B;
static uint8_t    led_sat       = LED_DEFAULT_SAT;
static uint16_t   breath_period = LED_DEFAULT_PERIOD_MS;
static uint16_t   color_period  = LED_DEFAULT_COLOR_PERIOD_MS;

static uint16_t   breath_phase   = 0U;
static uint16_t   color_phase    = 0U;
static uint32_t   breath_remainder = 0U;
static uint32_t   color_remainder  = 0U;
static uint32_t   blink_tick    = 0U;
static uint8_t    blink_on      = 1U;

/* 呼吸曲线 */
static const uint8_t led_breath_lut[64] =
{
      0,   0,   0,   0,   0,   1,   1,   2,
      4,   6,   9,  14,  19,  26,  34,  44,
     55,  68,  82,  97, 113, 130, 147, 164,
    180, 196, 210, 223, 234, 243, 250, 254,
    255, 254, 250, 243, 234, 223, 210, 196,
    180, 164, 147, 130, 113,  97,  82,  68,
     55,  44,  34,  26,  19,  14,   9,   6,
      4,   2,   1,   1,   0,   0,   0,   0,
};

/* ========================================================================== */
/* SPI编码 */
/* ========================================================================== */

static inline void WS2812_PackByte(uint8_t *buf, uint8_t value)
{
  static const uint16_t symbols[16] =
  {
    0x924U, 0x926U, 0x934U, 0x936U,
    0x9A4U, 0x9A6U, 0x9B4U, 0x9B6U,
    0xD24U, 0xD26U, 0xD34U, 0xD36U,
    0xDA4U, 0xDA6U, 0xDB4U, 0xDB6U
  };
  uint32_t encoded = ((uint32_t)symbols[value >> 4] << 12)
                   | symbols[value & 0x0FU];

  buf[0] = (uint8_t)(encoded >> 16);
  buf[1] = (uint8_t)(encoded >> 8);
  buf[2] = (uint8_t)encoded;
}

static void WS2812_Pack(uint8_t *buf, uint8_t g, uint8_t r, uint8_t b)
{
  WS2812_PackByte(&buf[0], g);
  WS2812_PackByte(&buf[3], r);
  WS2812_PackByte(&buf[6], b);
}

/* ========================================================================== */
/* DMA完成回调 */
/* ========================================================================== */

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
  if (hspi->Instance == SPI6) {
    dma_busy = 0;
  }
}

/* ========================================================================== */
/* 初始化 */
/* ========================================================================== */

/**
  * @brief  初始化LED DMA驱动
  * @note   调用此函数代替原来的 BSP_LED_Init()
  *         会自动配置GPIO和SPI6
  */
void BSP_LED_DMA_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_SPI6_CLK_ENABLE();

  /* PA7 -> SPI6_MOSI */
  GPIO_InitStruct.Pin       = GPIO_PIN_7;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_SPI6;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* SPI6配置 */
  hspi6.Instance               = SPI6;
  hspi6.Init.Mode              = SPI_MODE_MASTER;
  hspi6.Init.Direction         = SPI_DIRECTION_1LINE;
  hspi6.Init.DataSize          = SPI_DATASIZE_8BIT;
  hspi6.Init.CLKPolarity       = SPI_POLARITY_LOW;
  hspi6.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi6.Init.NSS               = SPI_NSS_SOFT;

  /* 自动选择最优分频 */
  uint32_t spi_clk = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI6);
  uint32_t best_br = SPI_BAUDRATEPRESCALER_32;

  if (spi_clk > 0) {
    uint32_t best_err = 0xFFFFFFFFU;
    static const uint32_t br_tab[8] =
    {
      SPI_BAUDRATEPRESCALER_2,   SPI_BAUDRATEPRESCALER_4,
      SPI_BAUDRATEPRESCALER_8,   SPI_BAUDRATEPRESCALER_16,
      SPI_BAUDRATEPRESCALER_32,  SPI_BAUDRATEPRESCALER_64,
      SPI_BAUDRATEPRESCALER_128, SPI_BAUDRATEPRESCALER_256
    };

    for (uint32_t i = 0; i < 8; i++) {
      uint32_t f = spi_clk >> (i + 1);
      uint32_t err = (f > 2500000) ? (f - 2500000) : (2500000 - f);
      if (err < best_err) {
        best_err = err;
        best_br = br_tab[i];
      }
    }
  }

  hspi6.Init.BaudRatePrescaler = best_br;
  hspi6.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi6.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi6.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi6.Init.CRCPolynomial     = 0U;
  hspi6.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
  hspi6.Init.NSSPolarity       = SPI_NSS_POLARITY_LOW;
  hspi6.Init.FifoThreshold     = SPI_FIFO_THRESHOLD_01DATA;
  hspi6.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.MasterSSIdleness          = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi6.Init.MasterInterDataIdleness   = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi6.Init.MasterReceiverAutoSusp    = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi6.Init.MasterKeepIOState         = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi6.Init.IOSwap                    = SPI_IO_SWAP_DISABLE;

  if (HAL_SPI_Init(&hspi6) != HAL_OK) {
    Error_Handler();
  }

  /* 链接DMA */
  __HAL_LINKDMA(&hspi6, hdmatx, hdma_spi6_tx);

  /* 创建互斥锁 */
  led_dma_mutex = osMutexNew(NULL);
  if (led_dma_mutex == NULL) {
    Error_Handler();
  }

  /* 初始发送全0 */
  memset(ws_buf_dma, 0, WS2812_BUF_SIZE);
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  SCB_CleanDCache_by_Addr((uint32_t*)ws_buf_dma, WS2812_BUF_SIZE);
#endif
  dma_busy = 1;
  HAL_SPI_Transmit_DMA(&hspi6, ws_buf_dma, WS2812_BUF_SIZE);
}

/* ========================================================================== */
/* 核心发送函数 */
/* ========================================================================== */

/**
  * @brief  发送RGB颜色（DMA版本）
  * @param  r/g/b 颜色分量
  * @retval HAL_OK 成功，HAL_TIMEOUT 超时
  */
HAL_StatusTypeDef BSP_LED_SetRGB_DMA(uint8_t r, uint8_t g, uint8_t b)
{
  /* 获取互斥锁 */
  if (osMutexAcquire(led_dma_mutex, 100) != osOK) {
    return HAL_TIMEOUT;
  }

  /* 等待上一次DMA完成 */
  uint32_t wait_ms = 0;
  while (dma_busy && wait_ms < 100) {
    osDelay(1);
    wait_ms++;
  }

  if (dma_busy) {
    HAL_SPI_DMAStop(&hspi6);
    dma_busy = 0;
  }

  /* 在work缓冲区打包RGB */
  WS2812_Pack(ws_buf_work, g, r, b);

  /* D-Cache处理 */
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  SCB_CleanDCache_by_Addr((uint32_t*)ws_buf_work, WS2812_BUF_SIZE);
#endif

  /* 交换缓冲区 */
  uint8_t* tmp = ws_buf_dma;
  ws_buf_dma = ws_buf_work;
  ws_buf_work = tmp;

  /* 启动DMA */
  dma_busy = 1;
  HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(&hspi6, ws_buf_dma, WS2812_BUF_SIZE);

  osMutexRelease(led_dma_mutex);

  return status;
}

/* ========================================================================== */
/* LED效果计算 */
/* ========================================================================== */

static inline uint16_t LED_AdvancePhase(uint16_t phase, uint16_t period, uint32_t *remainder)
{
  uint32_t numerator = 65536UL * (uint32_t)LED_TASK_PERIOD_MS + *remainder;
  uint32_t step = numerator / period;
  *remainder = numerator % period;
  return (uint16_t)(phase + step);
}

static uint8_t LED_Breath_Scale(void)
{
  breath_phase = LED_AdvancePhase(breath_phase, breath_period, &breath_remainder);

  uint32_t index    = breath_phase >> 10;
  uint32_t fraction = breath_phase & 0x03FFU;

  return (uint8_t)(
      ((uint32_t)led_breath_lut[index] * (1024U - fraction)
     + (uint32_t)led_breath_lut[(index + 1U) & 63U] * fraction
     + 512U) >> 10
  );
}

static inline void LED_ApplyBrightness(uint8_t scale, uint8_t *r, uint8_t *g, uint8_t *b)
{
  *r = (uint8_t)(((uint32_t)*r * scale + 127U) / 255U);
  *g = (uint8_t)(((uint32_t)*g * scale + 127U) / 255U);
  *b = (uint8_t)(((uint32_t)*b * scale + 127U) / 255U);
}

static void LED_HSVToRGB(uint8_t hue, uint8_t sat, uint8_t *r, uint8_t *g, uint8_t *b)
{
  uint16_t scaled_hue = (uint16_t)hue * 6U;
  uint8_t  sector     = (uint8_t)(scaled_hue >> 8);
  uint32_t fraction   = scaled_hue & 0xFFU;

  uint8_t v = 255U;
  uint8_t p = 255U - sat;
  uint8_t q = (uint8_t)(255U - (((uint32_t)sat * fraction + 128U) >> 8));
  uint8_t t = (uint8_t)(255U - (((uint32_t)sat * (256U - fraction) + 128U) >> 8));

  switch (sector) {
    case 0U:  *r = v; *g = t; *b = p; break;
    case 1U:  *r = q; *g = v; *b = p; break;
    case 2U:  *r = p; *g = v; *b = t; break;
    case 3U:  *r = p; *g = q; *b = v; break;
    case 4U:  *r = t; *g = p; *b = v; break;
    default:  *r = v; *g = p; *b = q; break;
  }
}

/* ========================================================================== */
/* 公开接口 */
/* ========================================================================== */

/**
  * @brief  设置LED模式和颜色
  */
void BSP_LED_SetMode_DMA(LED_Mode_t mode, uint8_t r, uint8_t g, uint8_t b)
{
  osMutexAcquire(led_dma_mutex, osWaitForever);

  led_mode      = mode;
  led_r         = r;
  led_g         = g;
  led_b         = b;
  blink_tick    = 0U;
  blink_on      = 1U;
  breath_phase  = 0U;
  color_phase   = 0U;

  osMutexRelease(led_dma_mutex);

  if (mode == LED_MODE_OFF) {
    BSP_LED_SetRGB_DMA(0U, 0U, 0U);
  }
}

/**
  * @brief  设置呼吸周期
  */
void BSP_LED_SetBreathPeriod_DMA(uint16_t period_ms)
{
  if (period_ms < 200)   period_ms = 200;
  if (period_ms > 10000) period_ms = 10000;

  osMutexAcquire(led_dma_mutex, osWaitForever);
  breath_period    = period_ms;
  breath_phase     = 0U;
  breath_remainder = 0U;
  osMutexRelease(led_dma_mutex);
}

/**
  * @brief  设置色相周期
  */
void BSP_LED_SetColorPeriod_DMA(uint16_t period_ms)
{
  if (period_ms < 1000)  period_ms = 1000;
  if (period_ms > 30000) period_ms = 30000;

  osMutexAcquire(led_dma_mutex, osWaitForever);
  color_period     = period_ms;
  color_phase      = 0U;
  color_remainder  = 0U;
  osMutexRelease(led_dma_mutex);
}

/**
  * @brief  设置饱和度
  */
void BSP_LED_SetSaturation_DMA(uint8_t sat)
{
  led_sat = sat;
}

/**
  * @brief  查询DMA忙状态
  */
uint8_t BSP_LED_IsBusy_DMA(void)
{
  return dma_busy;
}

/* ========================================================================== */
/* LED任务 - 20ms周期调用 */
/* ========================================================================== */

/**
  * @brief  LED灯效任务（由FreeRTOS 20ms周期调用）
  */
void BSP_LED_Task_DMA(void)
{
  switch (led_mode) {
    case LED_MODE_OFF:
      BSP_LED_SetRGB_DMA(0U, 0U, 0U);
      break;

    case LED_MODE_ON:
      BSP_LED_SetRGB_DMA(led_r, led_g, led_b);
      break;

    case LED_MODE_BLINK:
    {
      blink_tick += LED_TASK_PERIOD_MS;
      while (blink_tick >= LED_BLINK_HALF_MS) {
        blink_tick -= LED_BLINK_HALF_MS;
        blink_on = !blink_on;
      }
      BSP_LED_SetRGB_DMA(blink_on ? led_r : 0U,
                         blink_on ? led_g : 0U,
                         blink_on ? led_b : 0U);
      break;
    }

    case LED_MODE_BREATHE_COLOR:
    {
      uint8_t scale = LED_Breath_Scale();
      uint8_t r, g, b;

      color_phase = LED_AdvancePhase(color_phase, color_period, &color_remainder);
      LED_HSVToRGB((uint8_t)(color_phase >> 8), led_sat, &r, &g, &b);
      LED_ApplyBrightness(scale, &r, &g, &b);
      BSP_LED_SetRGB_DMA(r, g, b);
      break;
    }

    case LED_MODE_BREATHE:
    default:
    {
      uint8_t scale = LED_Breath_Scale();
      uint8_t r = led_r, g = led_g, b = led_b;

      LED_ApplyBrightness(scale, &r, &g, &b);
      BSP_LED_SetRGB_DMA(r, g, b);
      break;
    }
  }
}
