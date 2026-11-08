/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_led.c
  * @brief          : 板载 2812 LED（WS2812/SK6812）驱动 + 呼吸灯效果
  * @author         : DeepSeek
  * @date           : 2025/11/08
  * @version        : v1.0
  ******************************************************************************
  * @attention      :
  *  1. 时序：2812 每个 bit 用高电平宽度区分 0/1
  *         0 : 高 ≈0.40us，低 ≈0.80us
  *         1 : 高 ≈0.80us，低 ≈0.40us
  *         一位总长约 1.25us，帧间低电平 >50us 即复位锁存。
  *
  *  2. SPI6 编码：内核时钟默认取 PLL1Q = 160MHz（HSE 24M /M6 *N160 /Q4），
  *     预分频 64 -> 2.5MHz，即 1 个 SPI bit = 400ns：
  *         逻辑 0 -> "100"：高 400ns / 低 800ns
  *         逻辑 1 -> "110"：高 800ns / 低 400ns
  *     24 bit x 3 = 72 bit = 9 字节，一帧 28.8us。
  *     若以后改时钟树导致 PLL1Q 变化，必须同步改
  *     WS2812_SPI_PRESCALER，保证 SPI 频率落在 2.1~3.1MHz
  *     （对应 T0H 0.25~0.55us、T1H 0.65~0.95us）。
  *
  *  3. 发送是阻塞的（9 字节约 29us），只适合 20~50Hz 刷新，
  *     不要在电机输出任务（250Hz）里调用。
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bsp_led.h"

/* Private define ------------------------------------------------------------*/
/* 板上只有 1 颗 2812；若以后级联多颗，改这里并保证 ws_buf 够大 */
#define WS2812_LED_NUM        1U
/* 每颗灯 24bit，每 bit 用 3 个 SPI bit 表示 -> 72bit = 9 字节 */
#define WS2812_BYTES_PER_LED  9U
#define WS2812_BUF_SIZE       (WS2812_LED_NUM * WS2812_BYTES_PER_LED)

/* 呼吸相位每个 tick 的最大步进（防止周期过小时相位跳变过快） */
#define LED_PHASE_STEP_MAX    60000UL

/* Private variables ---------------------------------------------------------*/
/* SPI6 句柄：CubeMX 未生成 SPI6，这里手工初始化，避免与 spi.c 里的 SPI2 冲突 */
static SPI_HandleTypeDef hspi6;

/* 一帧的 SPI 编码缓冲 */
static uint8_t ws_buf[WS2812_BUF_SIZE];

/* 当前显示状态 */
static LED_Mode_t led_mode      = LED_MODE_BREATHE;
static uint8_t    led_r         = LED_DEFAULT_R;
static uint8_t    led_g         = LED_DEFAULT_G;
static uint8_t    led_b         = LED_DEFAULT_B;
static uint8_t    led_sat       = LED_DEFAULT_SAT;  /* 彩色呼吸的饱和度 */
static uint16_t   breath_period = LED_DEFAULT_PERIOD_MS;
static uint16_t   color_period  = LED_DEFAULT_COLOR_PERIOD_MS;

/* 呼吸相位、色相相位（16 位小数相位，分别取高 6 位查表 / 高 8 位取色相）与闪烁计时 */
static uint16_t   breath_phase = 0U;
static uint16_t   color_phase  = 0U;
static uint16_t   blink_tick   = 0U;
static uint8_t    blink_on     = 1U;

/* 呼吸曲线查表：brightness = 255 * ((1-cos(2*pi*i/64))/2)^2.2
 * 先做余弦再叠 gamma，视觉上才是等速的亮暗变化（否则暗段几乎看不出来） */
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

/* Private function prototypes -----------------------------------------------*/
static void WS2812_SetBit(uint8_t *buf, uint32_t bit_index, uint8_t value);
static void WS2812_Pack(uint8_t *buf, uint8_t g, uint8_t r, uint8_t b);
static uint8_t LED_Breath_Scale(void);
static void LED_HSVToRGB(uint8_t hue, uint8_t sat, uint8_t *r, uint8_t *g, uint8_t *b);

/* USER CODE BEGIN 1 */

/**
  * @brief  把 1 个逻辑 bit 写进 SPI 缓冲（MSB first）。
  * @param  buf       编码缓冲
  * @param  bit_index 逻辑 bit 序号（0 = 最高位）
  * @param  value     1 置位，0 不动
  * @retval None
  */
static void WS2812_SetBit(uint8_t *buf, uint32_t bit_index, uint8_t value)
{
  if (value)
  {
    buf[bit_index >> 3] |= (uint8_t)(0x80U >> (bit_index & 0x07U));
  }
}

/**
  * @brief  把一颗灯的 GRB 打包成 9 字节 SPI 波形。
  * @note   2812 的传输顺序是 G -> R -> B，高位在前。
  *         逻辑 0 -> SPI "100"；逻辑 1 -> SPI "110"
  *         （末位恒为 0，保证每一位都以低电平收尾，符合归零码要求）
  * @param  buf 9 字节输出缓冲
  * @param  g   绿色分量（2812 先收 G）
  * @param  r   红色分量
  * @param  b   蓝色分量
  * @retval None
  */
static void WS2812_Pack(uint8_t *buf, uint8_t g, uint8_t r, uint8_t b)
{
  uint32_t bits = ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;
  uint32_t i;

  for (i = 0U; i < WS2812_BYTES_PER_LED; i++)
  {
    buf[i] = 0U;
  }

  for (i = 0U; i < 24U; i++)
  {
    uint32_t one = (bits >> (23U - i)) & 0x01U;

    /* 每个 2812 bit 占 3 个 SPI bit：首位必为 1，次位表示 0/1，末位补 0 */
    WS2812_SetBit(buf, i * 3U, 1U);
    WS2812_SetBit(buf, i * 3U + 1U, (uint8_t)one);
  }
}

/**
  * @brief  初始化 PA7 / SPI6 并熄灭 LED。
  * @retval None
  */
void BSP_LED_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint32_t i;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_SPI6_CLK_ENABLE();

  /* PA7 -> SPI6_MOSI（AF8） */
  GPIO_InitStruct.Pin       = GPIO_PIN_7;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_SPI6;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* SPI6：只发不收，半双工单线，2.5MHz，MSB first */
  hspi6.Instance               = SPI6;
  hspi6.Init.Mode              = SPI_MODE_MASTER;
  hspi6.Init.Direction         = SPI_DIRECTION_1LINE;
  hspi6.Init.DataSize          = SPI_DATASIZE_8BIT;
  hspi6.Init.CLKPolarity       = SPI_POLARITY_LOW;
  hspi6.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi6.Init.NSS               = SPI_NSS_SOFT;
  hspi6.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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

  if (HAL_SPI_Init(&hspi6) != HAL_OK)
  {
    Error_Handler();
  }

  for (i = 0U; i < WS2812_BUF_SIZE; i++)
  {
    ws_buf[i] = 0U;
  }

  /* 上电先发一帧全 0，避免灯珠上电后保持随机颜色 */
  BSP_LED_SetRGB(0U, 0U, 0U);
}
//------------------------------------------------------------------------------

/**
  * @brief  发送一帧颜色（阻塞，约 29us）。
  * @param  r 红 0~255
  * @param  g 绿 0~255
  * @param  b 蓝 0~255
  * @retval None
  */
void BSP_LED_SetRGB(uint8_t r, uint8_t g, uint8_t b)
{
  uint32_t i;

  for (i = 0U; i < WS2812_LED_NUM; i++)
  {
    WS2812_Pack(&ws_buf[i * WS2812_BYTES_PER_LED], g, r, b);
  }

  (void)HAL_SPI_Transmit(&hspi6, ws_buf, (uint16_t)WS2812_BUF_SIZE, 10U);
}
//------------------------------------------------------------------------------

/**
  * @brief  设置灯效模式与颜色。
  * @param  mode  灯效模式
  * @param  r/g/b 颜色分量
  * @retval None
  */
void BSP_LED_SetMode(LED_Mode_t mode, uint8_t r, uint8_t g, uint8_t b)
{
  led_mode   = mode;
  led_r      = r;
  led_g      = g;
  led_b      = b;
  blink_tick  = 0U;
  blink_on    = 1U;
  color_phase = 0U;

  if (mode == LED_MODE_OFF)
  {
    BSP_LED_SetRGB(0U, 0U, 0U);
  }
}
//------------------------------------------------------------------------------

/**
  * @brief  设置呼吸周期。
  * @param  period_ms 一个完整亮暗周期（ms），钳位在 200~10000
  * @retval None
  */
void BSP_LED_SetBreathPeriod(uint16_t period_ms)
{
  if (period_ms < 200U)
  {
    period_ms = 200U;
  }
  if (period_ms > 10000U)
  {
    period_ms = 10000U;
  }

  breath_period = period_ms;
  breath_phase  = 0U;
}
//------------------------------------------------------------------------------

/**
  * @brief  设置彩色呼吸的色相转速。
  * @param  period_ms 色相绕完整一圈的时间（ms），钳位在 1000~30000
  * @retval None
  */
void BSP_LED_SetColorPeriod(uint16_t period_ms)
{
  if (period_ms < 1000U)
  {
    period_ms = 1000U;
  }
  if (period_ms > 30000U)
  {
    period_ms = 30000U;
  }

  color_period = period_ms;
  color_phase  = 0U;
}
//------------------------------------------------------------------------------

/**
  * @brief  设置彩色呼吸的饱和度。
  * @note   只影响 LED_MODE_BREATHE_COLOR；单色呼吸（LED_MODE_BREATHE）
  *         直接用 SetMode 传进来的 RGB，不受这里影响。
  * @param  sat 0~255：255 = 纯色，数值越小越接近白色（淡彩）
  * @retval None
  */
void BSP_LED_SetSaturation(uint8_t sat)
{
  led_sat = sat;
}
//------------------------------------------------------------------------------

/**
  * @brief  推进一次呼吸相位，返回当前亮度系数。
  * @retval 0~255（已按 gamma 2.2 校正，暗段变化慢、亮段变化快）
  */
static uint8_t LED_Breath_Scale(void)
{
  uint32_t step;

  /* 16 位相位累加：每个 tick 前进 (65536 * tick) / 周期 */
  step = (65536UL * (uint32_t)LED_TASK_PERIOD_MS) / (uint32_t)breath_period;
  if (step > LED_PHASE_STEP_MAX)
  {
    step = LED_PHASE_STEP_MAX;
  }
  breath_phase = (uint16_t)(breath_phase + (uint16_t)step);

  /* 高 6 位查表 -> 0~255 的亮度系数 */
  return led_breath_lut[(breath_phase >> 10) & 0x3FU];
}
//------------------------------------------------------------------------------

/**
  * @brief  色相/饱和度 -> RGB（六扇区 HSV，纯整数运算）。
  * @note   与"全饱和三段轮换"不同，这里带饱和度参数：
  *           sat = 255 时就是标准色环，sat 越小颜色越淡、趋向白色；
  *           典型用法：状态提示用淡彩（sat 150~200），醒目提示用纯色。
  *         v 恒取 255：明暗由调用方再乘呼吸系数，颜色与亮度两条时间线解耦。
  *         色环按 256 等分映射到 0~360°（hue=255 约为 358°），
  *         段内比例 rem 用 /59 拉伸到满量程 255，扇区交界处颜色连续。
  * @param  hue 色相 0~255
  * @param  sat 饱和度 0~255（0 = 白，255 = 纯色）
  * @param  r   红输出 0~255
  * @param  g   绿输出 0~255
  * @param  b   蓝输出 0~255
  * @retval None
  */
static void LED_HSVToRGB(uint8_t hue, uint8_t sat, uint8_t *r, uint8_t *g, uint8_t *b)
{
  uint16_t hh     = (uint16_t)(((uint32_t)hue * 360U) >> 8);      /* 0~255 -> 0~359 度 */
  uint16_t sector = (uint16_t)(hh / 60U);                          /* 落在哪个 60 度扇区 */
  int32_t  rem    = (int32_t)(((hh % 60U) * 255U) / 59U);          /* 扇区内比例 0~255 */
  int32_t  s      = (int32_t)sat;
  int32_t  v      = 255;                                           /* 明度交给调用方缩放 */
  int32_t  p;                                                      /* 最小分量 */
  int32_t  q;                                                      /* 上升沿分量 */
  int32_t  t;                                                      /* 下降沿分量 */

  p = (v * (255 - s)) / 255;
  q = (v * (255 - (rem * s) / 255)) / 255;
  t = (v * (255 - ((255 - rem) * s) / 255)) / 255;

  switch (sector)
  {
    case 0U:                                             /* 0~60°   红 -> 黄 */
      *r = (uint8_t)v; *g = (uint8_t)t; *b = (uint8_t)p;
      break;
    case 1U:                                             /* 60~120° 黄 -> 绿 */
      *r = (uint8_t)q; *g = (uint8_t)v; *b = (uint8_t)p;
      break;
    case 2U:                                             /* 120~180° 绿 -> 青 */
      *r = (uint8_t)p; *g = (uint8_t)v; *b = (uint8_t)t;
      break;
    case 3U:                                             /* 180~240° 青 -> 蓝 */
      *r = (uint8_t)p; *g = (uint8_t)q; *b = (uint8_t)v;
      break;
    case 4U:                                             /* 240~300° 蓝 -> 洋红 */
      *r = (uint8_t)t; *g = (uint8_t)p; *b = (uint8_t)v;
      break;
    default:                                             /* 300~360° 洋红 -> 红 */
      *r = (uint8_t)v; *g = (uint8_t)p; *b = (uint8_t)q;
      break;
  }
}
//------------------------------------------------------------------------------

/**
  * @brief  灯效节拍，由任务以 LED_TASK_PERIOD_MS(20ms) 周期调用。
  * @retval None
  */
void BSP_LED_Task(void)
{
  uint8_t  scale;
  uint8_t  r8, g8, b8;
  uint32_t step;

  switch (led_mode)
  {
    case LED_MODE_OFF:
      BSP_LED_SetRGB(0U, 0U, 0U);
      break;

    case LED_MODE_ON:
      BSP_LED_SetRGB(led_r, led_g, led_b);
      break;

    case LED_MODE_BLINK:
      blink_tick += LED_TASK_PERIOD_MS;
      if (blink_tick >= LED_BLINK_HALF_MS)
      {
        blink_tick = 0U;
        blink_on   = (uint8_t)(!blink_on);
      }
      if (blink_on)
      {
        BSP_LED_SetRGB(led_r, led_g, led_b);
      }
      else
      {
        BSP_LED_SetRGB(0U, 0U, 0U);
      }
      break;

    case LED_MODE_BREATHE_COLOR:
      /* 彩色呼吸 = 亮度呼吸 x 色相绕圈，两条独立的时间线：
       * 亮度 1500ms 一个明暗周期，色相 7000ms 绕完 360 度。
       * 两者不是整数倍关系（4.67 个呼吸才转完一圈），
       * 所以每个亮峰对应的色相都落在色环的不同位置，
       * 连续看颜色是缓慢漂移的，不会每几秒重演同一串颜色 */
      scale = LED_Breath_Scale();

      step = (65536UL * (uint32_t)LED_TASK_PERIOD_MS) / (uint32_t)color_period;
      if (step > LED_PHASE_STEP_MAX)
      {
        step = LED_PHASE_STEP_MAX;
      }
      color_phase = (uint16_t)(color_phase + (uint16_t)step);

      /* 相位高 8 位 -> 0~255 的色相，再按呼吸亮度缩放 */
      LED_HSVToRGB((uint8_t)(color_phase >> 8), led_sat, &r8, &g8, &b8);

      BSP_LED_SetRGB((uint8_t)(((uint32_t)r8 * scale) / 255U),
                     (uint8_t)(((uint32_t)g8 * scale) / 255U),
                     (uint8_t)(((uint32_t)b8 * scale) / 255U));
      break;

    case LED_MODE_BREATHE:
    default:
      scale = LED_Breath_Scale();

      BSP_LED_SetRGB((uint8_t)(((uint32_t)led_r * scale) / 255U),
                     (uint8_t)(((uint32_t)led_g * scale) / 255U),
                     (uint8_t)(((uint32_t)led_b * scale) / 255U));
      break;
  }
}
//------------------------------------------------------------------------------

/* USER CODE END 1 */
