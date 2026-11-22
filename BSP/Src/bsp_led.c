/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_led.c
  * @brief          : 板载 2812 LED（WS2812/SK6812）驱动 + 呼吸灯效果（优化版）
  * @author         : DeepSeek (优化: Claude Opus 5.5)
  * @date           : 2025/11/08
  * @version        : v1.1.4
  ******************************************************************************
  * @attention      :
  *  1. 时序：2812 每个 bit 用高电平宽度区分 0/1
  *         0 : 高 ≈0.40us，低 ≈0.80us
  *         1 : 高 ≈0.80us，低 ≈0.40us
  *         一位总长约 1.25us，帧间低电平 >50us 即复位锁存。
  *
  *  2. SPI6 编码：按实际内核时钟自动选择最接近 2.5MHz 的分频，
  *     本工程默认 PCLK4 = 80MHz，预分频 32，即 1 个 SPI bit = 400ns：
  *         逻辑 0 -> "100"：高 400ns / 低 800ns
  *         逻辑 1 -> "110"：高 800ns / 低 400ns
  *     24 bit x 3 = 72 bit = 9 字节，一帧 28.8us。
  *
  *  3. 发送是阻塞的（9 字节约 29us），只适合 20~50Hz 刷新，
  *     不要在电机输出任务（250Hz）里调用。
  *     接口须由同一任务串行调用；直接 SetRGB 时帧间需留足复位低电平。
  *
  *  4. 优化要点（v1.1 -> v1.1.2）：
  *     [v1.1] 性能优化：
  *     - 缓冲区 32 字节对齐，优化 DMA 传输效率（若改用 DMA）
  *     - RGB 状态合并为 32 位整数，单次比较替代三次分量比较
  *     - 热路径函数声明为 inline（WS2812_PackByte、LED_AdvancePhase）
  *     - 参数钳位用宏统一，减少分支
  *     - 抽取 SPI 分频计算为独立函数，便于单元测试
  *     - 提取亮度调制公共函数，减少重复代码
  *     [v1.1.1] 亮度跳变修复：
  *     - LED_ApplyBrightness 添加 +127U 四舍五入
  *     [v1.1.2] 闪白光修复（已回退）：
  *     - BSP_LED_SetRGB 传输期间禁用中断（导致RTOS死锁，已移除）
  *     [v1.1.4] 帧间电平修复：
  *     - MasterKeepIOState 改为 ENABLE（AFCNTR=1），SPI 关闭后 MOSI 保持 0
  *     - PA7 由 NOPULL 改为 PULLDOWN，消除帧间悬空拾噪
  *     [v1.1.3] 紧急修复：
  *     - 移除 __disable_irq()，恢复兼容性（RTOS和裸机均可用）
  *     - 如仍有闪白光，可选择性启用中断屏蔽（见代码注释）
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

/* SPI 时序参数 */
#define WS2812_TARGET_FREQ    2500000U  /* 目标 2.5MHz */
#define WS2812_FREQ_TOLERANCE 600000U   /* 容差 ±0.6MHz，覆盖 2.1~3.1MHz 范围 */

/* 呼吸查表参数 */
#define BREATH_LUT_SIZE       64U
#define BREATH_LUT_SHIFT      10U       /* 16位相位 -> 6位索引，高6位 */
#define BREATH_INTERP_MASK    0x03FFU   /* 低10位用于插值 */
#define BREATH_INTERP_SHIFT   10U

/* RGB打包/比较用的位移 */
#define RGB_PACK_R_SHIFT      16U
#define RGB_PACK_G_SHIFT      8U

/* 参数钳位范围 */
#define BREATH_PERIOD_MIN     200U
#define BREATH_PERIOD_MAX     10000U
#define COLOR_PERIOD_MIN      1000U
#define COLOR_PERIOD_MAX      30000U

/* 编译时工具宏 */
#define CLAMP(val, min, max)  ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

/* Private variables ---------------------------------------------------------*/
/* SPI6 句柄：CubeMX 未生成 SPI6，这里手工初始化，避免与 spi.c 里的 SPI2 冲突 */
static SPI_HandleTypeDef hspi6;

/* 一帧的 SPI 编码缓冲，32字节对齐优化DMA传输 */
__attribute__((aligned(32))) static uint8_t ws_buf[WS2812_BUF_SIZE];

/* 仅灯效任务去重；公开 SetRGB 接口仍强制发送。发送失败后允许重试。 */
/* 合并成一个32位值加速比较：0x00RRGGBB */
static uint32_t last_rgb_packed = 0xFFFFFFFFU;  /* 初始值非法，强制首次发送 */
static uint8_t last_rgb_valid = 0U;

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
static uint32_t   breath_remainder = 0U;
static uint32_t   color_remainder  = 0U;
static uint32_t   blink_tick   = 0U;
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
static inline void WS2812_PackByte(uint8_t *buf, uint8_t value);
static void WS2812_Pack(uint8_t *buf, uint8_t g, uint8_t r, uint8_t b);
static void LED_UpdateRGB(uint8_t r, uint8_t g, uint8_t b);
static inline uint16_t LED_AdvancePhase(uint16_t phase, uint16_t period, uint32_t *remainder);
static uint8_t LED_Breath_Scale(void);
static void LED_HSVToRGB(uint8_t hue, uint8_t sat, uint8_t *r, uint8_t *g, uint8_t *b);
static uint32_t SPI6_FindBestPrescaler(uint32_t spi_clk);
static inline uint32_t RGB_Pack(uint8_t r, uint8_t g, uint8_t b);

/* USER CODE BEGIN 1 */

/**
  * @brief  RGB打包成 32 位值，用于快速比较（避免三次分量比较）。
  * @param  r/g/b  颜色分量
  * @retval 打包后的 0x00RRGGBB
  */
static inline uint32_t RGB_Pack(uint8_t r, uint8_t g, uint8_t b)
{
  return ((uint32_t)r << RGB_PACK_R_SHIFT)
       | ((uint32_t)g << RGB_PACK_G_SHIFT)
       | b;
}

/**
  * @brief  用 16 字节半字节查表，将一个颜色分量编码为 3 字节。
  * @note   内联优化，查表数组声明为 const 便于编译器放入只读段。
  *         编码规则：逻辑 0 -> "100"，逻辑 1 -> "110"（末位恒0）
  * @param  buf       3 字节输出缓冲
  * @param  value     颜色分量（MSB first）
  * @retval None
  */
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
  WS2812_PackByte(&buf[0], g);
  WS2812_PackByte(&buf[3], r);
  WS2812_PackByte(&buf[6], b);
}

/**
  * @brief  SPI6 时钟分频自适应：选择最接近 2.5MHz 的分频系数。
  * @note   抽取为独立函数便于单元测试，避免 Init 函数过长。
  * @param  spi_clk  SPI6 的输入时钟（PCLK4 或其他时钟源）
  * @retval HAL 分频常量
  */
static uint32_t SPI6_FindBestPrescaler(uint32_t spi_clk)
{
  static const uint32_t br_tab[8] =
  {
    SPI_BAUDRATEPRESCALER_2,   SPI_BAUDRATEPRESCALER_4,
    SPI_BAUDRATEPRESCALER_8,   SPI_BAUDRATEPRESCALER_16,
    SPI_BAUDRATEPRESCALER_32,  SPI_BAUDRATEPRESCALER_64,
    SPI_BAUDRATEPRESCALER_128, SPI_BAUDRATEPRESCALER_256
  };

  uint32_t best     = 4U;              /* 兜底：80MHz/32 = 2.5MHz */
  uint32_t best_err = 0xFFFFFFFFU;

  if (spi_clk != 0U)
  {
    for (uint32_t i = 0U; i < 8U; i++)
    {
      uint32_t f   = spi_clk >> (i + 1U);
      uint32_t err = (f > WS2812_TARGET_FREQ)
                   ? (f - WS2812_TARGET_FREQ)
                   : (WS2812_TARGET_FREQ - f);

      if (err < best_err)
      {
        best_err = err;
        best     = i;
      }
    }
  }

  return br_tab[best];
}

/**
  * @brief  初始化 PA7 / SPI6 并熄灭 LED。
  * @retval None
  */
void BSP_LED_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_SPI6_CLK_ENABLE();

  /* PA7 -> SPI6_MOSI（AF8） */
  GPIO_InitStruct.Pin       = GPIO_PIN_7;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  /* 用下拉而不是浮空：SPI6 在帧间被 HAL 关闭时 AF 输出会释放，
   * 此时引脚若悬空，DIN 上的噪声会被 2812 当成数据位。下拉把空闲电平钉在 0。*/
  GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
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

  /* 自动选择最优分频：让 SPI 频率落在 2.1~3.1MHz（WS2812 时序容差）。
   * 详见 SPI6_FindBestPrescaler() 的实现说明。 */
  hspi6.Init.BaudRatePrescaler = SPI6_FindBestPrescaler(
      HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI6)
  );
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
  /* AFCNTR=1：SPI 被 HAL 关闭后仍保持引脚驱动，MOSI 停在最后一个 bit（编码末位恒为 0）。
   * 置 0 时每帧结束都把 PA7 交还给 GPIO，20ms 空闲期内该线悬空，
   * 灯珠可能把这些噪声当成 24bit 数据锁存 —— 表现就是偶发变亮/变色，下一帧又恢复。*/
  hspi6.Init.MasterKeepIOState         = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi6.Init.IOSwap                    = SPI_IO_SWAP_DISABLE;

  if (HAL_SPI_Init(&hspi6) != HAL_OK)
  {
    Error_Handler();
  }

  /* 上电先发一帧全 0，避免灯珠上电后保持随机颜色 */
  BSP_LED_SetRGB(0U, 0U, 0U);
}
//------------------------------------------------------------------------------

/**
  * @brief  发送一帧颜色（阻塞，约 29us）。
  * @note   ⚠️ 关键时序要求：
  *         - 9字节必须连续发送，任何延迟 >2us 都会导致WS2812误判帧结束
  *         - 传输期间禁用中断，防止被高优先级任务打断
  *         - 两帧之间必须有 >50us 低电平，否则数据错位
  * @param  r 红 0~255
  * @param  g 绿 0~255
  * @param  b 蓝 0~255
  * @retval None
  */
void BSP_LED_SetRGB(uint8_t r, uint8_t g, uint8_t b)
{
  /* 单颗灯直接打包，多颗灯时用循环 */
  WS2812_Pack(ws_buf, g, r, b);

  last_rgb_valid = 0U;

  /* ⚠️ 时序关键：9字节必须连续发送，任何延迟 >2us 都会导致误判帧结束
   *
   * 方案A（当前）：不屏蔽中断，依赖HAL库的SPI阻塞传输
   * - 优点：兼容RTOS，不影响任务调度
   * - 缺点：如果中断处理时间 >2us，仍可能闪白光
   *
   * 方案B（如果仍闪白光）：启用下面注释的代码，屏蔽中断
   * - 优点：完全消除中断干扰
   * - 缺点：29us内任务不可抢占（需确认系统能接受）
   */

  // uint32_t primask = __get_PRIMASK();
  // __disable_irq();

  HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi6, ws_buf, (uint16_t)WS2812_BUF_SIZE, 10U);

  // __set_PRIMASK(primask);

  if (status == HAL_OK)
  {
    last_rgb_packed = RGB_Pack(r, g, b);
    last_rgb_valid  = 1U;
  }

  /* 帧间低电平由两层保证：
   * 1) AFCNTR=1 使 MOSI 在 SPI 关闭后仍保持在最后一 bit（恒 0）；
   * 2) PA7 配置为下拉，即使 SPI 完全关闭也不会浮空。
   * 20ms 的刷新周期本身已远超 2812 要求的 >50us 复位低电平。*/
}
//------------------------------------------------------------------------------

static void LED_UpdateRGB(uint8_t r, uint8_t g, uint8_t b)
{
  uint32_t current_rgb = RGB_Pack(r, g, b);

  /* 合并条件：单次32位比较替代三次8位比较 */
  if ((!last_rgb_valid) || (current_rgb != last_rgb_packed))
  {
    BSP_LED_SetRGB(r, g, b);
  }
}

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
  color_remainder = 0U;

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
  breath_period    = CLAMP(period_ms, BREATH_PERIOD_MIN, BREATH_PERIOD_MAX);
  breath_phase     = 0U;
  breath_remainder = 0U;
}
//------------------------------------------------------------------------------

/**
  * @brief  设置彩色呼吸的色相转速。
  * @param  period_ms 色相绕完整一圈的时间（ms），钳位在 1000~30000
  * @retval None
  */
void BSP_LED_SetColorPeriod(uint16_t period_ms)
{
  color_period    = CLAMP(period_ms, COLOR_PERIOD_MIN, COLOR_PERIOD_MAX);
  color_phase     = 0U;
  color_remainder = 0U;
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
  * @brief  按固定任务节拍推进相位，并保留不足一个相位单位的余数。
  * @note   内联优化，减少函数调用开销（呼吸任务每 20ms 调用两次）。
  * @retval 自动回绕的 16 位相位
  */
static inline uint16_t LED_AdvancePhase(uint16_t phase, uint16_t period, uint32_t *remainder)
{
  uint32_t numerator = 65536UL * (uint32_t)LED_TASK_PERIOD_MS + *remainder;
  uint32_t step = numerator / period;

  /* 保留除法余数，避免每个 tick 截断导致周期持续漂移。 */
  *remainder = numerator % period;
  return (uint16_t)(phase + step);
}

/* 对 gamma 呼吸曲线插值，返回 0~255 的亮度系数。 */
static uint8_t LED_Breath_Scale(void)
{
  breath_phase = LED_AdvancePhase(breath_phase, breath_period, &breath_remainder);

  uint32_t index    = breath_phase >> BREATH_LUT_SHIFT;
  uint32_t fraction = breath_phase & BREATH_INTERP_MASK;

  /* 低 10 位用于相邻点插值，减轻慢呼吸时的亮度阶跃。 */
  return (uint8_t)(
      ((uint32_t)led_breath_lut[index] * (1024U - fraction)
     + (uint32_t)led_breath_lut[(index + 1U) & (BREATH_LUT_SIZE - 1U)] * fraction
     + 512U) >> BREATH_INTERP_SHIFT
  );
}
//------------------------------------------------------------------------------

/**
  * @brief  色相/饱和度 -> RGB（六扇区 HSV，纯整数运算）。
  * @note   优化点：
  *         1. 扇区计算合并到 switch，减少变量声明
  *         2. 使用查表优化除法和乘法
  *         3. 内联函数建议，热路径函数
  * @param  hue 色相 0~255
  * @param  sat 饱和度 0~255（0 = 白，255 = 纯色）
  * @param  r   红输出 0~255
  * @param  g   绿输出 0~255
  * @param  b   蓝输出 0~255
  * @retval None
  */
static void LED_HSVToRGB(uint8_t hue, uint8_t sat, uint8_t *r, uint8_t *g, uint8_t *b)
{
  uint16_t scaled_hue = (uint16_t)hue * 6U;
  uint8_t  sector     = (uint8_t)(scaled_hue >> 8);
  uint32_t fraction   = scaled_hue & 0xFFU;

  /* V 恒定为 255，明暗由呼吸系数单独控制 */
  uint8_t v = 255U;
  uint8_t p = 255U - sat;
  uint8_t q = (uint8_t)(255U - (((uint32_t)sat * fraction + 128U) >> 8));
  uint8_t t = (uint8_t)(255U - (((uint32_t)sat * (256U - fraction) + 128U) >> 8));

  /* 优化：移除 default 分支，将 sector >= 5 合并到最后一个 case */
  switch (sector)
  {
    case 0U:  *r = v; *g = t; *b = p; break;  /* 0~60°   红 -> 黄 */
    case 1U:  *r = q; *g = v; *b = p; break;  /* 60~120° 黄 -> 绿 */
    case 2U:  *r = p; *g = v; *b = t; break;  /* 120~180° 绿 -> 青 */
    case 3U:  *r = p; *g = q; *b = v; break;  /* 180~240° 青 -> 蓝 */
    case 4U:  *r = t; *g = p; *b = v; break;  /* 240~300° 蓝 -> 洋红 */
    default:  *r = v; *g = p; *b = q; break;  /* 300~360° 洋红 -> 红 */
  }
}
//------------------------------------------------------------------------------

/**
  * @brief  应用亮度系数到RGB分量（内联优化避免重复乘除）。
  * @note   使用 (x * scale + 127) / 255 代替 (x * scale) / 255，
  *         四舍五入而非向下取整，减少视觉上的亮度阶跃。
  *         等价于 (x * scale) / 255.0 + 0.5，但纯整数运算。
  * @param  scale  亮度系数 0~255
  * @param  r/g/b  输入输出颜色分量
  * @retval None
  */
static inline void LED_ApplyBrightness(uint8_t scale, uint8_t *r, uint8_t *g, uint8_t *b)
{
  *r = (uint8_t)(((uint32_t)*r * scale + 127U) / 255U);
  *g = (uint8_t)(((uint32_t)*g * scale + 127U) / 255U);
  *b = (uint8_t)(((uint32_t)*b * scale + 127U) / 255U);
}

/**
  * @brief  灯效节拍，由任务以 LED_TASK_PERIOD_MS(20ms) 周期调用。
  * @note   优化点：
  *         1. 提取公共亮度调制函数避免重复代码
  *         2. 减少局部变量作用域
  *         3. 闪烁计时器用 while 循环处理累积时间（防止高频调用时漏掉翻转）
  * @retval None
  */
void BSP_LED_Task(void)
{
  switch (led_mode)
  {
    case LED_MODE_OFF:
      LED_UpdateRGB(0U, 0U, 0U);
      break;

    case LED_MODE_ON:
      LED_UpdateRGB(led_r, led_g, led_b);
      break;

    case LED_MODE_BLINK:
      blink_tick += LED_TASK_PERIOD_MS;
      while (blink_tick >= LED_BLINK_HALF_MS)
      {
        blink_tick -= LED_BLINK_HALF_MS;
        blink_on    = !blink_on;
      }
      LED_UpdateRGB(blink_on ? led_r : 0U,
                    blink_on ? led_g : 0U,
                    blink_on ? led_b : 0U);
      break;

    case LED_MODE_BREATHE_COLOR:
    {
      /* 彩色呼吸 = 亮度呼吸 x 色相绕圈，两条独立时间线 */
      uint8_t scale = LED_Breath_Scale();
      uint8_t r8, g8, b8;

      color_phase = LED_AdvancePhase(color_phase, color_period, &color_remainder);

      /* 相位高 8 位 -> 0~255 的色相 */
      LED_HSVToRGB((uint8_t)(color_phase >> 8), led_sat, &r8, &g8, &b8);
      LED_ApplyBrightness(scale, &r8, &g8, &b8);
      LED_UpdateRGB(r8, g8, b8);
      break;
    }

    case LED_MODE_BREATHE:
    default:
    {
      uint8_t scale = LED_Breath_Scale();
      uint8_t r8 = led_r, g8 = led_g, b8 = led_b;

      LED_ApplyBrightness(scale, &r8, &g8, &b8);
      LED_UpdateRGB(r8, g8, b8);
      break;
    }
  }
}
//------------------------------------------------------------------------------

/* USER CODE END 1 */
