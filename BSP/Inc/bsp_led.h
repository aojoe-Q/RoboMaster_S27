/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_led.h
  * @brief          : 板载 2812 LED（WS2812/SK6812）驱动接口
  * @author         : DeepSeek
  * @date           : 2025/11/08
  * @version        : v1.0
  ******************************************************************************
  * @attention      :
  *   达妙 MC-02 开发板上，PA7 不是普通 GPIO LED：
  *       PA7 == SPI6_MOSI == 板上 "2812 LED" 的 DIN。
  *   2812 是单线归零码器件（0.4us/0.8us 高电平区分 0/1），
  *   必须用 SPI 或 PWM+DMA 生成严格位时序，普通 PWM 改占空比是点不亮的。
  *   本驱动用 SPI6（2.5MHz，3 个 SPI bit 表示 1 个 2812 bit）实现。
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef BSP_LED_H
#define BSP_LED_H

#include "main.h"

/* 灯效模式 */
typedef enum
{
  LED_MODE_OFF = 0,   /* 常灭 */
  LED_MODE_ON,        /* 常亮 */
  LED_MODE_BREATHE,       /* 单色呼吸灯 */
  LED_MODE_BREATHE_COLOR, /* 彩色呼吸：亮度走呼吸曲线，色相同时缓慢绕圈 */
  LED_MODE_BLINK          /* 闪烁，可用于故障提示 */
} LED_Mode_t;

/* 默认配色与节奏：蓝色呼吸，一个完整周期 1.5s */
#define LED_DEFAULT_R          0U
#define LED_DEFAULT_G          40U
#define LED_DEFAULT_B          255U
#define LED_DEFAULT_PERIOD_MS  1500U

/* 彩色呼吸：色相绕完一圈的时间（ms）；取与呼吸周期不成整数倍关系的值，
 * 两者相位会持续错开，视觉上不会出现"每 N 圈重复一次"的呆板图案 */
#define LED_DEFAULT_COLOR_PERIOD_MS  7000U

/* 彩色呼吸的饱和度：255 = 纯色；调低后颜色变淡、趋向白色（淡彩提示） */
#define LED_DEFAULT_SAT              255U

/* 闪烁半周期（ms）与灯效调用周期（ms） */
#define LED_BLINK_HALF_MS      150U
#define LED_TASK_PERIOD_MS     20U

extern void BSP_LED_Init(void);
extern void BSP_LED_SetRGB(uint8_t r, uint8_t g, uint8_t b);
extern void BSP_LED_SetMode(LED_Mode_t mode, uint8_t r, uint8_t g, uint8_t b);
extern void BSP_LED_SetBreathPeriod(uint16_t period_ms);
extern void BSP_LED_SetColorPeriod(uint16_t period_ms);
extern void BSP_LED_SetSaturation(uint8_t sat);
extern void BSP_LED_Task(void);

#endif // BSP_LED_H
