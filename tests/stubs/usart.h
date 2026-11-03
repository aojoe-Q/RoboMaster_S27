#ifndef TEST_USART_H
#define TEST_USART_H
#include <stdint.h>
typedef struct { int dummy; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart1;
uint32_t HAL_GetTick(void);
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t p) { (void)p; }
static inline int HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,unsigned n)
{ (void)u; (void)p; (void)n; return 0; }
#endif
