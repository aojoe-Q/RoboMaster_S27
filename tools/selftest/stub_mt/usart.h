/* Host-side stub for Image_Transmission.c: HAL tick, IRQ mask, UART TX. */
#ifndef STUB_USART_H
#define STUB_USART_H

#include <stdint.h>
#include <stddef.h>

typedef struct { int dummy; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart1;

uint32_t HAL_GetTick(void);
static inline int HAL_UART_Transmit_DMA(UART_HandleTypeDef *h, uint8_t *d, uint16_t n)
{
    (void)h; (void)d; (void)n;
    return 0;
}
static inline uint32_t __get_PRIMASK(void) { return 0U; }
static inline void __disable_irq(void) { }
static inline void __set_PRIMASK(uint32_t v) { (void)v; }

#endif /* STUB_USART_H */
