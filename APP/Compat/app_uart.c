#include "uart_bsp.h"

/*
 * The original S27_ARM_H7 APP declares this buffer but never defines it.
 * The referee task only consumes it; the custom-controller UART is optional on
 * this test build, so a zero-filled buffer is provided to keep the link and the
 * task functional without stealing USART1 from the image-transmission link.
 */
uint8_t uart1_rx_process_buf[512];
