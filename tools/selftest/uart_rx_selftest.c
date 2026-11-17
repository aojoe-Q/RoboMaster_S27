/* Model double-buffer CT/NDTR rollover and HAL's IDLE callback condition. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#undef assert
#define assert(c) do { if (!(c)) { fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#c); exit(1); } } while(0)
#define DMA_SxCR_EN 1U
#define DMA_SxCR_CT 2U
#define DMA_SxCR_DBM 4U
#define USART_CR3_DMAR 1U
#define HAL_UART_RECEPTION_TOIDLE 1U
#define UART_IT_IDLE 1U
#define TC_FLAG 1U
#define SET_BIT(r,b) ((r)|=(b))
#define CLEAR_BIT(r,b) ((r)&=~(b))
typedef struct { uint32_t CR,NDTR,PAR,M0AR,M1AR; } DMA_Stream_TypeDef;
typedef struct { DMA_Stream_TypeDef *Instance; uint32_t flags; } DMA_HandleTypeDef;
typedef struct { uint32_t RDR,CR3; } UART_Regs;
typedef struct { DMA_HandleTypeDef *hdmarx; UART_Regs *Instance; uint32_t ReceptionType,RxXferSize; } UART_HandleTypeDef;
#define __HAL_DMA_DISABLE(d) ((d)->Instance->CR&=~DMA_SxCR_EN)
#define __HAL_DMA_ENABLE(d) ((d)->Instance->CR|=DMA_SxCR_EN)
#define __HAL_DMA_SET_COUNTER(d,n) ((d)->Instance->NDTR=(n))
#define __HAL_DMA_GET_COUNTER(d) ((d)->Instance->NDTR)
#define __HAL_DMA_GET_TC_FLAG_INDEX(d) TC_FLAG
#define __HAL_DMA_GET_FLAG(d,f) ((d)->flags&(f))
#define __HAL_DMA_CLEAR_FLAG(d,f) ((d)->flags&=~(f))
#define __HAL_UART_ENABLE_IT(u,i) ((void)(u),(void)(i))
static uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_DMA_SIZE];
static unsigned calls,parsed_buffer;
static int remote_ctrl;
void SBUS_TO_RC(const uint8_t *data,int *remote) {
    assert(remote==&remote_ctrl);calls++;
    parsed_buffer=data==SBUS_MultiRx_Buf[0]?0:1;
    for(unsigned n=0;n<SBUS_RX_BUF_NUM;n++) assert(data[n]==0x55);
}
/* REAL_FUNCTIONS */
static void receive(UART_HandleTypeDef *uart,unsigned bytes) {
    DMA_HandleTypeDef *dma=uart->hdmarx;
    while(bytes--) {
        assert(dma->Instance->CR&DMA_SxCR_EN);
        unsigned buffer=(dma->Instance->CR&DMA_SxCR_CT)?1:0;
        SBUS_MultiRx_Buf[buffer][SBUS_RX_DMA_SIZE-dma->Instance->NDTR]=0x55;
        if(--dma->Instance->NDTR==0) {
            dma->Instance->CR^=DMA_SxCR_CT;
            dma->Instance->NDTR=uart->RxXferSize;
            dma->flags|=TC_FLAG;
        }
    }
    /* Same callback gate and Size calculation as HAL_UART_IRQHandler. */
    unsigned remaining=dma->Instance->NDTR;
    if(remaining>0 && remaining<uart->RxXferSize) {
        USER_USART5_RxHandler(uart,(uint16_t)(uart->RxXferSize-remaining));
        __HAL_DMA_ENABLE(dma);
    }
}
int main(void) {
    DMA_Stream_TypeDef stream={0};DMA_HandleTypeDef dma={&stream,TC_FLAG};
    UART_Regs regs={0};UART_HandleTypeDef uart={&dma,&regs,0,0};
    USART_RxDMA_MultiBuffer_Init(&uart,(uint32_t *)SBUS_MultiRx_Buf[0],
                               (uint32_t *)SBUS_MultiRx_Buf[1],sizeof SBUS_MultiRx_Buf[0]);
    assert(uart.RxXferSize==sizeof SBUS_MultiRx_Buf[0] && stream.NDTR==uart.RxXferSize);
    assert(uart.RxXferSize>SBUS_RX_BUF_NUM && !dma.flags);
    for(unsigned n=0;n<10;n++) {
        receive(&uart,SBUS_RX_BUF_NUM);
        assert(calls==n+1 && parsed_buffer==n%2);
    }
    unsigned before=calls;
    for(unsigned n=1;n<SBUS_RX_DMA_SIZE;n++) {
        if(n==SBUS_RX_BUF_NUM) continue;
        receive(&uart,n);assert(calls==before);
    }
    /* Reject both an exact DMA-block overflow and an 18-byte overflow tail. */
    receive(&uart,SBUS_RX_DMA_SIZE);assert(calls==before);
    receive(&uart,SBUS_RX_BUF_NUM);assert(calls==before);
    receive(&uart,SBUS_RX_BUF_NUM);assert(calls==++before);
    receive(&uart,SBUS_RX_DMA_SIZE+SBUS_RX_BUF_NUM);assert(calls==before);
    receive(&uart,SBUS_RX_BUF_NUM);assert(calls==++before);
    /* The shared USART1 initializer must report its exact capacity too. */
    USART_RxDMA_MultiBuffer_Init(&uart,(uint32_t *)SBUS_MultiRx_Buf[0],
                               (uint32_t *)SBUS_MultiRx_Buf[1],128);
    assert(uart.RxXferSize==128 && stream.NDTR==128);
    puts("uart: OK (alternating frames, partial/oversize rejection, overflow recovery)");
    return 0;
}
