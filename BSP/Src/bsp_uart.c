/* USART1：256 字节环形 DMA 流，HT/TC/IDLE 共用优先级 5。
 * UART5：普通 36 字节 DMA；只在 IDLE 时接受一帧 18 字节 DBUS 数据。
 * 任何缓冲区都不会被配置得比它的实际分配更大。 */
#include "bsp_uart.h"
#include "usart.h"
#include "Remote_Control.h"
#include "Image_Transmission.h"
#include "master_link.h"
static volatile uint8_t image_restart,remote_restart;
PLL2_ClocksTypeDef PLL2_ClockFreq;
static void remote_start(void)
{
    if(HAL_UARTEx_ReceiveToIdle_DMA(&huart5,(uint8_t *)SBUS_MultiRx_Buf,sizeof(SBUS_MultiRx_Buf))!=HAL_OK)
        remote_restart=1;
    else {
        remote_restart=0;
        __HAL_DMA_DISABLE_IT(huart5.hdmarx,DMA_IT_HT);
    }
}
void BSP_USART_Init(void)
{
    HAL_RCCEx_GetPLL2ClockFreq(&PLL2_ClockFreq);
    USART1->CR1 &= ~USART_CR1_UE;
    USART1->BRR=PLL2_ClockFreq.PLL2_Q_Frequency/921600U;
    USART1->CR1 |= USART_CR1_UE;
    Image_Transmission_StreamReset();
    image_restart=(HAL_UARTEx_ReceiveToIdle_DMA(&huart1,(uint8_t *)Image_Trans_MultiRx_Buff,sizeof(Image_Trans_MultiRx_Buff))!=HAL_OK);
    remote_start();
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t size)
{
    if(huart==&huart1) {
        /* 读实时 NDTR，避免已挂起的 HT/TC 重放旧字节。 */
        Image_Transmission_DMAEvent(sizeof(Image_Trans_MultiRx_Buff)-__HAL_DMA_GET_COUNTER(huart->hdmarx));
    } else if(huart==&huart5) {
        if(size==SBUS_RX_BUF_NUM && HAL_UARTEx_GetRxEventType(huart)==HAL_UART_RXEVENT_IDLE)
            SBUS_TO_RC(SBUS_MultiRx_Buf[0],&remote_ctrl);
        remote_start();
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart==&huart1) { image_restart=1; MasterLink_Invalidate(); }
    if(huart==&huart5) { remote_restart=1; remote_ctrl.rc_lost=true; }
}
void BSP_ImageRx_Service(void)
{
    if(image_restart) {
        HAL_NVIC_DisableIRQ(USART1_IRQn);
        HAL_NVIC_DisableIRQ(DMA1_Stream5_IRQn);
        HAL_UART_AbortReceive(&huart1);
        Image_Transmission_StreamReset();
        image_restart=(HAL_UARTEx_ReceiveToIdle_DMA(&huart1,(uint8_t *)Image_Trans_MultiRx_Buff,sizeof(Image_Trans_MultiRx_Buff))!=HAL_OK);
        HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
    if(remote_restart) {
        /* UART5 及其 DMA 都是优先级 5；任务临界区会同时屏蔽两者。 */
        uint32_t p=__get_PRIMASK(); __disable_irq();
        HAL_UART_AbortReceive(&huart5);
        remote_start();
        __set_PRIMASK(p);
    }
}
void USART_Vofa_Justfloat_Transmit(float SendValue1,float SendValue2,float SendValue3){
 
    __attribute__((section (".AXI_SRAM")))  static uint8_t Rx_Buf[16];

		uint8_t *SendValue1_Pointer,*SendValue2_Pointer,*SendValue3_Pointer;

		SendValue1_Pointer = (uint8_t *)&SendValue1;
		SendValue2_Pointer = (uint8_t *)&SendValue2;
		SendValue3_Pointer = (uint8_t *)&SendValue3;


		Rx_Buf[0] =  *SendValue1_Pointer;
		Rx_Buf[1] =  *(SendValue1_Pointer + 1);
		Rx_Buf[2] =  *(SendValue1_Pointer + 2);
		Rx_Buf[3] =  *(SendValue1_Pointer + 3);
		Rx_Buf[4] =  *SendValue2_Pointer;
		Rx_Buf[5] =  *(SendValue2_Pointer + 1);
		Rx_Buf[6] =  *(SendValue2_Pointer + 2);
		Rx_Buf[7] =  *(SendValue2_Pointer + 3);
		Rx_Buf[8] =  *SendValue3_Pointer;
		Rx_Buf[9] =  *(SendValue3_Pointer + 1);
		Rx_Buf[10] = *(SendValue3_Pointer + 2);
		Rx_Buf[11] = *(SendValue3_Pointer + 3);
		Rx_Buf[12] =  0x00;
		Rx_Buf[13] =  0x00;
		Rx_Buf[14] =  0x80;
		Rx_Buf[15] =  0x7F;
		HAL_UART_Transmit_DMA(&huart7,Rx_Buf,sizeof(Rx_Buf));
	

}


