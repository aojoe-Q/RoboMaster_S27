/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Image_Transmission.c
  * @brief          : Image_Transmission_Info interfaces functions 
  * @author         : GrassFan Wang
  * @date           : 2025/4/20
  * @version        : v1.0
  ******************************************************************************
  * @attention      : to be tested
  ******************************************************************************
  */
/* USER CODE END Header */

#include "Image_Transmission.h"
#include "CRC.h"
#include "usart.h"
#include "master_link.h"

__attribute__((section (".AXI_SRAM"))) uint8_t Image_Trans_MultiRx_Buff[2][128];

Image_Transmission_Info_TypeDef Image_Transmission_Info;
VT13_Info_TypeDef VT13_Info;

static int16_t bit8TObit16(uint8_t change_info[2]);


/* 重同步缓冲：最大帧 39 字节（0x0302），逐字节塞入再按帧头切分。 */
static uint8_t stream[39];
static uint16_t used, dma_read;
static MasterRawSnapshot master;
static ImageKeyboardSnapshot keyboard;
/* 快照会被中断写入，读写都用 PRIMASK 临界区保护。 */
static uint32_t link_lock(void) { uint32_t p=__get_PRIMASK(); __disable_irq(); return p; }
static void link_unlock(uint32_t p) { __set_PRIMASK(p); }
bool MasterLink_Read(MasterRawSnapshot *out)
{
    uint32_t p=link_lock(); *out=master; link_unlock(p); return out->valid;
}
bool ImageKeyboard_Read(ImageKeyboardSnapshot *out)
{
    uint32_t p=link_lock(); *out=keyboard; link_unlock(p); return out->valid;
}
void MasterLink_Invalidate(void)
{
    uint32_t p=link_lock(); master.valid=false; keyboard.valid=false; link_unlock(p);
}
/* 重启 DMA 或出错时丢弃半帧，并让上层看到主手已失效。 */
void Image_Transmission_StreamReset(void)
{
    used=0; dma_read=0; MasterLink_Invalidate();
}
/* 消费 n 字节：剩余字节前移，实现粘包与噪声后的重新同步。 */
static void consume(uint16_t n)
{
    used-=n; memmove(stream,stream+n,used);
}
/* 图传字节流解析入口（任务上下文）：有界切帧，先验长度与 CRC，再看 cmd。 */
void Image_Transmission_Info_Update(const uint8_t *data, uint16_t size)
{
    while (size--) {
        stream[used++]=*data++;
        while (used) {
            uint16_t total, cmd;
            /* 0xA9 0x53：VT13 的 21 字节帧，CRC16 通过后才交给旧解析。 */
            if (stream[0]==0xA9) {
                if (used<2) break;
                if (stream[1]!=0x53) { consume(1); continue; }
                if (used<21) break;
                if (!Verify_CRC16_Check_Sum(stream,21)) { consume(1); continue; }
                VT13_Info_Update(stream,&VT13_Info);
                consume(21); continue;
            }
            /* 0xA5 才是图传帧；其它字节一律丢弃 1 字节重新找帧头。 */
            if (stream[0]!=0xA5) { consume(1); continue; }
            if (used<5) break;
            /* 先验帧头 5 字节的 CRC8：长度字段本身不可信。 */
            if (!Verify_CRC8_Check_Sum(stream,5)) { consume(1); continue; }
            /* 总长 = 载荷 + 9（5 头 + 2 cmd + 2 CRC）；载荷上限 30，先查边界再等整帧。 */
            total=(uint16_t)stream[1] | ((uint16_t)stream[2]<<8);
            if (total>30) { consume(1); continue; }
            total+=9;
            if (used<total) break;
            if (!Verify_CRC16_Check_Sum(stream,total)) { consume(1); continue; }
            cmd=(uint16_t)stream[5] | ((uint16_t)stream[6]<<8);
            /* 0x0302：主手 30 字节载荷；0x0304：12 字节键盘。总长必须精确匹配。 */
            if (cmd==0x0302 && total==39) {
                uint32_t p=link_lock();
                memcpy(master.payload,stream+7,30);
                master.wire_seq=stream[3]; master.seq++;
                master.rx_tick=HAL_GetTick(); master.valid=true;
                memcpy(Image_Transmission_Info.custom_robot_data.data,stream+7,30);
                link_unlock(p);
            } else if (cmd==0x0304 && total==21) {
                uint32_t p=link_lock();
                memcpy(keyboard.payload,stream+7,12);
                keyboard.wire_seq=stream[3]; keyboard.seq++;
                keyboard.rx_tick=HAL_GetTick(); keyboard.valid=true;
                Image_Transmission_Info.remote_control.mouse_x=bit8TObit16(stream+7);
                Image_Transmission_Info.remote_control.mouse_y=bit8TObit16(stream+9);
                Image_Transmission_Info.remote_control.mouse_z=bit8TObit16(stream+11);
                Image_Transmission_Info.remote_control.left_button_down=stream[13];
                Image_Transmission_Info.remote_control.right_button_down=stream[14];
                Image_Transmission_Info.remote_control.Key.keyboard_value=bit8TObit16(stream+15);
                Image_Transmission_Info.remote_control.reserved=(uint16_t)stream[17]|((uint16_t)stream[18]<<8);
                link_unlock(p);
            }
            consume(total);
        }
    }
}
/* USART1 与 DMA1_Stream5 的中断抢占优先级相同。
 * position 是环形 DMA 的实时写游标（0..capacity），不是事件的 Size。
 * 读 NDTR 可以避免 IDLE 早于已挂起的 HT 中断时重放数据。 */
void Image_Transmission_DMAEvent(uint16_t position)
{
    const uint16_t capacity=sizeof(Image_Trans_MultiRx_Buff);
    const uint8_t *buffer=(const uint8_t *)Image_Trans_MultiRx_Buff;
    if (position>capacity) { Image_Transmission_StreamReset(); return; }
    if (position==capacity) position=0;
    if (position<dma_read) {
        Image_Transmission_Info_Update(buffer+dma_read,capacity-dma_read);
        dma_read=0;
    }
    if (position>dma_read)
        Image_Transmission_Info_Update(buffer+dma_read,position-dma_read);
    dma_read=position;
}


void VT13_Info_Update(uint8_t *Buff ,VT13_Info_TypeDef *VT13_Info){
	
	   if(Verify_CRC16_Check_Sum(&Buff[0],21) == true){
		 
		    VT13_Info->RC.Channel[0] = ( (Buff[2]) | (Buff[3] << 8) )& 0x07FF;                           
        VT13_Info->RC.Channel[1] = ( (Buff[3] >> 3) | (Buff[4] << 5 ) ) & 0x07FF;                            
        VT13_Info->RC.Channel[2] = ( (Buff[4] >> 6) | (Buff[5] << 2 ) | (Buff[6] << 10) ) & 0x07FF;      
        VT13_Info->RC.Channel[3] = ( (Buff[6] >> 1) | (Buff[7] << 7 ) ) & 0x07FF;                         
			  VT13_Info->RC.Switch     = (  Buff[7] >> 4 ) & 0x03;
			  VT13_Info->RC.Stop       = (  Buff[7] >> 6 ) & 0x01;
			  VT13_Info->RC.Left       = (  Buff[7] >> 7 ) & 0x01;
			  VT13_Info->RC.Right      = (  Buff[8]  ) & 0x01;
			  VT13_Info->RC.Wheel      = ( (Buff[8] >> 1) | (Buff[9] << 7 ) ) & 0x07FF; 
			  VT13_Info->RC.Trigger    = (  Buff[9] >> 4 ) & 0x01;
			 
			  VT13_Info->RC.Channel[0] -= 1024;
		    VT13_Info->RC.Channel[1] -= 1024;
        VT13_Info->RC.Channel[2] -= 1024;
		    VT13_Info->RC.Channel[3] -= 1024;
		    VT13_Info->RC.Wheel      -= 1024;
			 
			  VT13_Info->Mouse.X = (Buff[10]  | (Buff[11] << 8));
			  VT13_Info->Mouse.Y = (Buff[12]  | (Buff[13] << 8));
			  VT13_Info->Mouse.Z = (Buff[14]  | (Buff[15] << 8));
			 
			  VT13_Info->Mouse.Press_L = (Buff[16]) & 0x03;
				VT13_Info->Mouse.Press_R = (Buff[16] >> 2) & 0x03;
				VT13_Info->Mouse.Press_M = (Buff[16] >> 4) & 0x03;

        VT13_Info->Key.V = (Buff[17] | (Buff[18] << 8));
		 }
		 
}


void Robot_Data_to_Custom_(uint8_t *Data){

    for(uint8_t i = 0; i<30; i++){
			
		 Image_Transmission_Info.robot_custom_data.data[i] =  Data[i];
		
		}
    
		HAL_UART_Transmit_DMA(&huart1,Image_Transmission_Info.robot_custom_data.data,30);
}

/**
 * @brief  transform the bit8 to bit16
*/
static int16_t bit8TObit16(uint8_t change_info[2])
{
	union
	{
    int16_t  bit16;
		uint8_t  byte[2];
	}u16val;

  u16val.byte[0] = change_info[0];
  u16val.byte[1] = change_info[1];

	return u16val.bit16;
}