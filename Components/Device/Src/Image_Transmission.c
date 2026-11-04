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

__attribute__((section (".AXI_SRAM"))) uint8_t Image_Trans_MultiRx_Buff[2][IMAGE_TRANS_RX_BUF_LEN];

Image_Transmission_Info_TypeDef Image_Transmission_Info;
VT13_Info_TypeDef VT13_Info;

static int16_t bit8TObit16(uint8_t change_info[2]);
static void Image_Transmission_Frame_Solve(uint8_t *Frame, uint16_t Length);

/* USART1 byte stream: one frame may be split over two DMA blocks, one block
   may carry several frames. Only the USART1 interrupt touches it. */
static uint8_t  Image_Trans_Stream[IMAGE_TRANS_STREAM_LEN];
static uint16_t Image_Trans_Stream_Len;

#define VT13_FRAME_LENGTH  21U

void Image_Transmission_Info_Update(uint8_t *Buff, uint16_t Size){

	uint16_t index = 0;

	if(Size > IMAGE_TRANS_RX_BUF_LEN){
		Size = IMAGE_TRANS_RX_BUF_LEN;
	}
	/* The kept tail is always shorter than IMAGE_TRANS_RX_LENGTH, so this
	   only triggers if that invariant is broken: restart cleanly. */
	if(Image_Trans_Stream_Len + Size > IMAGE_TRANS_STREAM_LEN){
		Image_Trans_Stream_Len = 0;
		Image_Transmission_Info.overflows++;
	}
	memcpy(&Image_Trans_Stream[Image_Trans_Stream_Len], Buff, Size);
	Image_Trans_Stream_Len += Size;

	while(index < Image_Trans_Stream_Len){

		uint8_t *p = &Image_Trans_Stream[index];
		uint16_t avail = Image_Trans_Stream_Len - index;

		/*Check the header frame */
		if(p[0] == 0xA5){

			/* SOF, data_length (LSB first), seq, CRC8 */
			if(avail < FrameHeader_Length){
				break;
			}
			if(Verify_CRC8_Check_Sum(p,FrameHeader_Length) != true){
				Image_Transmission_Info.crc8_errors++;
				index++;
				continue;
			}
			uint16_t frame_len = (uint16_t)(p[2]<<8 | p[1]) + FrameHeader_Length + CMDID_Length + CRC16_Length;
			if(frame_len > IMAGE_TRANS_RX_LENGTH){
				/* No accepted frame is longer than 39 bytes: resync */
				Image_Transmission_Info.unknown_frames++;
				index++;
				continue;
			}
			if(avail < frame_len){
				break;    /* the rest arrives with the next DMA block */
			}
			if(Verify_CRC16_Check_Sum(p,frame_len) != true){
				Image_Transmission_Info.crc16_errors++;
				index++;
				continue;
			}
			Image_Transmission_Frame_Solve(p,frame_len);
			index += frame_len;

		}else if(p[0] == 0xA9){

			if(avail < 2){
				break;
			}
			if(p[1] != 0x53){
				Image_Transmission_Info.resync_bytes++;
				index++;
				continue;
			}
			if(avail < VT13_FRAME_LENGTH){
				break;
			}
			if(Verify_CRC16_Check_Sum(p,VT13_FRAME_LENGTH) != true){
				Image_Transmission_Info.crc16_errors++;
				index++;
				continue;
			}
			VT13_Info_Update(p,&VT13_Info);
			index += VT13_FRAME_LENGTH;

		}else{
			Image_Transmission_Info.resync_bytes++;
			index++;
		}
	}

	/* Keep the unparsed tail (part of one frame) for the next block */
	Image_Trans_Stream_Len -= index;
	if(Image_Trans_Stream_Len > 0 && index > 0){
		memmove(Image_Trans_Stream,&Image_Trans_Stream[index],Image_Trans_Stream_Len);
	}
}

/* One frame that passed CRC8 and CRC16. Accept it only if cmd_id and length
   match: a frame of the right length with another cmd_id is not 0x0302. */
static void Image_Transmission_Frame_Solve(uint8_t *Frame, uint16_t Length){

	uint16_t cmd_id = (uint16_t)(Frame[FrameHeader_Length+1]<<8 | Frame[FrameHeader_Length]);
	uint8_t *data   = &Frame[FrameHeader_Length + CMDID_Length];

	Image_Transmission_Info.Index = 0;
	Image_Transmission_Info.DataLength = Length;

 #ifdef CUSTOM_ROBOT_DATA_ID
	if(cmd_id == CUSTOM_ROBOT_DATA_ID && Length == 39){
		for(uint8_t i = 0; i< 30; i++){
			Image_Transmission_Info.custom_robot_data.data[i] = data[i];
		}
		Image_Transmission_Info.custom_rx_tick = HAL_GetTick();
		Image_Transmission_Info.custom_rx_count++;
		return;
	}
 #endif

 #ifdef REMOTE_CONTROL_ID
	if(cmd_id == REMOTE_CONTROL_ID && Length == 21){
		Image_Transmission_Info.remote_control.mouse_x = bit8TObit16(&data[0]);
		Image_Transmission_Info.remote_control.mouse_y = bit8TObit16(&data[2]);
		Image_Transmission_Info.remote_control.mouse_z = bit8TObit16(&data[4]);
		Image_Transmission_Info.remote_control.left_button_down  = data[6];
		Image_Transmission_Info.remote_control.right_button_down = data[7];
		Image_Transmission_Info.remote_control.Key.keyboard_value = bit8TObit16(&data[8]);
		Image_Transmission_Info.remote_rx_tick = HAL_GetTick();
		Image_Transmission_Info.remote_rx_count++;
		return;
	}
 #endif

	Image_Transmission_Info.unknown_frames++;
}

uint32_t Image_Transmission_Get_Custom(uint8_t data[30], uint32_t *rx_tick){

	uint32_t primask = __get_PRIMASK();
	uint32_t count;

	__disable_irq();
	memcpy(data,Image_Transmission_Info.custom_robot_data.data,30);
	count = Image_Transmission_Info.custom_rx_count;
	if(rx_tick != NULL){
		*rx_tick = Image_Transmission_Info.custom_rx_tick;
	}
	__set_PRIMASK(primask);

	return count;
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