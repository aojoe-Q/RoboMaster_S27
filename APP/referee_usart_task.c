/**
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  * @file       usb_task.c/h
  * @brief      usb outputs the IMU and gimbal data to the miniPC
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023-7-11       Penguin         1. done
  *  V1.0.1     Oct-31-2023     LihanChen       1. ��ɺ��Ŀ�ܵĹ�����ʹ�������ģʽ�� miniPC ģʽ���ݡ�
  *  V1.0.2     Nov-01-2023     LihanChen       1. �� Append_CRC16_Check_Sum_SendPacketVision() �� Append_CRC-16_Check_Stum_OutputPCData() �ϲ��� Append_CRC16_Check_Sum() ��
  *  v2.0.0     Feb-24-2024     LihanChen       1. �ع����룬�� usb_task �ķ��ͺͽ��շ��룬��ʵ�ְַ����ͺͽ����Ӿ��͵�������
  *  v2.0.1     Mar-02-2024     LihanChen       1. ɾ��δʹ�õ� USB_RECEIVE_STATE ���壬ȥ�����뾯��
  *  v2.1.0     Mar-04-2024     LihanChen       1. ���Ӳ���ϵͳ��Ϣת���������ӿڣ���������ʵ��ֵ��
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
*/

#include "referee_usart_task.h"
#include "usb_device.h"
#include "cmsis_os.h"
#include "crc8_crc16.h"
#include "Arm_L_task.h"
// uint8_t usb_rx_buf[APP_RX_DATA_SIZE];
 uint8_t usb_rx_buf[20];
// Constants
#define OUTPUT_VISION_STATE 0
#define OUTPUT_PC_STATE 1
static void usb_receive(void);
unpack_data_t referee_unpack_obj;
#define USB_RECEIVE_LEN 384
extern int8_t CDC_Receive_HS(uint8_t* pbuf, uint32_t *Len);
/* 主手 0x0302 真实来源：USART1 图传解析器（Image_Transmission.c）。
   不直接 include Image_Transmission.h：其 custom_robot_data_t 与
   CUSTOM_ROBOT_DATA_ID 宏和本模块同名定义冲突。 */
extern uint32_t Image_Transmission_Get_Custom(uint8_t data[30], uint32_t *rx_tick);
static void Master_Raw_Update(void);

robot_status_t robot_state;	
custom_robot_data_t custom_robot_data;											/*0X0302*/
Custom_controller_t Custom_RC[6];
master_raw_t Master_Raw;													/*0X0302 原始快照*/
uint8_t Custom_RC_Key1;
uint8_t Custom_RC_Key2;
/**
  * @brief      USB������������switch��Ϊ���ͣ����ղ�����usb_receive()�д���
  * @param[in]  argument: �������
  * @retval     None
  */
	
void referee_usart_Task(void const * argument)
{
//  MX_USB_DEVICE_Init();
  while (1)
  {
    (void)usb_receive;  /* 读取的 uart1_rx_process_buf 无写入者（全零占位），已停用 */
	Master_Raw_Update();
		
    osDelay(10); // Ϊ�˼�����λ���������ݵĸ��������ͷ���Ƶ�ʡ� Delay for 6.67 milliseconds (150Hz)
  }
}

/**
  * @brief      USB�������������������յ���ԭʼ���ݣ���������ͷ�ж��������Ͳ��ְ�����
  * @param      None
  * @retval     None
  */
static void usb_receive(void)
{
	 referee_unpack_fifo_data();
	
  }





/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          ���ֽڽ��
  * @param[in]      void
  * @retval         none
  */
  uint8_t usb_buffer[100];
extern uint8_t uart1_rx_process_buf[512];
void referee_unpack_fifo_data(void)
{
  uint32_t len = 128;  // �����ж�����ճ���

  unpack_data_t *p_obj = &referee_unpack_obj;
 //DC_Receive_HS(usb_rx_buf, &len); // Read data into the buffer
  // ��USB��ȡ����
  if (1)
  {
    // ����ֽڴ���������ԭ��״̬���߼���
    for (int i = 0; i < len; i++)
    {
      uint8_t byte = uart1_rx_process_buf[i];
		
      
    switch(p_obj->unpack_step) 
    {
      case STEP_HEADER_SOF:
      {
        if(byte == HEADER_SOF)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
         if ( verify_crc8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
				
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  
      
      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
if ( verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
          {
           referee_data_solve(p_obj->protocol_packet);
          }

        }
      }break;
  
      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}
	
}







/**
  * @brief      ���ַ���ת��Ϊuint8_t����
  * @param[out] word: ת���������
  * @param[in]  str: ԭʼ�ַ���
  * @retval     None
  */
void char_to_uint(uint8_t *word, const char *str)
{
    int i = 0;
    while (str[i] != '\0' && i < 10)
    {
        word[i] = str[i];
        i++;
    }
}

// CRC Table
const uint16_t W_CRC_TABLE[256] = {
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
  0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
  0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
  0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
  0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
  0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,
  0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a,
  0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
  0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,
  0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
  0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
  0x3de3, 0x2c6a, 0x1ef1, 0x0f78};



frame_header_struct_t referee_receive_header;

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {

						case ROBOT_STATE_CMD_ID:
				{
						memcpy(&robot_state, frame + index, sizeof(robot_state));
				}
			break;
						case CUSTOM_ROBOT_DATA_ID:
				{
						memcpy(&custom_robot_data, frame + index, sizeof(custom_robot_data));
				}
			break;

    default:
    {
        break;
    }
    }
}


// CRC8У�麯��
// ��ʼֵ0xFF, ���������, ����ʽ0x31, ����������ݷ�ת
uint8_t calculate_crc8(const uint8_t *data, uint32_t length)
{
    uint8_t crc = 0xFF;  // ��ʼֵ
    
    for (uint32_t i = 0; i < length; i++) {
        uint8_t byte = data[i];
        
        // �������ݷ�ת
        byte = (byte & 0x55) << 1 | (byte & 0xAA) >> 1;
        byte = (byte & 0x33) << 2 | (byte & 0xCC) >> 2;
        byte = (byte & 0x0F) << 4 | (byte & 0xF0) >> 4;
        
        crc ^= byte;
        
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;  // ����ʽ0x31
            } else {
                crc = crc << 1;
            }
        }
    }
    
    // ������ݷ�ת
    crc = (crc & 0x55) << 1 | (crc & 0xAA) >> 1;
    crc = (crc & 0x33) << 2 | (crc & 0xCC) >> 2;
    crc = (crc & 0x0F) << 4 | (crc & 0xF0) >> 4;
    
    return crc;  // ���������
}

// �򻯵�CRC8���㣨��������̫���ӣ�
uint8_t calculate_crc8_simple(const uint8_t *data, uint32_t length)
{
    uint8_t crc = 0xFF;
    uint8_t i;
    
    while (length--) {
        crc ^= *data++;
        for (i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

// CRC16-ModbusУ�麯��
// ��ʼֵ0xFFFF, ���������, ����ʽ0x8005
uint16_t calculate_crc16_modbus(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0xFFFF;  // ��ʼֵ
    
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;  // 0xA001��0x8005��λ����
            } else {
                crc = crc >> 1;
            }
        }
    }
    
    return crc;  // ���������
}

// ��֤���������ڲ���ʾ�����ݣ�
bool verify_crc8_check_sum(const uint8_t *data, uint32_t length)
{
    if (length < 1) return false;
    
    uint8_t calculated_crc = calculate_crc8(data, length - 1);
    uint8_t received_crc = data[length - 1];
    
    return (calculated_crc == received_crc);
}

bool verify_crc16_check_sum(const uint8_t *data, uint32_t length)
{
    if (length < 2) return false;
    
    uint16_t calculated_crc = calculate_crc16_modbus(data, length - 2);
    uint16_t received_crc = (data[length - 1] << 8) | data[length - 2];
    
    return (calculated_crc == received_crc);
}

/* 从图传链路取最新一帧 0x0302 原始载荷。只在接收计数变化时更新；
   字段语义未确认前（CUSTOM_RC_DECODE_ENABLE=0）不解码进 Custom_RC。 */
static void Master_Raw_Update(void)
{
	uint8_t  data[30];
	uint32_t tick = 0;
	uint32_t count = Image_Transmission_Get_Custom(data, &tick);

	if (count != 0U && count != Master_Raw.rx_count)
	{
		taskENTER_CRITICAL();
		memcpy(Master_Raw.data, data, sizeof(Master_Raw.data));
		Master_Raw.rx_tick  = tick;
		Master_Raw.rx_count = count;
		taskEXIT_CRITICAL();
#if CUSTOM_RC_DECODE_ENABLE
		Custom_RC_data(Master_Raw.data);
#endif
	}
	Master_Raw.online = (uint8_t)((Master_Raw.rx_count != 0U) &&
	                              ((HAL_GetTick() - Master_Raw.rx_tick) < MASTER_RAW_TIMEOUT_MS));
}

/* 按旧 APP 期望格式解码一帧载荷（4 个大端 float、2 个大端 int32、2 个按键字节）。
   该格式尚未用发送端源码或抓包确认。 */
void Custom_RC_data(const uint8_t *data)
{
	// �����յ����ֽ�����ת��Ϊ������
		Custom_RC[0].DM_CC_receive = bytes_to_float(data[0], data[1], data[2], data[3]);
		Custom_RC[1].DM_CC_receive = bytes_to_float(data[4], data[5], data[6], data[7]);
		Custom_RC[2].DM_CC_receive = bytes_to_float(data[8], data[9], data[10], data[11]);
		Custom_RC[3].DM_CC_receive = bytes_to_float(data[12], data[13], data[14], data[15]);
		// ���ֽ������е�����ת��Ϊ int32_t
		Custom_RC[4].DJ_CC_receive = (int32_t)(((uint32_t)data[16] << 24) | 
																 ((uint32_t)data[17] << 16) | 
																 ((uint32_t)data[18] << 8) | 
																 (uint32_t)data[19]);

		// ͬ��������һ�� 32 λ���ݴ��ֽ������н���
		Custom_RC[5].DJ_CC_receive = (int32_t)(((uint32_t)data[20] << 24) | 
																 ((uint32_t)data[21] << 16) | 
																 ((uint32_t)data[22] << 8) | 
																 (uint32_t)data[23]);
	
		Custom_RC_Key1 = data[24];
		Custom_RC_Key2 = data[25];
}


// �� 4 ���ֽ�ת��Ϊһ�� float ���͵�ֵ
float bytes_to_float(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4)
{
    uint32_t intRep = ((uint32_t)byte1 << 24) | ((uint32_t)byte2 << 16) | ((uint32_t)byte3 << 8) | (uint32_t)byte4;
    float value;
    memcpy(&value, &intRep, sizeof(value));  // �� 32 λ����ת��Ϊ float
    return value;
}
