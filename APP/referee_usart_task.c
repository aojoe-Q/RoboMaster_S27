/**
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  * @file       usb_task.c/h
  * @brief      usb outputs the IMU and gimbal data to the miniPC
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023-7-11       Penguin         1. done
  *  V1.0.1     Oct-31-2023     LihanChen       1. 调整迷你主机模式下的数据处理。
  *  V1.0.2     Nov-01-2023     LihanChen       1. 合并 CRC16 数据发送校验接口。
  *  v2.0.0     Feb-24-2024     LihanChen       1. 重构 USB 任务收发流程，拆分视觉与自定义控制器数据处理。
  *  v2.0.1     Mar-02-2024     LihanChen       1. 删除未使用的 USB_RECEIVE_STATE 定义。
  *  v2.1.0     Mar-04-2024     LihanChen       1. 增加裁判系统信息转发接口并更新反馈数据。
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
*/
/* 模块职责：裁判系统串口（UART1 + DMA）字节流解包：帧同步、帧头 CRC8、整帧 CRC16、命令分发。 */
/* 数据来源：uart1_rx_process_buf（UART1 DMA 空闲中断填充），本任务只消费，不做搬运。 */
/* 调用上下文：referee_usart_Task 任务循环，10 ms 轮询一次；解析在任务上下文完成，不在中断里解析。 */
/* 历史命名：函数与文件仍沿用 usb_ 前缀，实际链路已从 USB CDC 改为 UART1。 */

#include "referee_usart_task.h"
#include "usb_device.h"
#include "cmsis_os.h"
#include "crc8_crc16.h"
#include "Arm_L_task.h"
// uint8_t usb_rx_buf[APP_RX_DATA_SIZE];
 uint8_t usb_rx_buf[20];
// Constants
// 输出目标选择：0 = 视觉/迷你主机链路，1 = PC 调试链路（当前仅保留定义）。
#define OUTPUT_VISION_STATE 0
#define OUTPUT_PC_STATE 1
static void usb_receive(void);
// 全局解包上下文：跨轮询保存半帧状态（步进、缓冲与索引），因此必须常驻。
unpack_data_t referee_unpack_obj;
// 单次 USB 接收缓冲上限（旧 USB 通路遗留，当前未使用）。
#define USB_RECEIVE_LEN 384
// 旧 USB CDC 接收接口声明；实现已不再调用，仅保留以兼容历史代码。
extern int8_t CDC_Receive_HS(uint8_t* pbuf, uint32_t *Len);

robot_status_t robot_state;	
// 解析结果：0x0201 机器人状态、0x0302 自定义控制器原始载荷（30 字节）。
custom_robot_data_t custom_robot_data;											/*0X0302*/
// 自定义控制器 6 路通道：前 4 路 float（DM_CC），后 2 路 int32（DJ_CC）；另有 2 个按键字节。
Custom_controller_t Custom_RC[6];
uint8_t Custom_RC_Key1;
uint8_t Custom_RC_Key2;
/**
  * @brief      解析串口输入并更新自定义控制器数据。
  * @param[in]  argument: 未使用的 FreeRTOS 任务参数。
  * @retval     None
  */
	
/* 任务入口：先解包 UART1 缓冲，再把 0x0302 载荷拆成自定义控制器通道。 */
// 参数：argument 未使用；返回：无（常驻任务）；调用上下文：FreeRTOS 任务。
void referee_usart_Task(void const * argument)
{
//  MX_USB_DEVICE_Init();
  // 主循环：解析一帧 -> 刷新通道 -> 延时；解析失败不会阻塞下一轮。
  while (1)
  {
    usb_receive();
	Custom_RC_data();
		
    osDelay(10); // 每 10 ms 轮询一次，约 100 Hz。
  }
}

/**
  * @brief      接收包装函数：调用 FIFO 解包器处理输入数据。
  * @param      None
  * @retval     None
  */
/* USB 接收包装（历史命名）：现直接转调 UART1 的 FIFO 解包函数。 */
// 参数：无；返回：无。
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
  * @brief          逐字节解包函数。
  * @param[in]      void
  * @retval         none
  */
  // 旧 USB 通路遗留缓冲（本文件内未再引用，保留以维持历史布局）。
  uint8_t usb_buffer[100];
extern uint8_t uart1_rx_process_buf[512];
/* 逐字节解包状态机：从 uart1_rx_process_buf 顺序取字节喂入 6 步状态机。 */
// 参数：无；返回：无；副作用：更新 referee_unpack_obj，校验通过时调用 referee_data_solve()。
void referee_unpack_fifo_data(void)
{
  // 固定遍历 128 字节（= REF_PROTOCOL_FRAME_MAX_SIZE）；真实 DMA 接收长度在此被忽略，属历史写法。
  uint32_t len = 128;  // 遗留实现固定扫描 128 字节缓冲区。

  unpack_data_t *p_obj = &referee_unpack_obj;
 // 说明：DC_Receive_HS（USB CDC 读取）已停用，改用 UART1 DMA 缓冲。
 //DC_Receive_HS(usb_rx_buf, &len); // Read data into the buffer
  // USB CDC 接收已停用，当前使用 UART1 DMA 缓冲区。
  // if (1) 为恒真分支：保留旧代码结构，便于将来切回 USB 通路。
  if (1)
  {
    // 逐字节处理输入数据并推进帧解析状态机。
    for (int i = 0; i < len; i++)
    {
      uint8_t byte = uart1_rx_process_buf[i];
		
      
    // 状态机：SOF -> 长度低/高 -> 序号 -> 帧头 CRC8 -> 数据+CRC16；任一步失配都回到 SOF 重新同步。
    switch(p_obj->unpack_step) 
    {
      case STEP_HEADER_SOF:
      {
        if(byte == HEADER_SOF)  // 帧头固定 0xA5
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;  // 进入长度低字节状态，同时暂存帧头首字节
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;  // 长度低字节在前（小端）
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);  // 长度高字节：data_len = low | (high << 8)
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))  // 长度合法性检查：须 < 128 - 9 = 119 字节，否则复位重同步
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
      // 序号字节不参与校验，仅透传；随后进入帧头 CRC8 校验。
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)  // 帧头累计 5 字节：SOF + len(2) + seq + crc8
        {
         if ( verify_crc8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )  // CRC8 覆盖前 4 字节，通过后才继续收数据
				
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
        if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))  // 整帧目标长度 = 9 + data_len（帧头 5 + cmd_id 2 + 载荷 + CRC16 2）
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
if ( verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))  // 整帧 CRC16（Modbus 多项式）校验，失败则静默丢弃
          {
           referee_data_solve(p_obj->protocol_packet);  // 校验通过才分发，避免脏数据写入业务结构体
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
  * @brief      将字符串复制到字节数组。
  * @param[out] word: 输出字节数组。
  * @param[in]  str: 输入字符串。
  * @retval     None
  */
/* 字符串转定长字节数组：最多拷贝 10 字节，不追加结束符；当前无调用者，保留工具函数。 */
// 参数：word 输出缓冲；str 源字符串；返回：无。
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
// CRC16-Modbus 查表（256 项）；本文件实际使用逐位计算，此表未被引用。
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



// 最近一帧的帧头快照，调试时可直接查看序号与长度。
frame_header_struct_t referee_receive_header;

/* 命令分发：解析 5 字节帧头 + 2 字节 cmd_id，按命令号把载荷拷入对应结构体。 */
// 参数：frame 指向已通过 CRC 校验的完整帧；返回：无；调用上下文：解包状态机（任务上下文）。
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

						case ROBOT_STATE_CMD_ID:  // 0x0201：机器人状态（等级/血量/热量/功率限制）
				{
						memcpy(&robot_state, frame + index, sizeof(robot_state));
				}
			break;
						case CUSTOM_ROBOT_DATA_ID:  // 0x0302：自定义控制器 30 字节载荷
				{
						memcpy(&custom_robot_data, frame + index, sizeof(custom_robot_data));
				}
			break;

    default:  // 未识别命令：忽略，帧完整性已由 CRC 保证
    {
        break;
    }
    }
}


/* CRC8：初值 0xFF、多项式 0x31（反射），输入与输出均按位反转，用于 5 字节帧头校验。 */
// CRC8 帧头校验。
// 初值 0xFF，多项式 0x31，输入和输出按位反转。
// 参数：data 待校验数据；length 字节数；返回：8 位校验值。
uint8_t calculate_crc8(const uint8_t *data, uint32_t length)
{
    uint8_t crc = 0xFF;  // CRC8 初值。
    
    for (uint32_t i = 0; i < length; i++) {
        uint8_t byte = data[i];
        
        // 反转输入字节的位序。
        byte = (byte & 0x55) << 1 | (byte & 0xAA) >> 1;
        byte = (byte & 0x33) << 2 | (byte & 0xCC) >> 2;
        byte = (byte & 0x0F) << 4 | (byte & 0xF0) >> 4;
        
        crc ^= byte;
        
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;  // CRC8 多项式。
            } else {
                crc = crc << 1;
            }
        }
    }
    
    // 反转 CRC 输出位序。
    crc = (crc & 0x55) << 1 | (crc & 0xAA) >> 1;
    crc = (crc & 0x33) << 2 | (crc & 0xCC) >> 2;
    crc = (crc & 0x0F) << 4 | (crc & 0xF0) >> 4;
    
    return crc;  // 返回 CRC8 校验值。
}

// 简化版 CRC8：不进行输入和输出位反转。
// 简化版 CRC8：同一多项式/初值，但省略输入输出位反转；当前无调用者。
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

// CRC16-Modbus 校验。
// 初值 0xFFFF，反射多项式 0xA001（对应 0x8005）。
/* CRC16-Modbus：初值 0xFFFF、反射多项式 0xA001，用于整帧（含 cmd_id 与载荷）校验。 */
// 参数：data/length；返回：16 位校验值。
uint16_t calculate_crc16_modbus(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0xFFFF;  // CRC16 初值。
    
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;  // 0xA001 是 0x8005 的反射形式。
            } else {
                crc = crc >> 1;
            }
        }
    }
    
    return crc;  // 返回 CRC16 校验值。
}

// 校验接收到的数据帧。
/* 帧头 CRC8 校验：末字节为发送端校验值，其余字节参与计算。 */
bool verify_crc8_check_sum(const uint8_t *data, uint32_t length)
{
    if (length < 1) return false;
    
    uint8_t calculated_crc = calculate_crc8(data, length - 1);  // 末字节不参与计算
    uint8_t received_crc = data[length - 1];
    
    return (calculated_crc == received_crc);
}

/* 整帧 CRC16 校验：末尾 2 字节为小端 CRC，前面所有字节参与计算。 */
bool verify_crc16_check_sum(const uint8_t *data, uint32_t length)
{
    if (length < 2) return false;
    
    uint16_t calculated_crc = calculate_crc16_modbus(data, length - 2);
    uint16_t received_crc = (data[length - 1] << 8) | data[length - 2];  // 低字节在前
    
    return (calculated_crc == received_crc);
}

/* 自定义控制器刷新：把 0x0302 的 30 字节载荷拆成 4 路 float + 2 路 int32 + 2 个按键。 */
// 调用上下文：referee_usart_Task 每轮解析后调用；返回：无。
void Custom_RC_data(void)
{
	// 将控制器载荷字节拆分并转换为业务字段。
        // 四个 float 通道（data[0..15]）：大端 IEEE-754，每 4 字节一个。
		Custom_RC[0].DM_CC_receive = bytes_to_float(custom_robot_data.data[0], custom_robot_data.data[1], custom_robot_data.data[2], custom_robot_data.data[3]);
		Custom_RC[1].DM_CC_receive = bytes_to_float(custom_robot_data.data[4], custom_robot_data.data[5], custom_robot_data.data[6], custom_robot_data.data[7]);
		Custom_RC[2].DM_CC_receive = bytes_to_float(custom_robot_data.data[8], custom_robot_data.data[9], custom_robot_data.data[10], custom_robot_data.data[11]);
		Custom_RC[3].DM_CC_receive = bytes_to_float(custom_robot_data.data[12], custom_robot_data.data[13], custom_robot_data.data[14], custom_robot_data.data[15]);
		// 解码两个 int32 通道。
        // 两个 int32 通道（data[16..23]）：大端拼接，用于自定义控制器/大疆量。
		Custom_RC[4].DJ_CC_receive = (int32_t)(custom_robot_data.data[16] << 24) | 
																 (custom_robot_data.data[17] << 16) | 
																 (custom_robot_data.data[18] << 8) | 
																 custom_robot_data.data[19];

		// 解码第二个 int32 通道。
		Custom_RC[5].DJ_CC_receive = (int32_t)(custom_robot_data.data[20] << 24) | 
																 (custom_robot_data.data[21] << 16) | 
																 (custom_robot_data.data[22] << 8) | 
																 custom_robot_data.data[23];
	
        // 两个按键/开关字节（data[24]、data[25]）。
		Custom_RC_Key1 = custom_robot_data.data[24];
		Custom_RC_Key2 = custom_robot_data.data[25];
}


// 将 4 个字节重组成 float。
/* 大端 4 字节转 float：先拼 uint32，再用 memcpy 做位型重解释，避免严格别名问题。 */
float bytes_to_float(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4)
{
    uint32_t intRep = ((uint32_t)byte1 << 24) | ((uint32_t)byte2 << 16) | ((uint32_t)byte3 << 8) | (uint32_t)byte4;  // 大端序：byte1 为最高字节
    float value;
    // 用 memcpy 做位型重解释，规避严格别名问题（上一行已拼出 uint32 位型）。
    memcpy(&value, &intRep, sizeof(value));  // 按位重解释为 float，避免别名违规。
    return value;
}
