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

#ifndef REFEREE_USART_TASK_H
#define REFEREE_USART_TASK_H

#include <stdint.h>
#include <stdbool.h>
#include "usbd_cdc_if.h"

extern void usb_task(void);
typedef enum
{
    GAME_STATE_CMD_ID                 = 0x0001,
    GAME_RESULT_CMD_ID                = 0x0002,
    GAME_ROBOT_HP_CMD_ID              = 0x0003,
    FIELD_EVENTS_CMD_ID               = 0x0101,
    SUPPLY_PROJECTILE_ACTION_CMD_ID   = 0x0102,
    SUPPLY_PROJECTILE_BOOKING_CMD_ID  = 0x0103,
    REFEREE_WARNING_CMD_ID            = 0x0104,
    DART_INFO_CMD_ID                  = 0x0105,
    ROBOT_STATE_CMD_ID                = 0x0201,
    POWER_HEAT_DATA_CMD_ID            = 0x0202,
    ROBOT_POS_CMD_ID                  = 0x0203,
    BUFF_MUSK_CMD_ID                  = 0x0204,
    AERIAL_ROBOT_ENERGY_CMD_ID        = 0x0205,
    ROBOT_HURT_CMD_ID                 = 0x0206,
    SHOOT_DATA_CMD_ID                 = 0x0207,
    BULLET_REMAINING_CMD_ID           = 0x0208,
    ROBOT_RFID_STATE                  = 0x0209,
    DART_CLIENT_CMD_ID                = 0x020A,
    GROUND_ROBOT_POSITION_CMD_ID      = 0x020B,
    RADAR_MARK_DATA_CMD_ID            = 0x020C,
    SENTRY_INFO_CMD_ID                = 0x020D,
    RADAR_INFO_CMD_ID                 = 0x020E,

    ROBOT_INTERACTION_DATA_CMD_ID     = 0x0301,
		
		CUSTOM_ROBOT_DATA_ID  								=0X0302,
    
    IDCustomData,
}referee_cmd_id_t;
#define CRC16_INIT 0xFFFF
#define AUTO_MODE 0xA5
#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128
#define NAV_ID 0X0001
//#define    ROBOT_STATE_CMD_ID               0x0201,
// Э�鳣������
#define FRAME_HEADER_SIZE 5
#define CMD_ID_SIZE 2
#define FRAME_TAIL_SIZE 2
#define SQF_VALUE 0xA5
#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

// ֡ͷ�ṹ
#pragma pack(push, 1)
typedef struct {
    uint8_t sqf;           // ��ʼ�ֽ� 0xA5
    uint16_t data_length;  // ���ݳ���
    uint8_t seq;           // �����
    uint8_t crc8;          // ֡ͷCRC8У��
} frame_header_t;
#pragma pack(pop)

// ����֡�ṹ
typedef struct {
    frame_header_t header;
    uint16_t cmd_id;       // ����ID
    uint8_t *data;         // �����غ�
    uint16_t crc16;        // ����CRC16У��
} protocol_frame_t;
#pragma pack(push, 1)
typedef struct
{
 uint8_t robot_id;
uint8_t robot_level;
uint16_t current_HP;
 uint16_t maximum_HP;
uint16_t shooter_barrel_cooling_value;
uint16_t shooter_barrel_heat_limit;
uint16_t chassis_power_limit;
 uint8_t power_management_gimbal_output : 1;
uint8_t power_management_chassis_output : 1;
 uint8_t power_management_shooter_output : 1;
}robot_status_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct/*0X0302*/
{
    uint8_t data[30];
} custom_robot_data_t;
#pragma pack(pop)


// ����״̬
typedef enum {
    PARSE_STATE_HEADER,
    PARSE_STATE_CMD_ID,
    PARSE_STATE_DATA,
    PARSE_STATE_TAIL,
    PARSE_STATE_COMPLETE
} parse_state_t;
typedef struct
{

  float vx;
  float vy;
  float vw;

} NAV_t;
#pragma pack(push, 1)  // ��ȷ1�ֽڶ��룬��ֹ���
typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;
#pragma pack(pop)      // �ָ�Ĭ�϶���
typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;
typedef struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

/* 主手 0x0302 原始载荷快照：只在图传解析器接受新帧（cmd_id、长度、CRC 均通过）时更新 */
#define MASTER_RAW_TIMEOUT_MS   200U   /* 超过此时间无新帧视为离线 */
#define CUSTOM_RC_DECODE_ENABLE 0      /* 发送端协议确认前保持 0：不解码进 Custom_RC */
typedef struct
{
  uint8_t  data[30];
  uint32_t rx_tick;    /* 帧被接受时的 HAL tick */
  uint32_t rx_count;   /* 0 = 从未收到 */
  uint8_t  online;
} master_raw_t;
extern master_raw_t Master_Raw;

extern uint8_t Custom_RC_Key1;
extern uint8_t Custom_RC_Key2;
void Custom_RC_data(const uint8_t *data);
extern void referee_unpack_fifo_data(void);
void init_referee_struct_data(void);
void referee_data_solve(uint8_t *frame);
bool verify_crc8_check_sum(const uint8_t *data, uint32_t length);
bool verify_crc16_check_sum(const uint8_t *data, uint32_t length);
float bytes_to_float(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4);

#endif

