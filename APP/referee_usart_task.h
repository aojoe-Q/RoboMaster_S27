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
/* 模块职责：RoboMaster 裁判系统串口协议定义：命令 ID、帧头/帧视图、自定义控制器载荷与解包状态机类型。 */
/* 协议要点：帧头 5 字节（SOF=0xA5、小端长度、序号、CRC8），帧尾 CRC16 小端，单帧上限 128 字节。 */
/* 使用方：referee_usart_task.c 的逐字节解包、CRC 校验与命令分发。 */

#ifndef REFEREE_USART_TASK_H
#define REFEREE_USART_TASK_H

#include <stdint.h>
#include <stdbool.h>
#include "usbd_cdc_if.h"

extern void usb_task(void);
// 裁判系统命令 ID 全表：比赛流程、机器人状态/血量/功率、位置、交互数据等。
typedef enum
{
    GAME_STATE_CMD_ID                 = 0x0001,
    GAME_RESULT_CMD_ID                = 0x0002,
    GAME_ROBOT_HP_CMD_ID              = 0x0003,
    // 0x01xx：场地事件、补给/飞镖等比赛流程事件。
    FIELD_EVENTS_CMD_ID               = 0x0101,
    SUPPLY_PROJECTILE_ACTION_CMD_ID   = 0x0102,
    SUPPLY_PROJECTILE_BOOKING_CMD_ID  = 0x0103,
    REFEREE_WARNING_CMD_ID            = 0x0104,
    DART_INFO_CMD_ID                  = 0x0105,
    // 0x02xx：机器人实时状态（血量、功率与热量、位置、伤害、RFID、雷达等）。
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

    // 0x03xx：交互数据；0x0302 为自定义控制器透传载荷。
    ROBOT_INTERACTION_DATA_CMD_ID     = 0x0301,
		
		CUSTOM_ROBOT_DATA_ID  								=0X0302,  // 自定义控制器 30 字节载荷（本工程主用）
    
    IDCustomData,  // 枚举计数哨兵（未使用）
}referee_cmd_id_t;
// ===== 协议常量（帧格式与长度换算） =====
#define CRC16_INIT 0xFFFF  // CRC16 初值
#define AUTO_MODE 0xA5
#define HEADER_SOF 0xA5  // 帧起始字节 SOF
#define REF_PROTOCOL_FRAME_MAX_SIZE         128  // 含帧头与 CRC 的整帧上限
#define NAV_ID 0X0001
//#define    ROBOT_STATE_CMD_ID               0x0201,
// Э�鳣������
#define FRAME_HEADER_SIZE 5  // SOF(1) + len(2) + seq(1) + crc8(1)
#define CMD_ID_SIZE 2
#define FRAME_TAIL_SIZE 2
#define SQF_VALUE 0xA5
#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)  // 实际取值 5
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))  // 5 + 2 + 2 = 9：数据段解析起点
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))  // 帧头 + cmd_id

// ֡ͷ�ṹ
// 帧头按 1 字节对齐打包，禁止编译器插入填充，保证与线上字节流一致。
#pragma pack(push, 1)
typedef struct {
    uint8_t sqf;           // ��ʼ�ֽ� 0xA5
    uint16_t data_length;  // ���ݳ���
    uint8_t seq;           // �����
    uint8_t crc8;          // ֡ͷCRC8У��
} frame_header_t;
#pragma pack(pop)

// 逻辑帧视图（非线上布局）：描述“帧头 + 命令 + 载荷 + CRC”的组合关系。
// ����֡�ṹ
typedef struct {
    frame_header_t header;
    uint16_t cmd_id;       // ����ID
    uint8_t *data;         // �����غ�
    uint16_t crc16;        // ����CRC16У��
} protocol_frame_t;
// 0x0201 机器人状态载荷：等级/血量/热量/功率限制，以及云台/底盘/发射机构功率输出许可位。
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
 uint8_t power_management_gimbal_output : 1;  // 位域：三个功率输出许可
uint8_t power_management_chassis_output : 1;
 uint8_t power_management_shooter_output : 1;
}robot_status_t;
#pragma pack(pop)

// 0x0302 自定义控制器载荷：固定 30 字节，偏移拆分见 Custom_RC_data()。
#pragma pack(push, 1)
typedef struct/*0X0302*/
{
    uint8_t data[30];
} custom_robot_data_t;
#pragma pack(pop)


// 旧版解析状态枚举；当前解包使用下面的 unpack_step_e，此处保留以兼容历史代码。
// ����״̬
typedef enum {
    PARSE_STATE_HEADER,
    PARSE_STATE_CMD_ID,
    PARSE_STATE_DATA,
    PARSE_STATE_TAIL,
    PARSE_STATE_COMPLETE
} parse_state_t;
// 导航速度结构体（vx/vy/vw），供上层导航接口使用。
typedef struct
{

  float vx;
  float vy;
  float vw;

} NAV_t;
// 线上帧头结构（5 字节）：布局与 frame_header_t 一致，供解包状态机复用。
#pragma pack(push, 1)  // ��ȷ1�ֽڶ��룬��ֹ���
typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;
#pragma pack(pop)      // �ָ�Ĭ�϶���
// 解包状态机步进：帧头 -> 长度低/高 -> 序号 -> 帧头 CRC8 -> 数据 + CRC16。
typedef enum
{
  STEP_HEADER_SOF  = 0,  // 等待 0xA5
  STEP_LENGTH_LOW  = 1,  // 长度低字节
  STEP_LENGTH_HIGH = 2,  // 长度高字节
  STEP_FRAME_SEQ   = 3,  // 包序号
  STEP_HEADER_CRC8 = 4,  // 5 字节帧头 CRC8
  STEP_DATA_CRC16  = 5,  // cmd_id + 载荷 + CRC16
} unpack_step_e;
// 解包上下文：128 字节帧缓冲、已写索引 index、帧头声明的 data_len 与当前步进。
typedef struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;  // 帧头声明的载荷长度
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];  // 整帧缓冲
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

// 对外接口：自定义控制器按键、解包入口、命令分发与 CRC 校验。
extern uint8_t Custom_RC_Key1;
extern uint8_t Custom_RC_Key2;
void Custom_RC_data(void);
extern void referee_unpack_fifo_data(void);
void init_referee_struct_data(void);
void referee_data_solve(uint8_t *frame);
bool verify_crc8_check_sum(const uint8_t *data, uint32_t length);  // 帧头 CRC8 校验
bool verify_crc16_check_sum(const uint8_t *data, uint32_t length);  // 整帧 CRC16 校验
float bytes_to_float(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4);  // 大端 4 字节转 float

#endif

