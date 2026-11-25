#include <math.h>
#include "HT_motor.h"

#include "motor_encoding.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;



void CanComm_ControlCmd_Left(uint8_t cmd,uint32_t motor_id)
{
    uint8_t buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
    switch(cmd)
    {
			case CMD_MOTOR_MODE:
					buf[7] = 0xFC;
					break;
			
			case CMD_RESET_MODE:
					buf[7] = 0xFD;
			break;
			
			case CMD_ZERO_POSITION:
					buf[7] = 0xFE;
			break;
			
			default:
			return;
    }
    CanTransmit_Left(buf,8,motor_id);
}
void CanComm_ControlCmd_Right(uint8_t cmd,uint32_t motor_id)
{
    uint8_t buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
    switch(cmd)
    {
			case CMD_MOTOR_MODE:
					buf[7] = 0xFC;
					break;
			
			case CMD_RESET_MODE:
					buf[7] = 0xFD;
			break;
			
			case CMD_ZERO_POSITION:
					buf[7] = 0xFE;
			break;
			
			default:
			return; /* 直接返回。 */
    }
    CanTransmit_Right(buf,8,motor_id);
}



/**
  * @brief          send parameter data from CAN1 to control joint motor
  * @param[in]      buf: 8 bytes data, including motor control parameter information
  * @param[in]      len: size of buf
  * @param[in]      motor_id: id of joint motor,0x01~0x04
  * @retval         none
  */

static void CanTransmit_Right(uint8_t *buf, uint8_t len,uint32_t motor_id)
{
	canx_send_data(&hfdcan1, motor_id, buf, len);
}

static void CanTransmit_Left(uint8_t *buf, uint8_t len,uint32_t motor_id)
{
	canx_send_data(&hfdcan2, motor_id, buf, len);
}



void CanComm_SendControlPara_Left(float f_p, float f_v, float f_kp, float f_kd, float f_t,uint32_t motor_id)
{
    if (!isfinite(f_p) || !isfinite(f_v) || !isfinite(f_kp) ||
        !isfinite(f_kd) || !isfinite(f_t)) return;
    uint16_t p, v, kp, kd, t;
    uint8_t buf[8];
    
    LIMIT_MIN_MAX(f_p,  P_MIN,  P_MAX);
    LIMIT_MIN_MAX(f_v,  V_MIN,  V_MAX);
    LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX);
    LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX);
    LIMIT_MIN_MAX(f_t,  T_MIN,  T_MAX);
    
    p = float_to_uint_clamped(f_p,   P_MIN,  P_MAX,  16);
    v = float_to_uint_clamped(f_v,   V_MIN,  V_MAX,  12);
    kp = float_to_uint_clamped(f_kp, KP_MIN, KP_MAX, 12);
    kd = float_to_uint_clamped(f_kd, KD_MIN, KD_MAX, 12);
    t = float_to_uint_clamped(f_t,   T_MIN,  T_MAX,  12);
    
    buf[0] = p>>8;
    buf[1] = p&0xFF;
    buf[2] = v>>4;
    buf[3] = ((v&0xF)<<4)|(kp>>8);
    buf[4] = kp&0xFF;
    buf[5] = kd>>4;
    buf[6] = ((kd&0xF)<<4)|(t>>8);
    buf[7] = t&0xff;
    
    CanTransmit_Left(buf, sizeof(buf),motor_id);
}
void CanComm_SendControlPara_Right(float f_p, float f_v, float f_kp, float f_kd, float f_t,uint32_t motor_id)
{
    if (!isfinite(f_p) || !isfinite(f_v) || !isfinite(f_kp) ||
        !isfinite(f_kd) || !isfinite(f_t)) return;
    uint16_t p, v, kp, kd, t;
    uint8_t buf[8];
    
    LIMIT_MIN_MAX(f_p,  P_MIN,  P_MAX);
    LIMIT_MIN_MAX(f_v,  V_MIN,  V_MAX);
    LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX);
    LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX);
    LIMIT_MIN_MAX(f_t,  T_MIN,  T_MAX);
    
    p = float_to_uint_clamped(f_p,   P_MIN,  P_MAX,  16);
    v = float_to_uint_clamped(f_v,   V_MIN,  V_MAX,  12);
    kp = float_to_uint_clamped(f_kp, KP_MIN, KP_MAX, 12);
    kd = float_to_uint_clamped(f_kd, KD_MIN, KD_MAX, 12);
    t = float_to_uint_clamped(f_t,   T_MIN,  T_MAX,  12);
    
    buf[0] = p>>8;
    buf[1] = p&0xFF;
    buf[2] = v>>4;
    buf[3] = ((v&0xF)<<4)|(kp>>8);
    buf[4] = kp&0xFF;
    buf[5] = kd>>4;
    buf[6] = ((kd&0xF)<<4)|(t>>8);
    buf[7] = t&0xff;
    
    CanTransmit_Right(buf, sizeof(buf),motor_id);
}

