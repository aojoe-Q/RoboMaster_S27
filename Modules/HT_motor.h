#ifndef HT_MOTOR_H
#define HT_MOTOR_H
#include "main.h"
#include "fdcan.h"
#include "struct_typedef.h"
#include "can_bsp.h"



static void CanTransmit_Left(uint8_t *buf, uint8_t len,uint32_t motor_id);
extern uint16_t float_to_uint_DM(float x, float x_min, float x_max, uint8_t bits);
static void CanTransmit_Right(uint8_t *buf, uint8_t len,uint32_t motor_id);

extern void CanComm_SendControlPara_Left(float f_p, float f_v, float f_kp, float f_kd, float f_t,uint32_t motor_id);

extern void CanComm_ControlCmd_Left(uint8_t cmd,uint32_t motor_id);
extern void CanComm_SendControlPara_Right(float f_p, float f_v, float f_kp, float f_kd, float f_t,uint32_t motor_id);

extern void CanComm_ControlCmd_Right(uint8_t cmd,uint32_t motor_id);

#endif
