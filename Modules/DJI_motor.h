#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "struct_typedef.h"
/**
  * @brief          发送底盘电机控制电流(0x201,0x202)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_3508_left(int16_t Trigger);
extern void CAN_3508_right(int16_t Trigger);

#endif

