#ifndef LK_MOTOR_H
#define LK_MOTOR_H
#include "can_bsp.h"
#include "main.h"
#include "fdcan.h"
#include "struct_typedef.h"

void CAN_cmd_measure(void);
void CAN_cmd_chassis_LK_Left(int16_t motor);
void CAN_cmd_chassis_LK_Right(int16_t motor);

#endif


