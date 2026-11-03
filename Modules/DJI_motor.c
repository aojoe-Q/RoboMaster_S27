#include "DJI_motor.h"
#include "can_bsp.h"
#include "fdcan.h"
#include "struct_typedef.h"

//static CAN_TxHeaderTypeDef  chassis_tx_message;
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;


static uint8_t MOTOR_can_send_data[8];
static uint8_t MOTOR_can_send_data1[8];

void CAN_3508_left(int16_t Trigger)
{	

  MOTOR_can_send_data[2] = Trigger >> 8;
  MOTOR_can_send_data[3] = Trigger;  

  canx_send_data(&hfdcan2, CAN_SHOOT_ALL_ID, MOTOR_can_send_data, sizeof(MOTOR_can_send_data));
}


void CAN_3508_right(int16_t Trigger)
{	

  MOTOR_can_send_data1[0] = Trigger >> 8;
  MOTOR_can_send_data1[1] = Trigger;  

  canx_send_data(&hfdcan1, CAN_SHOOT_ALL_ID, MOTOR_can_send_data1, sizeof(MOTOR_can_send_data1));
}


