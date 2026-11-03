#include "LK_motor.h"
 uint8_t chassis_can1_send_data[8];
 uint8_t chassis_can2_send_data[8];
 uint8_t measure_date[8];
 uint8_t measure = 1;
 
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;


void CAN_cmd_measure(void)
{
	measure_date[0] = measure;
	canx_send_data(&hfdcan3, CAN_MEASURE_ID, measure_date, sizeof(measure_date));
}

void CAN_cmd_chassis_LK_Left(int16_t motor)
{
	chassis_can2_send_data[2] = *((uint8_t *)(&motor) + 0);
  chassis_can2_send_data[3] = *((uint8_t *)(&motor) + 1);
	canx_send_data(&hfdcan2, CAN_CHASSIS_LK_ALL_ID, chassis_can2_send_data, sizeof(chassis_can2_send_data));
}

void CAN_cmd_chassis_LK_Right(int16_t motor)
{

  chassis_can1_send_data[0] = *((uint8_t *)(&motor) + 0);
  chassis_can1_send_data[1] = *((uint8_t *)(&motor) + 1);

  canx_send_data(&hfdcan1, CAN_CHASSIS_LK_ALL_ID, chassis_can1_send_data, sizeof(chassis_can1_send_data));
}
