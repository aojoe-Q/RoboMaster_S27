#include "Moter_task.h"
#include "Arm_L_task.h"
#include "DM_motor.h"
#include "DJI_motor.h"
#include "can_bsp.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "uart_bsp.h"
#include "VT13_rc_ctrl.h"
#include "DT7_DR16.h"

  extern FDCAN_HandleTypeDef hfdcan1;
  extern FDCAN_HandleTypeDef hfdcan2;
  extern FDCAN_HandleTypeDef hfdcan3;
  
  extern DT7_RC_ctrl_t DT7_rc_ctrl;    //DT7

/*实时控制电机任务*/
void Moter_Task(void const * argument)   
{
	osDelay(100);
	while(1)
	{
	    /* Arm_L moter*/
		mit_ctrl(&hfdcan1, 0x01, ArmData.J1.send_set, 0, ArmData.J1.Kp, ArmData.J1.Kd, 0);
		osDelay(2);
		mit_ctrl(&hfdcan1, 0x02, ArmData.J2.send_set, 0, ArmData.J2.Kp, ArmData.J2.Kd, 0);
		osDelay(2);
		mit_ctrl(&hfdcan1, 0x03, ArmData.J3.send_set, 0, ArmData.J3.Kp, ArmData.J3.Kd, 0);
		osDelay(2);
		mit_ctrl(&hfdcan1, 0x04, ArmData.J4.send_set, 0, ArmData.J4.Kp, ArmData.J4.Kd, 0);
		osDelay(2);
		mit_ctrl(&hfdcan1, 0x05, ArmData.J5.send_set, 0, ArmData.J5.Kp, ArmData.J5.Kd, 0);
		osDelay(2);
		mit_ctrl(&hfdcan1, 0x06, ArmData.J6.send_set, 0, ArmData.J6.Kp, ArmData.J6.Kd, 0);
		osDelay(2);
		if(switch_is_up(DT7_RIGHT_SW)&&switch_is_mid(DT7_LEFT_SW))
			mit_ctrl(&hfdcan1, 0x07, 0, 0, 0, 0, 0.2);
		else
		{
			mit_ctrl(&hfdcan1, 0x07, ArmData.Grip.send_set, 0, ArmData.Grip.Kp, ArmData.Grip.Kd, 0);
		}
		osDelay(5);
		
		
//		mit_ctrl(&hfdcan1, 0x01, 0, 0, 0, 0, 0);
//		osDelay(2);
//		mit_ctrl(&hfdcan1, 0x02, 0, 0, 0, 0, 0);
//		osDelay(2);
//		mit_ctrl(&hfdcan1, 0x03, 0, 0, 0, 0, 0);
//		osDelay(2);
//		mit_ctrl(&hfdcan1, 0x04, 0, 0, 0, 0, 0);
//		osDelay(2);
//		mit_ctrl(&hfdcan1, 0x05, 0, 0, 0, 0, 0);
//		osDelay(2);
//		mit_ctrl(&hfdcan1, 0x06, 0, 0, 0, 0, 0);
//		osDelay(2);
//		mit_ctrl(&hfdcan1, 0x07, 0, 0, 0, 0, 0);
	}
}
