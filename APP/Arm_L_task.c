#include "Arm_L_task.h"
#include "DM_motor.h"
#include "DJI_motor.h"
#include "can_bsp.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "uart_bsp.h"
#include "VT13_rc_ctrl.h"
#include "DT7_DR16.h"
#include "LK_motor.h"
#include "referee_usart_task.h"
#include "math.h"
/**
 * @brief �������ź�Ӧ���������Ƶĺ궨�塣
 *
 * ��������źŵľ���ֵ���ڸ�����������ֵ��dealine����������źŵ��������źţ�
 * ��������ź�Ϊ�㡣���ڷ�ֹС���ȵ����벨����ϵͳ��������Ҫ����Ӧ��
 *
 * @param input �����źš�
 * @param output ����źţ���������������Ľ����
 * @param dealine ������ֵ��
 */
#define rc_deadband_limit(input, output, dealine)    \
  {                                                  \
    if ((input) > (dealine) || (input) < -(dealine)) \
    {                                                \
      (output) = (input);                            \
    }                                                \
    else                                             \
    {                                                \
      (output) = 0;                                  \
    }                                                \
  }
  
  extern FDCAN_HandleTypeDef hfdcan1;
  extern FDCAN_HandleTypeDef hfdcan2;
  extern FDCAN_HandleTypeDef hfdcan3;
  
  extern DT7_RC_ctrl_t DT7_rc_ctrl;    //DT7
  extern VT13_RC_ctrl_t VT13_rc_ctrl;  //VT13

  ramp_function_source_t J1_L_position_set;      //DM_4340p  
  ramp_function_source_t J2_L_position_set;	     //DM_J6248p
  ramp_function_source_t J3_L_position_set;      //DM_8009
  ramp_function_source_t J4_L_position_set;      //DM_4310P
  ramp_function_source_t J5_L_position_set;      //DM_4310P
  ramp_function_source_t J6_L_position_set;      //DM_4310P
  ramp_function_source_t grip_L_position_set;    //DM_4310
  
  ramp_function_source_t J1_L_top_set;      //DM_4340p  
  ramp_function_source_t J2_L_top_set;	     //DM_J6248p
  ramp_function_source_t J3_L_top_set;      //DM_8009
  ramp_function_source_t J4_L_top_set;      //DM_4310P
  ramp_function_source_t J5_L_top_set;      //DM_4310P
  ramp_function_source_t J6_L_top_set;      //DM_4310P
  ramp_function_source_t grip_L_top_set;    //DM_4310
  
  /*һ����תJ4(roll)*/
  ramp_function_source_t J4_L_key2_1_set;     
  ramp_function_source_t J4_L_key2_0_set;    
  
  ButtonCounter left_button_btn;  
  ButtonCounter right_button_btn;
  ArmData_t ArmData;
  RC_contorl_t RC_contorl;
  
  joint_motor_measure_t joint_L_motor[7];

  
  
  /*��ʼ��̬*/
  float J1_L_initial_posture = 0.0,J2_L_initial_posture = 0.0f,
        J3_L_initial_posture = 0.0f,J4_L_initial_posture = 0.0f,
        J5_L_initial_posture = 0.0f,J6_L_initial_posture = 0.43354702f,
        grip_L_initial_posture = 0.0f;
  fp32 J1_send_set, J2_send_set, J3_send_set, J4_send_set, J5_send_set, J6_send_set, Grip_send_set,
       J1_set_initial_data, J2_set_initial_data, J3_set_initial_data, J4_set_initial_data, J5_set_initial_data, J6_set_initial_data, Grip_set_initial_data;
  fp32 grip_pawl_flag = 1,J1_J2_J3_J4_J5_J6_J7_init_flag = 1;
  fp32 slackawl_flag = 1;
  uint8_t left_button_stop_state, right_button_stop_state;
  uint32_t left_button_press_count, right_button_press_count;
  fp32 limit_data5,limit_data6;
  float Init_position;
  float Last_position;
  /*���ؽ�б������*/
  double J1_ramp=0,J2_ramp=0,J3_ramp=0,J4_ramp=0,J5_ramp=0,J6_ramp=0,Grip_ramp=0;
  
  int i=1;//���ԣ�

void Arm_L_Task(void const * argument)
{
	static uint8_t prev_ArmMode = 0xFF;
	Arm_L_Init();
	while (1)
	{
		data_updata();

		/* ��⵽��ģʽ�л����ɵ�ǰλ�ó�ʼ�� RC_Sum_Value���ر����룩����ֹλ����Ծ */
		if (ArmData.ArmMode != prev_ArmMode)
		{
			if (ArmData.ArmMode == ARM_L_MOVEMENT_J1_J4)
			{
				ArmData.J1.RC_Sum_Value = ArmData.J1.position - J1_zero_position;
				ArmData.J2.RC_Sum_Value = ArmData.J2.position - J2_zero_position;
				ArmData.J3.RC_Sum_Value = ArmData.J3.position - J3_zero_position;
				ArmData.J4.RC_Sum_Value = ArmData.J4.position - J4_zero_position;
				ArmData.J5.RC_Sum_Value = ArmData.J5.position - J5_zero_position - Custom_RC[4].DM_CC_receive - ArmData.J5.Key_Control_Value;
				ArmData.J6.RC_Sum_Value = ArmData.J6.position - J6_zero_position - Custom_RC[5].DM_CC_receive - ArmData.J6.Key_Control_Value;
				ArmData.Grip.RC_Sum_Value = ArmData.Grip.position - grip_zero_position;
			}
			prev_ArmMode = ArmData.ArmMode;
		}

		switch (ArmData.ArmMode)
        {
//					
//			case TAKE_WATER:
//				TAKE_WATER_1();
//				break;
			case grip_homing:
				grip_homing_1();
				break;
//			case ARM_RESET:
//				ARM_RESET_1();
//				break;
//			case TAKE_Grip:
//				TAKE_Grip_1();
//			break;
			case ARM_L_MOVEMENT_J1_J4:
				Arm_L_Movement_J1_J4();
			break;

			case ARM_POWERLESS:
				Arm_Powerless();
			break;
			// Ĭ�������ֹͣ���˶�
			default:
				Arm_Powerless();
            break;
        }
		osDelay(8);
		
	}
}
	
  

/************************* ��ʼ�� *************************/
void Arm_L_Init(void)
{
	osDelay(750);
	button_init(&left_button_btn);
	button_init(&right_button_btn);
	
	/*�ؽڵ��ʹ��*/
	enable_motor_mode(&hfdcan1, 0x01, 0);
	osDelay(5);
	enable_motor_mode(&hfdcan1, 0x02, 0);
	osDelay(10);
	enable_motor_mode(&hfdcan1, 0x03, 0);
	osDelay(10);
	enable_motor_mode(&hfdcan1, 0x04, 0);
	osDelay(5);
	enable_motor_mode(&hfdcan1, 0x05, 0);
	osDelay(5);
	enable_motor_mode(&hfdcan1, 0x06, 0);
	osDelay(25);
	enable_motor_mode(&hfdcan1, 0x07, 0);
	osDelay(25);
	
	
//	save_motor_zero(&hfdcan1, 0X01, 0);
//	osDelay(5);
//	save_motor_zero(&hfdcan1, 0X02, 0);
//	osDelay(5);
//	save_motor_zero(&hfdcan1, 0X03, 0);
//	osDelay(5);
//	save_motor_zero(&hfdcan1, 0X04, 0);
//	osDelay(5);
//	save_motor_zero(&hfdcan1, 0X05, 0);
//	osDelay(5);
//	save_motor_zero(&hfdcan1, 0X07, 0);
//	osDelay(5);
	
	//���ֵ��λ��
	ArmData.J1.send_set=joint_L_motor[0].position;
	ArmData.J2.send_set=joint_L_motor[1].position;
	ArmData.J3.send_set=joint_L_motor[2].position;
	ArmData.J4.send_set=joint_L_motor[3].position;
	ArmData.J5.send_set=joint_L_motor[4].position;
	ArmData.J6.send_set=joint_L_motor[5].position;
	ArmData.Grip.send_set=joint_L_motor[6].position;
	vTaskDelay(3600);
	ArmData.J1.position=joint_L_motor[0].position;
	ArmData.J2.position=joint_L_motor[1].position;
	ArmData.J3.position=joint_L_motor[2].position;
	ArmData.J4.position=joint_L_motor[3].position;
	ArmData.J5.position=joint_L_motor[4].position;
	ArmData.J6.position=joint_L_motor[5].position;
	ArmData.Grip.position=joint_L_motor[6].position;

	ArmData.J1.send_set=joint_L_motor[0].position;
	ArmData.J2.send_set=joint_L_motor[1].position;
	ArmData.J3.send_set=joint_L_motor[2].position;
	ArmData.J4.send_set=joint_L_motor[3].position;
	ArmData.J5.send_set=joint_L_motor[4].position;
	ArmData.J6.send_set=joint_L_motor[5].position;
	ArmData.Grip.send_set=joint_L_motor[6].position;

	ArmData.J1.Initial_Total_position=ArmData.J1.position;
	ArmData.J2.Initial_Total_position=ArmData.J2.position;	
	ArmData.J3.Initial_Total_position=ArmData.J3.position;
	ArmData.J4.Initial_Total_position=ArmData.J4.position;	
	ArmData.J5.Initial_Total_position=ArmData.J5.position;
	ArmData.J6.Initial_Total_position=ArmData.J6.position;	 
	ArmData.Grip.Initial_Total_position=ArmData.Grip.position;	
	
//	RC_contorl.RC = get_RC_measure_point();  //VT13_rc_ctrl
	

	arm_data_ramp_init(&J1_L_position_set,1,ArmData.J1.Initial_Total_position);	
	arm_data_ramp_init(&J2_L_position_set,1,ArmData.J2.Initial_Total_position); 
	arm_data_ramp_init(&J3_L_position_set,1,ArmData.J3.Initial_Total_position);	
	arm_data_ramp_init(&J4_L_position_set,1,ArmData.J4.Initial_Total_position); 
	arm_data_ramp_init(&J5_L_position_set,1,ArmData.J5.Initial_Total_position);	
	arm_data_ramp_init(&J6_L_position_set,1,ArmData.J6.Initial_Total_position);
	arm_data_ramp_init(&grip_L_position_set,1,ArmData.Grip.Initial_Total_position);
	
	arm_data_ramp_init(&J4_L_key2_1_set,1,ArmData.J4.Initial_Total_position);
	arm_data_ramp_init(&J4_L_key2_0_set,1,ArmData.J4.Initial_Total_position);
	
	ArmData.J1.Kp = J1_PID_KP;ArmData.J1.Kd = J1_PID_KD;
	ArmData.J2.Kp = J2_PID_KP;ArmData.J2.Kd = J2_PID_KD;
	ArmData.J3.Kp = J3_PID_KP;ArmData.J3.Kd = J3_PID_KD;
	ArmData.J4.Kp = J4_PID_KP;ArmData.J4.Kd = J4_PID_KD;
	ArmData.J5.Kp = J5_PID_KP;ArmData.J5.Kd = J5_PID_KD;
	ArmData.J6.Kp = J6_PID_KP;ArmData.J6.Kd = J6_PID_KD;
	ArmData.Grip.Kp = Grip_PID_KP;ArmData.Grip.Kd = Grip_PID_KD;

do{	
	arm_ramp_calc(&J1_L_position_set,0.0038,J1_zero_position);
	arm_ramp_calc(&J2_L_position_set,0.0035,J2_zero_position);
	arm_ramp_calc(&J3_L_position_set,0.0035,J3_zero_position);
	arm_ramp_calc(&J4_L_position_set,0.0035,J4_zero_position);
	arm_ramp_calc(&J5_L_position_set,0.0035,J5_zero_position);
	arm_ramp_calc(&J6_L_position_set,0.0035,J6_zero_position);
	arm_ramp_calc(&grip_L_position_set,0.0035,grip_zero_position);
	
	ArmData.J1.send_set = J1_L_position_set.out;
	ArmData.J2.send_set = J2_L_position_set.out;
	ArmData.J3.send_set = J3_L_position_set.out;
	ArmData.J4.send_set = J4_L_position_set.out;
	ArmData.J5.send_set = J5_L_position_set.out;
	ArmData.J6.send_set = J6_L_position_set.out;
	ArmData.Grip.send_set = grip_L_position_set.out;
	
	limitPosition(&ArmData.J1.send_set,J1_Max_position,J1_Min_position);
	limitPosition(&ArmData.J2.send_set,J2_Max_position,J2_Min_position);
	limitPosition(&ArmData.J3.send_set,J3_Max_position,J3_Min_position);
	limitPosition(&ArmData.J4.send_set,J4_Max_position,J4_Min_position);
	limitPosition(&ArmData.J5.send_set,J5_Max_position,J5_Min_position);
	limitPosition(&ArmData.J6.send_set,J6_Max_position,J6_Min_position);
	limitPosition(&ArmData.Grip.send_set,Grip_Max_position,Grip_Min_position);

	//�ж��Ƿ�ص���ʼ��̬
	if(J1_L_position_set.out == J1_zero_position && J2_L_position_set.out == J2_zero_position && J3_L_position_set.out == J3_zero_position && J4_L_position_set.out == J4_zero_position && J5_L_position_set.out == J5_zero_position && J6_L_position_set.out == J6_zero_position && grip_L_position_set.out == grip_zero_position)
	{
			J1_J2_J3_J4_J5_J6_J7_init_flag = 0;
	}
	osDelay(5);
}while(J1_J2_J3_J4_J5_J6_J7_init_flag);


	/*��������*/
	ArmData.J1.RC_Sum_Value = 0;
	ArmData.J2.RC_Sum_Value = 0;
	ArmData.J3.RC_Sum_Value = 0;
	ArmData.J4.RC_Sum_Value = 0;
	ArmData.J5.RC_Sum_Value = 0;
	ArmData.J6.RC_Sum_Value = 0;
	ArmData.Grip.RC_Sum_Value = 0;

	ArmData.J1.Key_Control_Value = 0;
	ArmData.J2.Key_Control_Value = 0;
	ArmData.J3.Key_Control_Value = 0;
	ArmData.J4.Key_Control_Value = 0;
	ArmData.J5.Key_Control_Value = 0;
	ArmData.J6.Key_Control_Value = 0;
	ArmData.Grip.Key_Control_Value = 0;

    arm_data_ramp_init(&J1_L_position_set,1,0);	
	arm_data_ramp_init(&J2_L_position_set,1,0); 
	arm_data_ramp_init(&J3_L_position_set,1,J3_zero_position);	
	arm_data_ramp_init(&J4_L_position_set,1,0); 
	arm_data_ramp_init(&J5_L_position_set,1,0);	
	arm_data_ramp_init(&J6_L_position_set,1,0);
	arm_data_ramp_init(&grip_L_position_set,1,0);
	vTaskDelay(10);
}

/************************* ���ݸ��� *************************/
void data_updata(void)
{
//	RC_contorl.RC = get_RC_measure_point();
//	left_button_stop_state = RC_contorl.RC->rc.left_button;
//	right_button_stop_state = RC_contorl.RC->rc.right_button;
//	// ���°���������
//    button_update(&left_button_btn, left_button_stop_state);
//	button_update(&right_button_btn, right_button_stop_state);
//	// ��ȡ��������
//    left_button_press_count = button_get_count(&left_button_btn);
//	right_button_press_count = button_get_count(&right_button_btn);
	
	ArmData.ArmMode = GetArmMode();
	
	ArmData.J1.position = joint_L_motor[0].position;
	ArmData.J2.position = joint_L_motor[1].position;
	ArmData.J3.position = joint_L_motor[2].position;
	ArmData.J4.position = joint_L_motor[3].position;
	ArmData.J5.position = joint_L_motor[4].position;
	ArmData.J6.position = joint_L_motor[5].position;
	ArmData.Grip.position = joint_L_motor[6].position;
}

/************************* ģʽ�ж� *************************/
uint8_t GetArmMode(void)
{
    uint8_t mode = 0;

    // �жϾ���ģʽ
	if(switch_is_up(RCData.rc.s[RC_Right])&&switch_is_up(RCData.rc.s[RC_Left]))  //left_button_press_count % 2 != 0&&RC_contorl.RC->rc.mode_sw == 1
	{
			mode = TAKE_WATER;
	}
	else if (switch_is_down(RCData.rc.s[RC_Right])&&switch_is_mid(RCData.rc.s[RC_Left]))  //left_button_press_count % 2 != 0&&RC_contorl.RC->rc.mode_sw == 0
	{
			mode = grip_homing;
	}
//	else if (switch_is_mid(RCData.rc.s[RC_Right])&&switch_is_mid(RCData.rc.s[RC_Left]))  //RC_contorl.RC->rc.shutter == 1  //switch_is_up(RCData.rc.s[RC_Right])&&switch_is_up(RCData.rc.s[RC_Left])
//	{
//			mode = ARM_RESET;
//	}
//	else if (switch_is_mid(RCData.rc.s[RC_Right])&&switch_is_down(RCData.rc.s[RC_Left]))  //left_button_press_count % 2 != 0&&RC_contorl.RC->rc.mode_sw == 2
//	{
//			mode = PASS_WATER;/*�ع��ʼ��̬*/
//	}
		else if (switch_is_down(RCData.rc.s[RC_Right])&&switch_is_up(RCData.rc.s[RC_Left]))  //left_button_press_count % 2 != 0&&RC_contorl.RC->rc.mode_sw == 2
	{
			mode = ARM_L_MOVEMENT_J1_J4;/*�ع��ʼ��̬*/
	}

	
//	else if (left_button_press_count % 2 == 0&&right_button_press_count % 2 == 0)  //left_button_press_count % 2 == 0&&right_button_press_count % 2 == 0
//	{
//			mode = CHASSIS_MOVEMENT_ARM_KEEP;
//	}
//	else if (RC_contorl.RC->rc.shutter == 1)  //RC_contorl.RC->rc.shutter == 1
//	{
//			mode = ARM_POWERLESS;
//	}
	else if (switch_is_down(RCData.rc.s[RC_Right])&&switch_is_down(RCData.rc.s[RC_Left]))  //RC_contorl.RC->rc.shutter == 1
	{
			mode = ARM_POWERLESS;
	}
	
	else
	{
			mode = ARM_POWERLESS;
	}
    return mode;
}

/*************************** ģʽ ****************************/
/*  ����ģʽ  */
void TAKE_Grip_1(void)
{
	ArmData.J1.Kp = J1_PID_KP;ArmData.J1.Kd = J1_PID_KD;
	ArmData.J2.Kp = J2_PID_KP;ArmData.J2.Kd = J2_PID_KD;
	ArmData.J3.Kp = J3_PID_KP;ArmData.J3.Kd = J3_PID_KD;
	ArmData.J4.Kp = J4_PID_KP;ArmData.J4.Kd = J4_PID_KD;
	ArmData.J5.Kp = J5_PID_KP;ArmData.J5.Kd = J5_PID_KD;
	ArmData.J6.Kp = J6_PID_KP;ArmData.J6.Kd = J6_PID_KD;
	ArmData.Grip.Kp = J6_PID_KP;ArmData.Grip.Kd = J6_PID_KD;

	J1_send_set = J1_reset_position;
	J2_send_set = J2_reset_position;
	J3_send_set = J3_reset_position;
	J4_send_set = J4_reset_position;
	J5_send_set = J5_reset_position;
	J6_send_set = J6_reset_position;
	Grip_send_set = 0;

	J1_ramp= 0.0090,J2_ramp= 0.003,J3_ramp= 0.0045,J4_ramp= 0.0042,J5_ramp= 0.003,J6_ramp= 0.003,Grip_ramp= 0.030;  //������

	/*��׶�һ���������ص�λ���ٱպ���צȡˮ*/

	/*��λ*/
	limitPosition(&J1_send_set,J1_Max_position,J1_Min_position);
	limitPosition(&J2_send_set,J2_Max_position,J2_Min_position);
	limitPosition(&J3_send_set,J3_Max_position,J3_Min_position);
	limitPosition(&J4_send_set,J4_Max_position,J4_Min_position);
	limitPosition(&J5_send_set,J5_Max_position,J5_Min_position);
	limitPosition(&J6_send_set,J6_Max_position,J6_Min_position);
	limitPosition(&Grip_send_set,Grip_Max_position,Grip_Min_position);

	/*б�º���*/
	arm_ramp_calc(&J1_L_position_set,J1_ramp,J1_send_set);
	arm_ramp_calc(&J2_L_position_set,J2_ramp,J2_send_set);
	arm_ramp_calc(&J3_L_position_set,J3_ramp,J3_send_set);
	arm_ramp_calc(&J4_L_position_set,J4_ramp,J4_send_set);
	arm_ramp_calc(&J5_L_position_set,J5_ramp,J5_send_set);
	arm_ramp_calc(&J6_L_position_set,J6_ramp,J6_send_set);
	arm_ramp_calc(&grip_L_position_set,Grip_ramp,Grip_send_set);

	/*���λ�ø�ֵ*/
	ArmData.J1.send_set = J1_L_position_set.out;
	ArmData.J2.send_set = J2_L_position_set.out;
	ArmData.J3.send_set = J3_L_position_set.out;
	ArmData.J4.send_set = J4_L_position_set.out;
	ArmData.J5.send_set = J5_L_position_set.out;
	ArmData.J6.send_set = J6_L_position_set.out;
	ArmData.Grip.send_set = grip_L_position_set.out;

}

void ARM_RESET_1(void)
{
	ArmData.J1.Kp = J1_PID_KP;ArmData.J1.Kd = J1_PID_KD;
	ArmData.J2.Kp = J2_PID_KP;ArmData.J2.Kd = J2_PID_KD;
	ArmData.J3.Kp = J3_PID_KP;ArmData.J3.Kd = J3_PID_KD;
	ArmData.J4.Kp = J4_PID_KP;ArmData.J4.Kd = J4_PID_KD;
	ArmData.J5.Kp = J5_PID_KP;ArmData.J5.Kd = J5_PID_KD;
	ArmData.J6.Kp = J6_PID_KP;ArmData.J6.Kd = J6_PID_KD;
	ArmData.Grip.Kp = J6_PID_KP;ArmData.Grip.Kd = J6_PID_KD;

	J1_send_set = J1_reset_position;
	J2_send_set = J2_reset_position;
	J3_send_set = J3_reset_position;
	J4_send_set = J4_reset_position;
	J5_send_set = J5_reset_position;
	J6_send_set = J6_reset_position;
	Grip_send_set = Grip_TAKE_position_1;

	J1_ramp= 0.0090,J2_ramp= 0.003,J3_ramp= 0.0045,J4_ramp= 0.0042,J5_ramp= 0.003,J6_ramp= 0.003,Grip_ramp= 0.030;  //������

	/*��׶�һ���������ص�λ���ٱպ���צȡˮ*/

	/*��λ*/
	limitPosition(&J1_send_set,J1_Max_position,J1_Min_position);
	limitPosition(&J2_send_set,J2_Max_position,J2_Min_position);
	limitPosition(&J3_send_set,J3_Max_position,J3_Min_position);
	limitPosition(&J4_send_set,J4_Max_position,J4_Min_position);
	limitPosition(&J5_send_set,J5_Max_position,J5_Min_position);
	limitPosition(&J6_send_set,J6_Max_position,J6_Min_position);
	limitPosition(&Grip_send_set,Grip_Max_position,Grip_Min_position);

	/*б�º���*/
	arm_ramp_calc(&J1_L_position_set,J1_ramp,J1_send_set);
	arm_ramp_calc(&J2_L_position_set,J2_ramp,J2_send_set);
	arm_ramp_calc(&J3_L_position_set,J3_ramp,J3_send_set);
	arm_ramp_calc(&J4_L_position_set,J4_ramp,J4_send_set);
	arm_ramp_calc(&J5_L_position_set,J5_ramp,J5_send_set);
	arm_ramp_calc(&J6_L_position_set,J6_ramp,J6_send_set);
	arm_ramp_calc(&grip_L_position_set,Grip_ramp,Grip_send_set);

	/*���λ�ø�ֵ*/
	ArmData.J1.send_set = J1_L_position_set.out;
	ArmData.J2.send_set = J2_L_position_set.out;
	ArmData.J3.send_set = J3_L_position_set.out;
	ArmData.J4.send_set = J4_L_position_set.out;
	ArmData.J5.send_set = J5_L_position_set.out;
	ArmData.J6.send_set = J6_L_position_set.out;
	ArmData.Grip.send_set = grip_L_position_set.out;

}

void grip_homing_1(void)
{
	ArmData.J1.Kp = J1_PID_KP;ArmData.J1.Kd = J1_PID_KD;
	ArmData.J2.Kp = J2_PID_KP;ArmData.J2.Kd = J2_PID_KD;
	ArmData.J3.Kp = J3_PID_KP;ArmData.J3.Kd = J3_PID_KD;
	ArmData.J4.Kp = J4_PID_KP;ArmData.J4.Kd = J4_PID_KD;
	ArmData.J5.Kp = J5_PID_KP;ArmData.J5.Kd = J5_PID_KD;
	ArmData.J6.Kp = J6_PID_KP;ArmData.J6.Kd = J6_PID_KD;
	ArmData.Grip.Kp = J6_PID_KP;ArmData.Grip.Kd = J6_PID_KD;

	J1_send_set = J1_TAKE_position;
	J2_send_set = J2_TAKE_position;
	J3_send_set = J3_TAKE_position;
	J4_send_set = J4_TAKE_position;
	J5_send_set = J5_TAKE_position;
	J6_send_set = J6_TAKE_position;
	Grip_send_set = Grip_TAKE_position_1;

	J1_ramp= 0.0090,J2_ramp= 0.003,J3_ramp= 0.0045,J4_ramp= 0.0042,J5_ramp= 0.003,J6_ramp= 0.003,Grip_ramp= 0.090;  

	
	limitPosition(&J1_send_set,J1_Max_position,J1_Min_position);
	limitPosition(&J2_send_set,J2_Max_position,J2_Min_position);
	limitPosition(&J3_send_set,J3_Max_position,J3_Min_position);
	limitPosition(&J4_send_set,J4_Max_position,J4_Min_position);
	limitPosition(&J5_send_set,J5_Max_position,J5_Min_position);
	limitPosition(&J6_send_set,J6_Max_position,J6_Min_position);
	limitPosition(&Grip_send_set,Grip_Max_position,Grip_Min_position);

	/*斜坡函数*/
	arm_ramp_calc(&J1_L_position_set,J1_ramp,J1_send_set);
	arm_ramp_calc(&J2_L_position_set,J2_ramp,J2_send_set);
	arm_ramp_calc(&J3_L_position_set,J3_ramp,J3_send_set);
	arm_ramp_calc(&J4_L_position_set,J4_ramp,J4_send_set);
	arm_ramp_calc(&J5_L_position_set,J5_ramp,J5_send_set);
	arm_ramp_calc(&J6_L_position_set,J6_ramp,J6_send_set);
	arm_ramp_calc(&grip_L_position_set,Grip_ramp,Grip_send_set);

	/*赋值*/
	ArmData.J1.send_set = J1_L_position_set.out;
	ArmData.J2.send_set = J2_L_position_set.out;
	ArmData.J3.send_set = J3_L_position_set.out;
	ArmData.J4.send_set = J4_L_position_set.out;
	ArmData.J5.send_set = J5_L_position_set.out;
	ArmData.J6.send_set = J6_L_position_set.out;
	ArmData.Grip.send_set = grip_L_position_set.out;
}

void TAKE_WATER_1(void)
{
	//static int take_water = 0;
	ArmData.J1.Kp = J1_PID_KP;ArmData.J1.Kd = J1_PID_KD;
	ArmData.J2.Kp = J2_PID_KP;ArmData.J2.Kd = J2_PID_KD;
	ArmData.J3.Kp = J3_PID_KP;ArmData.J3.Kd = J3_PID_KD;
	ArmData.J4.Kp = J4_PID_KP;ArmData.J4.Kd = J4_PID_KD;
	ArmData.J5.Kp = J5_PID_KP;ArmData.J5.Kd = J5_PID_KD;
	ArmData.J6.Kp = J6_PID_KP;ArmData.J6.Kd = J6_PID_KD;
	ArmData.Grip.Kp = J6_PID_KP;ArmData.Grip.Kd = J6_PID_KD;

	J1_send_set = J1_TAKE_position;
	J2_send_set = J2_TAKE_position;
	J3_send_set = J3_TAKE_position;
	J4_send_set = J4_TAKE_position;
	J5_send_set = J5_TAKE_position;
	J6_send_set = J6_TAKE_position;
	Grip_send_set = 0;

//	Grip_send_set = grip_zero_position;

	J1_ramp= 0.0090,J2_ramp= 0.003,J3_ramp= 0.0045,J4_ramp= 0.0042,J5_ramp= 0.003,J6_ramp= 0.003,Grip_ramp= 0.030;  //������

	/*��׶�һ���������ص�λ���ٱպ���צȡˮ*/

	/*��λ*/
	limitPosition(&J1_send_set,J1_Max_position,J1_Min_position);
	limitPosition(&J2_send_set,J2_Max_position,J2_Min_position);
	limitPosition(&J3_send_set,J3_Max_position,J3_Min_position);
	limitPosition(&J4_send_set,J4_Max_position,J4_Min_position);
	limitPosition(&J5_send_set,J5_Max_position,J5_Min_position);
	limitPosition(&J6_send_set,J6_Max_position,J6_Min_position);
	limitPosition(&Grip_send_set,Grip_Max_position,Grip_Min_position);

	/*б�º���*/
	arm_ramp_calc(&J1_L_position_set,J1_ramp,J1_send_set);
	arm_ramp_calc(&J2_L_position_set,J2_ramp,J2_send_set);
	arm_ramp_calc(&J3_L_position_set,J3_ramp,J3_send_set);
	arm_ramp_calc(&J4_L_position_set,J4_ramp,J4_send_set);
	arm_ramp_calc(&J5_L_position_set,J5_ramp,J5_send_set);
	arm_ramp_calc(&J6_L_position_set,J6_ramp,J6_send_set);
	arm_ramp_calc(&grip_L_position_set,Grip_ramp,Grip_send_set);

	/*���λ�ø�ֵ*/
	ArmData.J1.send_set = J1_L_position_set.out;
	ArmData.J2.send_set = J2_L_position_set.out;
	ArmData.J3.send_set = J3_L_position_set.out;
	ArmData.J4.send_set = J4_L_position_set.out;
	ArmData.J5.send_set = J5_L_position_set.out;
	ArmData.J6.send_set = J6_L_position_set.out;
	ArmData.Grip.send_set = grip_L_position_set.out;
}

//void Arm_Key_Control_mode(void)
//{
//	static uint8_t mode_lock = 0;
//	static int L_save_flag = 0,L_get_flag = 0,L_save_flag_move = 1,L_get_flag_move = 1;
//	static int R_save_flag = 0,R_get_flag = 0,R_save_flag_move = 1,R_get_flag_move = 1;
//	
//	static fp32 J4_half = 0.0f;//J4ת��
//	static int end_direction = -1; //
//	
//	/*������PD��ֵ*/
//	ArmData.J1.Kp = J1_PID_KP;ArmData.J1.Kd = J1_PID_KD;
//	ArmData.J2.Kp = J2_PID_KP;ArmData.J2.Kd = J2_PID_KD;
//	ArmData.J3.Kp = J3_PID_KP;ArmData.J3.Kd = J3_PID_KD;
//	ArmData.J4.Kp = J4_PID_KP;ArmData.J4.Kd = J4_PID_KD;
//	ArmData.J5.Kp = J5_PID_KP;ArmData.J5.Kd = J5_PID_KD;
//	ArmData.J6.Kp = J6_PID_KP;ArmData.J6.Kd = J6_PID_KD;
//	ArmData.Grip.Kp = J6_PID_KP;ArmData.Grip.Kd = J6_PID_KD;
//	
//	/*(��)һ����������Ԫ*/
//	if (((RCData.key.v & KEY_PRESSED_OFFSET_Q) && (RCData.mouse.press_l) && mode_lock == 0) || L_save_flag == 1){/*һ����������Ԫ*/
//		if (mode_lock == 0) mode_lock = 1;
//		L_save_flag = 1;
//		switch (L_save_flag_move)
//        {	
//			case 1:/*�׶�һ*/
//			{	
//				J1_send_set = L_J1_save_position_1;
//				J2_send_set = L_J2_save_position_1;
//				J3_send_set = L_J3_save_position_1;
//				J4_send_set = L_J4_save_position_1;
//				J5_send_set = L_J5_save_position_1;
//				J6_send_set = L_J6_save_position_1;
//				Grip_send_set = Grip_clamp_position;
//				
//				J1_ramp= 0.0090,J2_ramp= 0.003,J3_ramp= 0.0045,J4_ramp= 0.0042,J5_ramp= 0.003,J6_ramp= 0.003,Grip_ramp= 0.030;  //������
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					L_save_flag_move =2;
//				}
//			}
//				break;
//			case 2:/*�׶ζ�*///��צ�����洢����(��Ҫϸ����)
//			{
//				J1_send_set = L_J1_save_position_2;
//				J2_send_set = L_J2_save_position_2;
//				J3_send_set = L_J3_save_position_2;
//				J4_send_set = L_J4_save_position_2;
//				J5_send_set = L_J5_save_position_2;
//				J6_send_set = L_J6_save_position_2;
//				Grip_send_set = L_Grip_save_position_2;
//				
//				
//				J1_ramp= 0.00450,J2_ramp= 0.003,J3_ramp= 0.0015,J4_ramp= 0.0015,J5_ramp= 0.002,J6_ramp= 0.0015,Grip_ramp= 0.00350;  //������
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					L_save_flag_move =3;
//					}
//			}
//				break;
//			case 3:/*�׶���*/
//			{
//				J1_send_set = L_J1_save_position_1;
//				J2_send_set = L_J2_save_position_1;
//				J3_send_set = L_J3_save_position_1;
//				J4_send_set = L_J4_save_position_1;
//				J5_send_set = L_J5_save_position_1;
//				J6_send_set = L_J6_save_position_1;
//				Grip_send_set = L_Grip_save_position_2;
//				
//				J1_ramp= 0.0090,J2_ramp= 0.003,J3_ramp= 0.0045,J4_ramp= 0.0015,J5_ramp= 0.003,J6_ramp= 0.0015,Grip_ramp= 0.030;  //������
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					L_save_flag_move = 4;
//				}
//			}
//				break;
//			case 4:/*�׶���*/
//			{
//				J1_send_set = J1_zero_position;
//				J2_send_set = J2_zero_position;
//				J3_send_set = J3_zero_position;
//				J4_send_set = J4_zero_position;
//				J5_send_set = J5_zero_position;
//				J6_send_set = J6_zero_position;
//				Grip_send_set = grip_zero_position;
//				
//				J1_ramp= 0.01,J2_ramp= 0.0028,J3_ramp= 0.002,J4_ramp= 0.003,J5_ramp= 0.0035,J6_ramp= 0.004,Grip_ramp= 0.0030;  //������
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					L_save_flag_move = 5;
//				}
//			}
//				break;
//			default:
//            break;
//        }
//		if(L_save_flag_move == 5){
//			L_save_flag_move = 1;
//			L_save_flag = 0;
//			mode_lock = 0;
//		}
//	}
//	/*(��)һ��ȡ������Ԫ*/
//	else if(((RCData.key.v & KEY_PRESSED_OFFSET_Q) && (RCData.mouse.press_r) && mode_lock == 0) || L_get_flag == 1){
//		if (mode_lock == 0) mode_lock = 1;
//	    L_get_flag = 1;
//		switch (L_get_flag_move)
//        {	
//			case 1:/*�׶�һ*/
//			{
//				J1_send_set = L_J1_get_position_1;
//				J2_send_set = L_J2_get_position_1;
//				J3_send_set = L_J3_get_position_1;
//				J4_send_set = L_J4_get_position_1;
//				J5_send_set = L_J5_get_position_1;
//				J6_send_set = L_J6_get_position_1;
//				Grip_send_set = L_Grip_get_position_1;
//			
//				J1_ramp= 0.0090,J2_ramp= 0.002,J3_ramp= 0.003,J4_ramp= 0.005,J5_ramp= 0.003,J6_ramp= 0.002,Grip_ramp= 0.020;  //������
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					L_get_flag_move =2;
//				}
//			}
//				break;
//			case 2:/*�׶ζ�*/
//			{
//				J1_send_set = L_J1_get_position_2;
//				J2_send_set = L_J2_get_position_2;
//				J3_send_set = L_J3_get_position_2;
//				J4_send_set = L_J4_get_position_2;
//				J5_send_set = L_J5_get_position_2;
//				J6_send_set = L_J6_get_position_2;
//				Grip_send_set = L_Grip_get_position_1;
//				
//				J1_ramp= 0.0030,J2_ramp= 0.0015,J3_ramp= 0.001,J4_ramp= 0.001,J5_ramp= 0.0015,J6_ramp= 0.0023,Grip_ramp= 0.020;  //������
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					L_get_flag_move =3;
//				}
//			}
//				break;
//			case 3:/*�׶���*///��צ��סȡ����Ԫ
//			{
//				Grip_send_set = Grip_clamp_position;
//				Grip_ramp= 0.0150;  //������
//				if(grip_L_position_set.out == Grip_send_set){
//					L_get_flag_move = 4;
//				}
//			}
//				break;
//			case 4:/*�׶���*/
//			{
//				J1_send_set = L_J1_get_position_1;
//				J2_send_set = L_J2_get_position_1;
//				J3_send_set = L_J3_get_position_1;
//				J4_send_set = L_J4_get_position_1;
//				J5_send_set = L_J5_get_position_1;
//				J6_send_set = L_J6_get_position_1;
//				Grip_send_set = Grip_clamp_position;
//				J1_ramp= 0.0030,J2_ramp= 0.001,J3_ramp= 0.001,J4_ramp= 0.001,J5_ramp= 0.0015,J6_ramp= 0.0015,Grip_ramp= 0.0020;  //������
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					L_get_flag_move = 5;
//				}
//			}
//				break;
//			case 5:/*�׶���*///̧��J3(��ֹ������Ԫ��Χ����)
//			{
//				J3_send_set = 0.737690163f;
//				J1_ramp= 0.0050,J2_ramp= 0.002,J3_ramp= 0.003,J4_ramp= 0.002,J5_ramp= 0.002,J6_ramp= 0.003,Grip_ramp= 0.020;  //������
//				if(J3_L_position_set.out == J3_send_set){
//					L_get_flag_move = 6;
//				}
//				break;
//			}
//			case 6:/*�׶���*/
//			{
//				J1_send_set = J1_zero_position;
//				J2_send_set = J2_zero_position;
//				J3_send_set = J3_zero_position;
//				J4_send_set = J4_zero_position;
//				J5_send_set = J5_zero_position;
//				J6_send_set = J6_zero_position;
//				Grip_send_set = Grip_clamp_position;
//				J1_ramp= 0.01,J2_ramp= 0.0028,J3_ramp= 0.003,J4_ramp= 0.004,J5_ramp= 0.004,J6_ramp= 0.0036,Grip_ramp= 0.020;  //������
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					L_get_flag_move = 7;
//				}
//			}
//				break;
//				break;
//			default:
//            break;
//        }
//		if(L_get_flag_move == 7){
//			L_get_flag_move = 1;
//			L_get_flag = 0;
//			mode_lock = 0;
//		}
//	}
//	
//	/*(��)һ����������Ԫ*/
//	else if (((RCData.key.v & KEY_PRESSED_OFFSET_E) && (RCData.mouse.press_l) && mode_lock == 0) || R_save_flag == 1){/*һ����������Ԫ*/
//	if (mode_lock == 0) mode_lock = 1;
//		R_save_flag = 1;
//		switch (R_save_flag_move)
//        {	
//			case 1:/*�׶�һ*/
//			{	
//				J1_send_set = R_J1_save_position_1;
//				J2_send_set = R_J2_save_position_1;
//				J3_send_set = R_J3_save_position_1;
//				J4_send_set = R_J4_save_position_1;
//				J5_send_set = R_J5_save_position_1;
//				J6_send_set = R_J6_save_position_1;
//				Grip_send_set = Grip_clamp_position;
//				
//				J1_ramp= 0.0090,J2_ramp= 0.003,J3_ramp= 0.0045,J4_ramp= 0.0042,J5_ramp= 0.0035,J6_ramp= 0.002,Grip_ramp= 0.030;  //������
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					R_save_flag_move = 2;
//				}
//			}
//				break;
//			case 2:/*�׶ζ�*///��צ�����洢����(��Ҫϸ����)
//			{
//				J1_send_set = R_J1_save_position_2;
//				J2_send_set = R_J2_save_position_2;
//				J3_send_set = R_J3_save_position_2;
//				J4_send_set = R_J4_save_position_2;
//				J5_send_set = R_J5_save_position_2;
//				J6_send_set = R_J6_save_position_2;
//				Grip_send_set = R_Grip_save_position_2;
//				
//				
//				J1_ramp= 0.00450,J2_ramp= 0.003,J3_ramp= 0.0015,J4_ramp= 0.0015,J5_ramp= 0.002,J6_ramp= 0.0015,Grip_ramp= 0.00450;  //������
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					R_save_flag_move =3;
//					}
//			}
//				break;
//			case 3:/*�׶���*/
//			{
//				J1_send_set = R_J1_save_position_1;
//				J2_send_set = R_J2_save_position_1;
//				J3_send_set = R_J3_save_position_1;
//				J4_send_set = R_J4_save_position_1;
//				J5_send_set = R_J5_save_position_1;
//				J6_send_set = R_J6_save_position_1;
//				Grip_send_set = R_Grip_save_position_2;
//				
//				J1_ramp= 0.0090,J2_ramp= 0.003,J3_ramp= 0.0045,J4_ramp= 0.0015,J5_ramp= 0.003,J6_ramp= 0.0015,Grip_ramp= 0.030;  //������
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					R_save_flag_move = 4;
//				}
//			}
//				break;
//			case 4:/*�׶���*/
//			{
//				J1_send_set = J1_zero_position;
//				J2_send_set = J2_zero_position;
//				J3_send_set = J3_zero_position;
//				J4_send_set = J4_zero_position;
//				J5_send_set = J5_zero_position;
//				J6_send_set = J6_zero_position;
//				Grip_send_set = grip_zero_position;
//				
//				J1_ramp= 0.01,J2_ramp= 0.0028,J3_ramp= 0.002,J4_ramp= 0.003,J5_ramp= 0.0035,J6_ramp= 0.004,Grip_ramp= 0.0030;  //������
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					R_save_flag_move = 5;
//				}
//			}
//				break;
//			default:
//            break;
//        }
//		if(R_save_flag_move == 5){
//			R_save_flag_move = 1;
//			R_save_flag = 0;
//			mode_lock = 0;
//		}
//	}
//	/*(��)һ��ȡ������Ԫ*/
//	else if(((RCData.key.v & KEY_PRESSED_OFFSET_E) && (RCData.mouse.press_r) && mode_lock == 0) || R_get_flag == 1){
//	if (mode_lock == 0) mode_lock = 1;
//	    R_get_flag = 1;
//		switch (R_get_flag_move)
//        {	
//			case 1:/*�׶�һ*/
//			{
//				J1_send_set = R_J1_get_position_1;
//				J2_send_set = R_J2_get_position_1;
//				J3_send_set = R_J3_get_position_1;
//				J4_send_set = R_J4_get_position_1;
//				J5_send_set = R_J5_get_position_1;
//				J6_send_set = R_J6_get_position_1;
//				Grip_send_set =R_Grip_get_position_1;
//			
//				J1_ramp= 0.0090,J2_ramp= 0.002,J3_ramp= 0.003,J4_ramp= 0.005,J5_ramp= 0.003,J6_ramp= 0.002,Grip_ramp= 0.020;  //������
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					R_get_flag_move =2;
//				}
//			}
//				break;
//			case 2:/*�׶ζ�*/
//			{
//				J1_send_set = R_J1_get_position_2;
//				J2_send_set = R_J2_get_position_2;
//				J3_send_set = R_J3_get_position_2;
//				J4_send_set = R_J4_get_position_2;
//				J5_send_set = R_J5_get_position_2;
//				J6_send_set = R_J6_get_position_2;
//				Grip_send_set = R_Grip_get_position_1;
//				
//				J1_ramp= 0.0030,J2_ramp= 0.0015,J3_ramp= 0.001,J4_ramp= 0.001,J5_ramp= 0.0015,J6_ramp= 0.0023,Grip_ramp= 0.020;  //������
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					R_get_flag_move = 3;
//				}
//			}
//				break;
//			case 3:/*�׶���*///��צ��סȡ����Ԫ
//			{
//				Grip_send_set = Grip_clamp_position;
//				Grip_ramp= 0.0150;  //������
//				if(grip_L_position_set.out == Grip_send_set){
//					R_get_flag_move = 4;
//				}
//			}
//				break;
//			case 4:/*�׶���*/
//			{
//				J1_send_set = R_J1_get_position_1;
//				J2_send_set = R_J2_get_position_1;
//				J3_send_set = R_J3_get_position_1;
//				J4_send_set = R_J4_get_position_1;
//				J5_send_set = R_J5_get_position_1;
//				J6_send_set = R_J6_get_position_1;
//				Grip_send_set = Grip_clamp_position;
//				
//				J1_ramp= 0.0030,J2_ramp= 0.001,J3_ramp= 0.001,J4_ramp= 0.001,J5_ramp= 0.0015,J6_ramp= 0.0015,Grip_ramp= 0.0020;  //������
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					R_get_flag_move = 5;
//				}
//			}
//				break;
//			case 5:/*�׶���*///̧��J3(��ֹ������Ԫ��Χ����)
//			{
//				J3_send_set = 0.725537453f;
//				J1_ramp= 0.0050,J2_ramp= 0.002,J3_ramp= 0.003,J4_ramp= 0.002,J5_ramp= 0.002,J6_ramp= 0.003,Grip_ramp= 0.020;  //������
//				if(J3_L_position_set.out == J3_send_set){
//					R_get_flag_move = 6;
//				}
//				break;
//			}
//			case 6:/*�׶���*/
//			{
//				J1_send_set = J1_zero_position;
//				J2_send_set = J2_zero_position;
//				J3_send_set = J3_zero_position;
//				J4_send_set = J4_zero_position;
//				J5_send_set = J5_zero_position;
//				J6_send_set = J6_zero_position;
//				Grip_send_set = Grip_clamp_position;
//				J1_ramp= 0.01,J2_ramp= 0.0028,J3_ramp= 0.002,J4_ramp= 0.004,J5_ramp= 0.004,J6_ramp= 0.0036,Grip_ramp= 0.020;  //������
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					R_get_flag_move = 7;
//				}
//			}
//				break;
//				break;
//			default:
//            break;
//        }
//		if(R_get_flag_move == 7){
//			R_get_flag_move = 1;
//			R_get_flag = 0;
//			mode_lock = 0;
//		}
//	}
//    else{
//		J1_ramp= 0.030,J2_ramp= 0.01,J3_ramp= 0.01,J4_ramp= 0.02,J5_ramp= 0.01,J6_ramp= 0.01,Grip_ramp= 0.020;
//		
//		/*��צ����*/
//		if(CC_data.CC_Key_grip == 1)
//		Grip_send_set = grip_zero_position + Grip_clamp_position;
//		if(CC_data.CC_Key_grip == 0)
//		Grip_send_set = grip_zero_position;
//		
//		/*J4һ����ת*/
//		if(CC_data.CC_Key_J4 ==1){
//		J4_half = J4_half_turn;  //��תһȦ
//			end_direction = 1;
//			}
//		if(CC_data.CC_Key_J4 ==0){
//		J4_half = 0;  //��תһȦ
//			end_direction = -1;
//		}
//		
//		/*��λ*/
//		limitPosition(&CC_data.DM_CC_2_receive,J2_Max_position - ArmData.J2.Initial_Total_position,J2_Min_position - ArmData.J2.Initial_Total_position);
//		limitPosition(&CC_data.DM_CC_3_receive,J3_Max_position - ArmData.J3.Initial_Total_position,J3_Min_position - ArmData.J3.Initial_Total_position);
//		limitPosition(&CC_data.DM_CC_4_receive,J4_Max_position - ArmData.J4.Initial_Total_position,J4_Min_position - ArmData.J4.Initial_Total_position);
//		limitPosition(&limit_data5,J5_Max_position - ArmData.J5.Initial_Total_position,J5_Min_position - ArmData.J5.Initial_Total_position);
//		limitPosition(&limit_data6,J6_Max_position - ArmData.J6.Initial_Total_position,J6_Min_position - ArmData.J6.Initial_Total_position);
//		/*���͸�MIT����ģʽ�µĺ�����λ��*/
//		J1_set_initial_data = J1_zero_position + CC_data.DM_CC_1_receive;  //J1_zero_position + ArmData.J1.RC_Sum_Value + Custom_RC[0].DM_CC_receive;
//		J2_set_initial_data = J2_zero_position + CC_data.DM_CC_2_receive;
//		J3_set_initial_data = J3_zero_position + CC_data.DM_CC_3_receive;
//		J4_set_initial_data = J4_zero_position + CC_data.DM_CC_4_receive * 4.0;
//		J5_set_initial_data = J5_zero_position + CC_data.DJ_CC_1_receive * 0.000019103116231;//2.48340511/130000
//		J6_set_initial_data = J6_zero_position - CC_data.DJ_CC_2_receive * 0.0008;
//	    // ������͹��ȱ���	
//		J1_send_set = J1_zero_position + CC_data.DM_CC_1_receive;    //J1_zero_position + ArmData.J1.RC_Sum_Value + Custom_RC[0].DM_CC_receive + ArmData.J1.Key_Control_Value;//��n���ؽڵ����ʼλ��+ң������������+�Զ����������������
//		J2_send_set = J2_zero_position + CC_data.DM_CC_2_receive;
//		J3_send_set = J3_zero_position + CC_data.DM_CC_3_receive;
//		J4_send_set = J4_zero_position + CC_data.DM_CC_4_receive * 4.0 + J4_half;

//		limit_data5 = CC_data.DJ_CC_1_receive * 0.000019103116231;
//		limit_data6 = CC_data.DJ_CC_2_receive * 0.0008;
//		
//		J5_send_set = end_direction * (J5_zero_position + limit_data5);//��n���ؽڵ����ʼλ��+ң������������+�Զ����������������
//		J6_send_set = J6_zero_position - limit_data6;
//		
//	}
//	
//	/*��λ*/
//	if(joint_L_motor[1].position < Special_J2_limit && J2_L_position_set.out < Special_J2_limit)  
//	{
//		limitPosition(&J1_send_set,J1_Special_Max_position,J1_Special_Min_position);
//	}
//	else
//	{
//		limitPosition(&J1_send_set,J1_Max_position,J1_Min_position);
//	}
//	limitPosition(&ArmData.J2.send_set,J2_Max_position,J2_Min_position);
//	limitPosition(&ArmData.J3.send_set,J3_Max_position,J3_Min_position);
//	limitPosition(&ArmData.J4.send_set,J4_Max_position + J4_half,J4_Min_position - J4_half);
//	limitPosition(&ArmData.J5.send_set,J5_Max_position,J5_Min_position);
//	limitPosition(&ArmData.J6.send_set,J6_Max_position,J6_Min_position);
//	limitPosition(&ArmData.Grip.send_set,Grip_Max_position,Grip_Min_position);
//	
//	//б�º���
//	arm_ramp_calc(&J1_L_position_set,J1_ramp,J1_send_set);
//	arm_ramp_calc(&J2_L_position_set,J2_ramp,J2_send_set);
//	arm_ramp_calc(&J3_L_position_set,J3_ramp,J3_send_set);
//	arm_ramp_calc(&J4_L_position_set,J4_ramp,J4_send_set);
//	arm_ramp_calc(&J5_L_position_set,J5_ramp,J5_send_set);
//	arm_ramp_calc(&J6_L_position_set,J6_ramp,J6_send_set);	
//	arm_ramp_calc(&grip_L_position_set,Grip_ramp,Grip_send_set);
//	/*���λ�ø�ֵ*/	
//	ArmData.J1.send_set = J1_L_position_set.out;
//	ArmData.J2.send_set = J2_L_position_set.out;
//	ArmData.J3.send_set = J3_L_position_set.out;
//	ArmData.J4.send_set = J4_L_position_set.out;
//	ArmData.J5.send_set = J5_L_position_set.out;
//	ArmData.J6.send_set = J6_L_position_set.out;
//	ArmData.Grip.send_set = grip_L_position_set.out;
//	
//	/*��������ǰλ�ø�������ģʽ*/
//	ArmData.J1.Now_set_position = ArmData.J1.send_set;
//	ArmData.J2.Now_set_position = ArmData.J2.send_set;
//	ArmData.J3.Now_set_position = ArmData.J3.send_set;
//	ArmData.J4.Now_set_position = ArmData.J4.send_set;
//	ArmData.J5.Now_set_position = ArmData.J5.send_set;
//	ArmData.J6.Now_set_position = ArmData.J6.send_set;
//	ArmData.Grip.Now_set_position = ArmData.Grip.send_set;
//}

/*����ģʽ*/
void Arm_Powerless(void)
{
	ArmData.J1.Kp = ArmData.J2.Kp = ArmData.J3.Kp = ArmData.J4.Kp = ArmData.J5.Kp = ArmData.J6.Kp = ArmData.Grip.Kp = ArmData.J1.Kd = ArmData.J2.Kd = ArmData.J3.Kd = ArmData.J4.Kd = ArmData.J5.Kd = ArmData.J6.Kd = ArmData.Grip.Kd = Joint_Atony_KP_KD;
	/*���͵�ǰλ��*/
	ArmData.J1.send_set=ArmData.J1.position;
	ArmData.J2.send_set=ArmData.J2.position;
	ArmData.J3.send_set=ArmData.J3.position;	
	ArmData.J4.send_set=ArmData.J4.position;
	ArmData.J5.send_set=ArmData.J5.position;
	ArmData.J6.send_set=ArmData.J6.position;
	ArmData.Grip.send_set=ArmData.Grip.position;
	/*���赱ǰλ��*/
	ArmData.J1.Initial_Total_position=ArmData.J1.position;
	ArmData.J2.Initial_Total_position=ArmData.J2.position;
	ArmData.J3.Initial_Total_position=ArmData.J3.position;
	ArmData.J4.Initial_Total_position=ArmData.J4.position;
	ArmData.J5.Initial_Total_position=ArmData.J5.position;
	ArmData.J6.Initial_Total_position=ArmData.J6.position;
	ArmData.Grip.Initial_Total_position=ArmData.Grip.position;
	
	/*����б�º����ṹ�嵱ǰλ��*/
	J1_L_position_set.out = ArmData.J1.position;
	J2_L_position_set.out = ArmData.J2.position;
	J3_L_position_set.out = ArmData.J3.position;
	J4_L_position_set.out = ArmData.J4.position;
	J5_L_position_set.out = ArmData.J5.position;
	J6_L_position_set.out = ArmData.J6.position;
	grip_L_position_set.out = ArmData.Grip.position;
	
	/*ң���ۼ�����*/
	ArmData.J1.RC_Sum_Value = 0;
	ArmData.J2.RC_Sum_Value = 0;
	ArmData.J3.RC_Sum_Value = 0;
	ArmData.J4.RC_Sum_Value = 0;
	ArmData.J5.RC_Sum_Value = 0;
	ArmData.J6.RC_Sum_Value = 0;
	ArmData.Grip.RC_Sum_Value = 0;
	
}

/*************************************ң��������*******************************************/

void Arm_L_Movement_J1_J4(void)
{
	int16_t channel_0,channel_1,channel_2,channel_3;
	/*��ȡң����ͨ�����룬Ӧ����������*/
	rc_deadband_limit(RCData.rc.ch[0],channel_0,10);
	rc_deadband_limit(RCData.rc.ch[1],channel_1,10);
	rc_deadband_limit(RCData.rc.ch[2],channel_2,10);
	rc_deadband_limit(RCData.rc.ch[3],channel_3,10);
	/*������PD��ֵ*/
	ArmData.J1.Kp = J1_PID_KP;ArmData.J1.Kd = J1_PID_KD;
	ArmData.J2.Kp = J2_PID_KP;ArmData.J2.Kd = J2_PID_KD;
	ArmData.J3.Kp = J3_PID_KP;ArmData.J3.Kd = J3_PID_KD;
	ArmData.J4.Kp = J4_PID_KP;ArmData.J4.Kd = J4_PID_KD;
	ArmData.J5.Kp = J5_PID_KP;ArmData.J5.Kd = J5_PID_KD;
	ArmData.J6.Kp = J6_PID_KP;ArmData.J6.Kd = J6_PID_KD;
	ArmData.Grip.Kp = J6_PID_KP;ArmData.Grip.Kd = J6_PID_KD;
	/*ң�������ۼ�ֵ*/                 
	ArmData.J1.RC_Sum_Value += channel_0*0.00001;
	ArmData.J2.RC_Sum_Value += channel_1*0.00001;
	ArmData.J3.RC_Sum_Value += channel_0*0.00001;
	ArmData.J4.RC_Sum_Value += channel_1*0.00002;
	ArmData.J5.RC_Sum_Value += channel_2*0.00002;
	ArmData.J6.RC_Sum_Value += channel_2*0.00002;
	ArmData.Grip.RC_Sum_Value += channel_3*0.0003;

	if(joint_L_motor[1].position < 0.496871948f)
	{
		limitPosition(&ArmData.J1.RC_Sum_Value,J1_Max_position - ArmData.J1.Initial_Total_position,-0.282964571 - ArmData.J1.Initial_Total_position);
	}	
	else
	{
		limitPosition(&ArmData.J1.RC_Sum_Value,J1_Max_position - ArmData.J1.Initial_Total_position,J1_Min_position - ArmData.J1.Initial_Total_position);
	}  
	
	/*���͸�MIT����ģʽ�µĺ�����λ��*/
	J1_set_initial_data = J1_zero_position + ArmData.J1.RC_Sum_Value;  //J1_zero_position + ArmData.J1.RC_Sum_Value + Custom_RC[0].DM_CC_receive;
	J2_set_initial_data = J2_zero_position + ArmData.J2.RC_Sum_Value;
	J3_set_initial_data = J3_zero_position + ArmData.J3.RC_Sum_Value;
	J4_set_initial_data = J4_zero_position + ArmData.J4.RC_Sum_Value;
	J5_set_initial_data = J5_zero_position + ArmData.J5.RC_Sum_Value + Custom_RC[4].DM_CC_receive;
	J6_set_initial_data = J6_zero_position + ArmData.J6.RC_Sum_Value + Custom_RC[5].DM_CC_receive;
	Grip_set_initial_data = grip_zero_position + ArmData.Grip.RC_Sum_Value;

	J1_send_set = J1_zero_position + ArmData.J1.RC_Sum_Value;    //J1_zero_position + ArmData.J1.RC_Sum_Value + Custom_RC[0].DM_CC_receive + ArmData.J1.Key_Control_Value;//��n���ؽڵ����ʼλ��+ң������������+�Զ����������������
	J2_send_set = J2_zero_position + ArmData.J2.RC_Sum_Value;
	J3_send_set = J3_zero_position + ArmData.J3.RC_Sum_Value;
	J4_send_set = J4_zero_position + ArmData.J4.RC_Sum_Value;
	J5_send_set = J5_zero_position + ArmData.J5.RC_Sum_Value + Custom_RC[4].DM_CC_receive + ArmData.J5.Key_Control_Value;
	J6_send_set = J6_zero_position + ArmData.J6.RC_Sum_Value + Custom_RC[5].DM_CC_receive + ArmData.J6.Key_Control_Value;
	Grip_send_set = grip_zero_position + ArmData.Grip.RC_Sum_Value + ArmData.Grip.Key_Control_Value;
	
	if(joint_L_motor[1].position < 0.496871948f)
	{
		limitPosition(&J1_send_set,J1_Max_position,-0.282964571);
	}	
	else
	{
		limitPosition(&J1_send_set,J1_Max_position,J1_Min_position);
	}  
	/*���λ�ø�ֵ*/
	ArmData.J1.send_set = J1_send_set;
	ArmData.J2.send_set = J2_send_set;
	ArmData.J3.send_set = J3_send_set;
	ArmData.J4.send_set = J4_send_set;
	ArmData.J5.send_set = J5_send_set;
	ArmData.J6.send_set = J6_send_set;
	ArmData.Grip.send_set = Grip_send_set;

	/* 同步斜坡状态量到当前实际位置，避免切模式时从零点起跳 */
	J1_L_position_set.out = ArmData.J1.position;
	J2_L_position_set.out = ArmData.J2.position;
	J3_L_position_set.out = ArmData.J3.position;
	J4_L_position_set.out = ArmData.J4.position;
	J5_L_position_set.out = ArmData.J5.position;
	J6_L_position_set.out = ArmData.J6.position;
	grip_L_position_set.out = ArmData.Grip.position;

//	Arm_Key_Control_mode();  //�������
	
	/*��λ*/  
	limitPosition(&ArmData.J1.RC_Sum_Value,J1_Max_position - ArmData.J1.Initial_Total_position,J1_Min_position - ArmData.J1.Initial_Total_position);
	limitPosition(&ArmData.J2.RC_Sum_Value,J2_Max_position - ArmData.J2.Initial_Total_position - ArmData.J2.Key_Control_Value,J2_Min_position - ArmData.J2.Initial_Total_position - ArmData.J2.Key_Control_Value);
	limitPosition(&ArmData.J3.RC_Sum_Value,J3_Max_position - ArmData.J3.Initial_Total_position,J3_Min_position - ArmData.J3.Initial_Total_position);
	limitPosition(&ArmData.J4.RC_Sum_Value,J4_Max_position - ArmData.J4.Initial_Total_position - ArmData.J4.Key_Control_Value,J4_Min_position - ArmData.J4.Initial_Total_position - ArmData.J4.Key_Control_Value);
	limitPosition(&ArmData.J5.RC_Sum_Value,J5_Max_position - ArmData.J5.Initial_Total_position,J5_Min_position - ArmData.J5.Initial_Total_position);
	limitPosition(&ArmData.J6.RC_Sum_Value,J6_Max_position - ArmData.J6.Initial_Total_position - ArmData.J6.Key_Control_Value,J6_Min_position - ArmData.J6.Initial_Total_position - ArmData.J6.Key_Control_Value);
	limitPosition(&ArmData.Grip.RC_Sum_Value,Grip_Max_position - grip_zero_position - ArmData.Grip.Key_Control_Value,Grip_Min_position - grip_zero_position - ArmData.Grip.Key_Control_Value);

//				J1_send_set = R_J1_get_position_1;
//				J2_send_set = R_J2_get_position_1;
//				J3_send_set = R_J3_get_position_1;
//				J4_send_set = R_J4_get_position_1;
//				J5_send_set = R_J5_get_position_1;
//				J6_send_set = R_J6_get_position_1;
//				Grip_send_set =R_Grip_get_position_1;	
	if(joint_L_motor[1].position < 0.496871948f)
	{
		limitPosition(&ArmData.J1.send_set,J1_Max_position,-0.17605114);
	}	
	else
	{
		limitPosition(&ArmData.J1.send_set,J1_Max_position,J1_Min_position);
	}  
	limitPosition(&ArmData.J1.send_set,J1_Max_position,J1_Min_position);
	limitPosition(&ArmData.J2.send_set,J2_Max_position,J2_Min_position);
	limitPosition(&ArmData.J3.send_set,J3_Max_position,J3_Min_position);
	limitPosition(&ArmData.J4.send_set,J4_Max_position,J4_Min_position);
	limitPosition(&ArmData.J5.send_set,J5_Max_position,J5_Min_position);
	limitPosition(&ArmData.J6.send_set,J6_Max_position,J6_Min_position);
	limitPosition(&ArmData.Grip.send_set,Grip_Max_position,Grip_Min_position);
	
	/*��������ǰλ�ø�������ģʽ*/
	ArmData.J1.Now_set_position = ArmData.J1.send_set;
	ArmData.J2.Now_set_position = ArmData.J2.send_set;
	ArmData.J3.Now_set_position = ArmData.J3.send_set;
	ArmData.J4.Now_set_position = ArmData.J4.send_set;
	ArmData.J5.Now_set_position = ArmData.J5.send_set;
	ArmData.J6.Now_set_position = ArmData.J6.send_set;
	ArmData.Grip.Now_set_position = ArmData.Grip.send_set;
	
}

void Arm_return_initial_position(void)
{
	ArmData.J1.send_set=joint_L_motor[0].position;
	ArmData.J2.send_set=joint_L_motor[1].position;
	ArmData.J3.send_set=joint_L_motor[2].position;
	ArmData.J4.send_set=joint_L_motor[3].position;
	ArmData.J5.send_set=joint_L_motor[4].position;
	ArmData.J6.send_set=joint_L_motor[5].position;
	ArmData.Grip.send_set=joint_L_motor[6].position;
	osDelay(500);
	ArmData.J1.position=joint_L_motor[0].position;
	ArmData.J2.position=joint_L_motor[1].position;
	ArmData.J3.position=joint_L_motor[2].position;
	ArmData.J4.position=joint_L_motor[3].position;
	ArmData.J5.position=joint_L_motor[4].position;
	ArmData.J6.position=joint_L_motor[5].position;
	ArmData.Grip.position=joint_L_motor[6].position;

	ArmData.J1.send_set=joint_L_motor[0].position;
	ArmData.J2.send_set=joint_L_motor[1].position;
	ArmData.J3.send_set=joint_L_motor[2].position;
	ArmData.J4.send_set=joint_L_motor[3].position;
	ArmData.J5.send_set=joint_L_motor[4].position;
	ArmData.J6.send_set=joint_L_motor[5].position;
	ArmData.Grip.send_set=joint_L_motor[6].position;

	ArmData.J1.Initial_Total_position=ArmData.J1.position;
	ArmData.J2.Initial_Total_position=ArmData.J2.position;	
	ArmData.J3.Initial_Total_position=ArmData.J3.position;
	ArmData.J4.Initial_Total_position=ArmData.J4.position;	
	ArmData.J5.Initial_Total_position=ArmData.J5.position;
	ArmData.J6.Initial_Total_position=ArmData.J6.position;	 
	ArmData.Grip.Initial_Total_position=ArmData.Grip.position;	

	arm_data_ramp_init(&J1_L_position_set,1,ArmData.J1.Initial_Total_position);	
	arm_data_ramp_init(&J2_L_position_set,1,ArmData.J2.Initial_Total_position); 
	arm_data_ramp_init(&J3_L_position_set,1,ArmData.J3.Initial_Total_position);	
	arm_data_ramp_init(&J4_L_position_set,1,ArmData.J4.Initial_Total_position); 
	arm_data_ramp_init(&J5_L_position_set,1,ArmData.J5.Initial_Total_position);	
	arm_data_ramp_init(&J6_L_position_set,1,ArmData.J6.Initial_Total_position);
	arm_data_ramp_init(&grip_L_position_set,1,ArmData.Grip.Initial_Total_position);
	
	ArmData.J1.Kp = J1_PID_KP;ArmData.J1.Kd = J1_PID_KD;
	ArmData.J2.Kp = J2_PID_KP;ArmData.J2.Kd = J2_PID_KD;
	ArmData.J3.Kp = J3_PID_KP;ArmData.J3.Kd = J3_PID_KD;
	ArmData.J4.Kp = J4_PID_KP;ArmData.J4.Kd = J4_PID_KD;
	ArmData.J5.Kp = J5_PID_KP;ArmData.J5.Kd = J5_PID_KD;
	ArmData.J6.Kp = J6_PID_KP;ArmData.J6.Kd = J6_PID_KD;
	ArmData.Grip.Kp = Grip_PID_KP;ArmData.Grip.Kd = Grip_PID_KD;
	
	fp32 J1_J2_J3_J4_J5_J6_J7_flag = 1;

	do{	
		arm_ramp_calc(&J1_L_position_set,0.002,J1_zero_position);
		arm_ramp_calc(&J2_L_position_set,0.002,J2_zero_position);
		arm_ramp_calc(&J3_L_position_set,0.002,J3_zero_position);
		arm_ramp_calc(&J4_L_position_set,0.002,J4_zero_position);
		arm_ramp_calc(&J5_L_position_set,0.002,J5_zero_position);
		arm_ramp_calc(&J6_L_position_set,0.002,J6_zero_position);
		arm_ramp_calc(&grip_L_position_set,0.002,grip_zero_position);
		
		ArmData.J1.send_set = J1_L_position_set.out;
		ArmData.J2.send_set = J2_L_position_set.out;
		ArmData.J3.send_set = J3_L_position_set.out;
		ArmData.J4.send_set = J4_L_position_set.out;
		ArmData.J5.send_set = J5_L_position_set.out;
		ArmData.J6.send_set = J6_L_position_set.out;
		ArmData.Grip.send_set = grip_L_position_set.out;
		
		limitPosition(&ArmData.J1.send_set,J1_Max_position,J1_Min_position);
		limitPosition(&ArmData.J2.send_set,J2_Max_position,J2_Min_position);
		limitPosition(&ArmData.J3.send_set,J3_Max_position,J3_Min_position);
		limitPosition(&ArmData.J4.send_set,J4_Max_position,J4_Min_position);
		limitPosition(&ArmData.J5.send_set,J5_Max_position,J5_Min_position);
		limitPosition(&ArmData.J6.send_set,J6_Max_position,J6_Min_position);
		limitPosition(&ArmData.Grip.send_set,Grip_Max_position,Grip_Min_position);

		//�ж��Ƿ�ص���ʼ��̬
		if(J1_L_position_set.out == J1_zero_position && J2_L_position_set.out == J2_zero_position && J3_L_position_set.out == J3_zero_position && J4_L_position_set.out == J4_zero_position && J5_L_position_set.out == J5_zero_position && J6_L_position_set.out == J6_zero_position && grip_L_position_set.out == grip_zero_position)
		{
				J1_J2_J3_J4_J5_J6_J7_flag = 0;
		}
		osDelay(5);
	}while(J1_J2_J3_J4_J5_J6_J7_flag);
}

void Chassis_Movement(void)
{
	
}

/*  ��λ����  */
void limitPosition(fp32* position, fp32 max_position, fp32 min_position) 
{
    if (*position >= max_position) 
    {
        *position = max_position;  // ����������ֵ�����������ֵ
    } 
    else if (*position <= min_position) 
    {
        *position = min_position;  // ���������Сֵ����������Сֵ
    }
}
  
  
