/* 模块职责：右臂（Arm_R）关节控制的早期版本：DM 关节位置斜坡、上电姿态对齐与 MIT 指令发送。 */
/* 当前状态：整个文件已被逐行注释，编译单元为空，不参与任何构建；保留仅作历史参考。 */
/* 历史依赖：FDCAN1 上的达妙关节（0x01..0x07）、VT13 遥控器、ramp_function_source_t 斜坡发生器。 */
//#include "Arm_R_task.h"
//#include "DM_motor.h"
//#include "DJI_motor.h"
//#include "can_bsp.h"
//#include "cmsis_os.h"
//#include "user_lib.h"
//#include "uart_bsp.h"
//#include "VT13_rc_ctrl.h"
//#include "LK_motor.h"

///**
// * @brief 对输入信号应用死区限制的宏定义。
// *
// * 如果输入信号的绝对值大于给定的死区阈值（dealine），则输出信号等于输入信号，
// * 否则输出信号为零。用于防止小幅度的输入波动对系统产生不必要的响应。
// *
// * @param input 输入信号。
// * @param output 输出信号，经过死区处理后的结果。
// * @param dealine 死区阈值。
// */
//#define rc_deadband_limit(input, output, dealine)    \
//  {                                                  \
//    if ((input) > (dealine) || (input) < -(dealine)) \
//    {                                                \
//      (output) = (input);                            \
//    }                                                \
//    else                                             \
//    {                                                \
//      (output) = 0;                                  \
//    }                                                \
//  }
//  
//  extern FDCAN_HandleTypeDef hfdcan1;
//  extern FDCAN_HandleTypeDef hfdcan2;
//  extern FDCAN_HandleTypeDef hfdcan3;

// 各关节目标位置斜坡发生器；注释里标明了对应的达妙电机型号（J1 暂用 DM8009）。
//  ramp_function_source_t J1_position_set;      //DM_J6248p  (暂时：8009)
//  ramp_function_source_t J2_position_set;	   //DM_J6248p
//  ramp_function_source_t J3_position_set;      //DM_4340p 
//  ramp_function_source_t J4_position_set;      //DM_4310P
//  ramp_function_source_t J5_position_set;      //DM_4310P
//  ramp_function_source_t J5_position_set;      //DM_4310P
//  ramp_function_source_t grip_position_set1;   //DM_4310
//  ramp_function_source_t grip_position_set2;
//  
// 上电初始姿态（各关节零位偏置）与关节反馈数组；ArmData_t 汇总各关节控制量。
//  /*初始姿态*/
//  float J1_initial_posture = 0.0f,J2_initial_posture = 0.0f,J3_initial_posture = 0.0f,J4_initial_posture = 0.0f,J5_initial_posture = 0.0f,J6_initial_posture = 0.0f,grip_initial_posture = 0.0f;
//  
//  joint_motor_measure_t joint_R_motor[6];
//  
//  ArmData_t ArmData;
//  RC_contorl_t RC_contorl;
//  
//  fp32 grip_pawl_flag = 1;
//  fp32 slack_pawl_flag = 1;
//  float Init_position;
//  float Last_position;
//  
/* 任务主体（历史）：初始化后以 2 ms 周期向 FDCAN1 的 0x03 号电机发送 MIT 指令（前四项为 0，末尾力矩项非零），仅用于联调。 */
//void Arm_R_task(void)
//{
//	Arm_R_Init();
//	while (1)
//	{
//		mit_ctrl(&hfdcan1, 0X03 ,0, 0, 0, 0, 1);
//		osDelay(2);
////		CAN_cmd_measure();     //测试h7与c板的双板通信
//	}
//}
//	
//  

/* 初始化（历史）：读取关节反馈作为初始位置，等待 3 s 后按 0.001 步进把目标平滑拉回零位。 */
///************************* 初始化 *************************/
//void Arm_R_Init(void)
//{
//	ArmData.J1.send_set=joint_R_motor[0].position;
//	ArmData.J2.send_set=joint_R_motor[1].position;
//	ArmData.J3.send_set=joint_R_motor[2].position;
//	ArmData.J4.send_set=joint_R_motor[3].position;
//	ArmData.J5.send_set=joint_R_motor[4].position;
//	ArmData.J6.send_set=joint_R_motor[5].position;
//	vTaskDelay(3000);
//	ArmData.J1.position=joint_R_motor[0].position;
//	ArmData.J2.position=joint_R_motor[1].position;
//	ArmData.J3.position=joint_R_motor[2].position;
//	ArmData.J4.position=joint_R_motor[3].position;
//	ArmData.J5.position=joint_R_motor[4].position;
//	ArmData.J6.position=joint_R_motor[5].position;

//	ArmData.J1.send_set=joint_R_motor[0].position;
//	ArmData.J2.send_set=joint_R_motor[1].position;
//	ArmData.J3.send_set=joint_R_motor[2].position;
//	ArmData.J4.send_set=joint_R_motor[3].position;
//	ArmData.J5.send_set=joint_R_motor[4].position;
//	ArmData.J6.send_set=joint_R_motor[5].position;

//	ArmData.J1.Initial_Total_position=ArmData.J1.position;
//	ArmData.J2.Initial_Total_position=ArmData.J2.position;	
//	ArmData.J3.Initial_Total_position=ArmData.J3.position;
//	ArmData.J4.Initial_Total_position=ArmData.J4.position;	
//	ArmData.J5.Initial_Total_position=ArmData.J5.position;
//	ArmData.J6.Initial_Total_position=ArmData.J6.position;	
//	
//	RC_contorl.RC = get_RC_measure_point();

//arm_data_ramp_init(&J1_position_set,1,ArmData.J1.Initial_Total_position);	
//arm_data_ramp_init(&J2_position_set,1,ArmData.J2.Initial_Total_position); 
//arm_data_ramp_init(&J3_position_set,1,ArmData.J3.Initial_Total_position);	
//arm_data_ramp_init(&J4_position_set,1,ArmData.J4.Initial_Total_position); 
//arm_data_ramp_init(&J5_position_set,1,ArmData.J5.Initial_Total_position);	
//arm_data_ramp_init(&J6_position_set,1,ArmData.J6.Initial_Total_position);


// 位置斜坡收敛循环：每个周期逼近一次零点，全部关节斜坡到位后清除初始化标志。
//do{	
//	ArmData.J1.Kp = J1_PID_KP;ArmData.J1.Kd = J1_PID_KD;
//	ArmData.J2.Kp = J2_PID_KP;ArmData.J2.Kd = J2_PID_KD;
//	ArmData.J3.Kp = J3_PID_KP;ArmData.J3.Kd = J3_PID_KD;
//	ArmData.J4.Kp = J4_PID_KP;ArmData.J4.Kd = J4_PID_KD;
//	ArmData.J5.Kp = J5_PID_KP;ArmData.J5.Kd = J5_PID_KD;
//	ArmData.J6.Kp = J6_PID_KP;ArmData.J5.Kd = J6_PID_KD;
//	arm_ramp_calc(&J1_position_set,0.001,J1_zero_position);
//	arm_ramp_calc(&J2_position_set,0.001,J2_zero_position);
//	arm_ramp_calc(&J3_position_set,0.001,J3_zero_position);
//	arm_ramp_calc(&J4_position_set,0.001,J4_zero_position);
//	arm_ramp_calc(&J5_position_set,0.001,J5_zero_position);
//	arm_ramp_calc(&J6_position_set,0.001,J6_zero_position);
//	ArmData.J1.send_set = J1_position_set.out;
//	ArmData.J2.send_set = J2_position_set.out;
//	ArmData.J3.send_set = J3_position_set.out;
//	ArmData.J4.send_set = J4_position_set.out;
//	ArmData.J5.send_set = J5_position_set.out;
//	ArmData.J6.send_set = J6_position_set.out;

//	if(J1_position_set.out == J1_zero_position && J2_position_set.out == J2_zero_position && J3_position_set.out == J3_zero_position && J4_position_set.out == J4_zero_position && J5_position_set.out == J5_zero_position && J6_position_set.out == J6_zero_position && J7_position_set.out == J7_zero_position)
//	{
//			J1_J2_J3_J4_J5_J6_J7_init_flag = 0;
//	}
//			
//	vTaskDelay(5);
//}while(J1_J2_J3_J4_J5_J6_J7_init_flag);


// 初始化收尾：清空遥控/按键累计量与斜坡状态，避免首帧控制量跳变。
//	/*发送清零*/
//	ArmData.J1.RC_Sum_Value = 0;
//	ArmData.J2.RC_Sum_Value = 0;
//	ArmData.J3.RC_Sum_Value = 0;
//	ArmData.J4.RC_Sum_Value = 0;
//	ArmData.J5.RC_Sum_Value = 0;
//	ArmData.J6.RC_Sum_Value = 0;
//	ArmData.J7.RC_Sum_Value = 0;

//	ArmData.J1.Key_Control_Value = 0;
//	ArmData.J2.Key_Control_Value = 0;
//	ArmData.J3.Key_Control_Value = 0;
//	ArmData.J4.Key_Control_Value = 0;
//	ArmData.J5.Key_Control_Value = 0;
//	ArmData.J6.Key_Control_Value = 0;
//	ArmData.J7.Key_Control_Value = 0;

//	arm_data_ramp_init(&J1_position_set,1,0);	
//	arm_data_ramp_init(&J2_position_set,1,0);
//	arm_data_ramp_init(&J3_position_set,1,0);	
//	arm_data_ramp_init(&J4_position_set,1,0);
//	arm_data_ramp_init(&J5_position_set,1,0);	
//	arm_data_ramp_init(&J6_position_set,1,0);
//	arm_data_ramp_init(&J7_position_set,1,0);
//	vTaskDelay(10);
//}
//  
//  
