//#ifndef __ARM_R_TASK_H
//#define __ARM_R_TASK_H

//#include "main.h"
//#include "can_bsp.h"
//#include "bsp_delay.h"
//#include "VT13_rc_ctrl.h"

////各关节PD(未调)
//#define J1_PID_KP 20.0f		//J1 KP
//#define J1_PID_KD 1.0f		//J1 KD
//#define J2_PID_KP 25.0f		//J2 KP
//#define J2_PID_KD 1.0f		//J2 KD
//#define J3_PID_KP 25.0f		//J3 KP
//#define J3_PID_KD 1.0f		//J3 KD
//#define J4_PID_KP 25.0f		//J4 KP
//#define J4_PID_KD 1.0f		//J4 KD
//#define J5_PID_KP 25.0f		//J5 KP
//#define J5_PID_KD 1.0f		//J5 KD
//#define J6_PID_KP 25.0f		//J6 KP
//#define J6_PID_KD 1.0f		//J6 KD

////各个关节零点位置（未调）
//#define J1_zero_position 3.41679549f
//#define J2_zero_position 2.73786354f
//#define J3_zero_position 3.41679549f
//#define J4_zero_position 2.73786354f
//#define J5_zero_position 3.41679549f
//#define J6_zero_position 2.73786354f


//typedef  struct
//{
//	const VT13_RC_ctrl_t *RC;
//	uint16_t RC_mode;
//	
//}RC_contorl_t;

//typedef  struct
//{
//		fp32 Initial_Total_position;
//		fp32 position;
//		fp32 send_set;
//		fp32 Error;
//		fp32 Now_set_position;
//		fp32 Now_theta;
//		fp32 Angle_Sum_Value;
//		fp32 RC_Sum_Value;
//		fp32 Key_Control_Value;
//    fp32 Kp;
//    fp32 Kd;
//}DM_contorl_t;

//typedef  struct
//{
//		fp32 Initial_Total_position;
//		fp32 send_set;
//		fp32 Now_set_position;
//		fp32 Total_Angle;
//		fp32 sent_current;
//		fp32 Key_Control_Value;
//}DJ_contorl_t;



//typedef struct 
//{
//	uint8_t ArmMode;
//	DM_contorl_t J1,J2,J3,J4,J5,J6,Grip;
//	fp32 Total_Angle;
////	arm_set_t arm_set;  //过度角度
//	fp32 pitch_position;
//	fp32 roll_position;
//}ArmData_t;



//void Arm_R_Init(void);



//#endif
