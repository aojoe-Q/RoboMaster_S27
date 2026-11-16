/* 模块职责：右臂控制的数据结构与参数声明：关节 PD 参数、零位角度、遥控/关节控制结构体。 */
/* 当前状态：整个头文件已被逐行注释，不参与编译；定义仅作历史参考保留。 */
//#ifndef __ARM_R_TASK_H
//#define __ARM_R_TASK_H

//#include "main.h"
//#include "can_bsp.h"
//#include "bsp_delay.h"
//#include "VT13_rc_ctrl.h"

// 各关节 MIT 控制增益（Kp/Kd）初值，原注释标注“未调”。
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

// 各关节机械零位对应的电机编码器角度（弧度），用于上电姿态对齐。
////各个关节零点位置（未调）
//#define J1_zero_position 3.41679549f
//#define J2_zero_position 2.73786354f
//#define J3_zero_position 3.41679549f
//#define J4_zero_position 2.73786354f
//#define J5_zero_position 3.41679549f
//#define J6_zero_position 2.73786354f


// 遥控器控制结构体：保存 VT13 遥控数据指针与当前模式字。
//typedef  struct
//{
//	const VT13_RC_ctrl_t *RC;
//	uint16_t RC_mode;
//	
//}RC_contorl_t;

// 达妙关节控制结构体：位置/目标/误差、累计角度、遥控与按键增量，以及 MIT 的 Kp/Kd。
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

// 大疆（DJI）电机控制结构体：目标设定、累计角度、输出电流与按键增量。
//typedef  struct
//{
//		fp32 Initial_Total_position;
//		fp32 send_set;
//		fp32 Now_set_position;
//		fp32 Total_Angle;
//		fp32 sent_current;
//		fp32 Key_Control_Value;
//}DJ_contorl_t;



// 机械臂整体数据：模式字 + 6 个达妙关节 + 夹爪 + 云台 pitch/roll 角。
//typedef struct 
//{
//	uint8_t ArmMode;
//	DM_contorl_t J1,J2,J3,J4,J5,J6,Grip;
//	fp32 Total_Angle;
////	arm_set_t arm_set;  //过度角度
//	fp32 pitch_position;
//	fp32 roll_position;
//}ArmData_t;



/* 右臂初始化入口（历史声明）：文件已停用，无实现参与编译。 */
//void Arm_R_Init(void);



//#endif
