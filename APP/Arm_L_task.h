#ifndef __ARM_L_TASK_H
#define __ARM_L_TASK_H

#include "main.h"
#include "can_bsp.h"
#include "bsp_delay.h"
#include "VT13_rc_ctrl.h"

#define M_PI 3.14159265358979323846f
#define LEFT 0
#define RIGHT 1
#define STOP 0
#define START 1


//���ؽ���λ
#define J1_Max_position 2.81695309f		
#define J1_Min_position -2.96054893f		     
#define J2_Max_position 1.7732357f		
#define J2_Min_position 0.0f		     
#define J3_Max_position 2.39008179f		
#define J3_Min_position 0.0f		 
#define J4_Max_position 4.3f		
#define J4_Min_position -4.3f		 
#define J5_Max_position 1.23497334f		
#define J5_Min_position -1.29562798f	 
#define J6_Max_position 2.22337666f		
#define J6_Min_position -2.25160637f 
#define Grip_Max_position 0.7f    	 
#define Grip_Min_position 0.0f		
/*������λ*/
#define Special_J2_limit 0.0635156631
#define J1_Special_Max_position 0.215261688
#define J1_Special_Min_position -0.215261688  //�����������??


//���ؽ�PD
#define J1_PID_KP 68.0f		//J1 KP
#define J1_PID_KD 4.0f		//J1 KD
#define J2_PID_KP 200.0f	//J2 KP
#define J2_PID_KD 4.0f		//J2 KD
#define J3_PID_KP 165.0f	//J3 KP 200
#define J3_PID_KD 3.0f		//J3 KD
#define J4_PID_KP 32.0f		//J4 KP
#define J4_PID_KD 0.5f		//J4 KD
#define J5_PID_KP 40.0f		//J5 KP
#define J5_PID_KD 0.5f		//J5 KD
#define J6_PID_KP 32.0f		//J6 KP
#define J6_PID_KD 1.0f		//J6 KD
#define Grip_PID_KP 31.0f	//Grip KP
#define Grip_PID_KD 1.0f	//Grip KD

#define Joint_Atony_KP_KD 0.0f		//���ؽ�����KP KD

//�����ؽڳ�ʼλ��
#define J1_zero_position 0.0f
#define J2_zero_position 0.0f
#define J3_zero_position 0.1f
#define J4_zero_position 0.0f
#define J5_zero_position 0.0f
#define J6_zero_position 0.0f
#define grip_zero_position 0.0f

//��צ�н�λ��
#define Grip_clamp_position 0.68f
//һ����תJ4ת��
#define J4_half_turn 4.42530632f   

//һ����������Ԫ·������
/*���??*/
/*��������Ԫ*/
#define L_J1_save_position_1 -2.87270164f
#define L_J2_save_position_1 0.82826561f
#define L_J3_save_position_1 0.648248634f
#define L_J4_save_position_1 -0.157740593f
#define L_J5_save_position_1 -1.09578896f
#define L_J6_save_position_1 0.803196907f
#define L_Grip_save_position_1 0.67f

#define L_J1_save_position_2 -2.91139366f
#define L_J2_save_position_2 0.94166754f
#define L_J3_save_position_2 0.516672264f
#define L_J4_save_position_2 -0.0753412247f
#define L_J5_save_position_2 -0.892843246f
#define L_J6_save_position_2 0.804341316f
#define L_Grip_save_position_2 0.01f

/*ȡ������Ԫ*/
#define L_J1_get_position_1 -2.87270164f
#define L_J2_get_position_1 0.82826561f
#define L_J3_get_position_1 0.648248634f
#define L_J4_get_position_1 -0.157740593f
#define L_J5_get_position_1 -1.09578896f
#define L_J6_get_position_1 0.803196907f
#define L_Grip_get_position_1 0.0f

#define L_J1_get_position_2 -2.91139366f
#define L_J2_get_position_2 0.94666754f
#define L_J3_get_position_2 0.510672264f
#define L_J4_get_position_2 -0.0753412247f
#define L_J5_get_position_2 -0.892843246f
#define L_J6_get_position_2 0.804341316f
#define L_Grip_get_position_2 0.67f

/*�Ҳ�*/
/*��������Ԫ*/
#define R_J1_save_position_1 2.77771378f
#define R_J2_save_position_1 0.84295858f
#define R_J3_save_position_1 0.646714455f
#define R_J4_save_position_1 -0.00438690186f
#define R_J5_save_position_1 -1.07709599f
#define R_J6_save_position_1 -0.741397858f
#define R_Grip_save_position_1 0.67f

#define R_J1_save_position_2 2.76932144f
#define R_J2_save_position_2 0.96061578f
#define R_J3_save_position_2 0.500839355f
#define R_J4_save_position_2 0.0036239624f
#define R_J5_save_position_2 -0.858510971f
#define R_J6_save_position_2 -0.699435234f
#define R_Grip_save_position_2 0.01f

/*ȡ������Ԫ*/
#define R_J1_get_position_1 2.77771378f
#define R_J2_get_position_1 0.84295858f
#define R_J3_get_position_1 0.646714455f
#define R_J4_get_position_1 -0.00438690186f
#define R_J5_get_position_1 -1.07709599f
#define R_J6_get_position_1 -0.741397858f
#define R_Grip_get_position_1 0.0f

#define R_J1_get_position_2 2.76932144f
#define R_J2_get_position_2 0.96261578f
#define R_J3_get_position_2 0.498839355f
#define R_J4_get_position_2 0.0036239624f
#define R_J5_get_position_2 -0.858510971f
#define R_J6_get_position_2 -0.699435234f
#define R_Grip_get_position_2 0.67f
//TAKE WATER
#define J1_TAKE_position 0.0f
#define J2_TAKE_position 1.8732357f
#define J3_TAKE_position 0.599870682f
#define J4_TAKE_position -1.98577118f
#define J5_TAKE_position -0.972190857f
#define J6_TAKE_position  0.0143051147f
#define Grip_TAKE_position_1 0.714694023f
//RESET ARM
#define J1_reset_position 0.0f
#define J2_reset_position 0.0852594376f
#define J3_reset_position 0.151254654f	
#define J4_reset_position -1.85950279f
#define J5_reset_position -0.0f
#define J6_reset_position 0.0f
#define Grip_reset_position 0.714694023f
//TAKE Grip

typedef  struct
{
	const VT13_RC_ctrl_t *RC;
	uint16_t RC_mode;
}RC_contorl_t;

typedef enum 
{
	ARM_L_MOVEMENT_J1_J4,        //��ۣ�J1-J4���˶�
	ARM_L_MOVEMENT_J5_GRIP,      //��ۣ�J5-��צ���˶�
	ARM_KEY_CONTROL,             //�����Զ��������ģ�?
	ARM_RETURN_INITIAL_POSTURE,  //�ۻع��ʼ���?�ϣ���̬
	CHASSIS_MOVEMENT_ARM_KEEP,   //�����ڱ�������������˶�??
	ARM_POWERLESS,
	TAKE_WATER,//ֹͣģʽ
	PASS_WATER,
	ARM_RESET,
	TAKE_Grip,

}Arm_mode_t; 

typedef  struct
{
		fp32 Initial_Total_position;
		fp32 position;
		fp32 send_set;
		fp32 Error;
		fp32 Now_set_position;
		fp32 Now_theta;
		fp32 Angle_Sum_Value;
		fp32 RC_Sum_Value;
		fp32 Key_Control_Value;
    fp32 Kp;
    fp32 Kd;
}DM_contorl_t;

typedef  struct
{
		fp32 Initial_Total_position;
		fp32 send_set;
		fp32 Now_set_position;
		fp32 Total_Angle;
		fp32 sent_current;
		fp32 Key_Control_Value;
}DJ_contorl_t;



typedef struct 
{
	uint8_t ArmMode;
	DM_contorl_t J1,J2,J3,J4,J5,J6,Grip;
	fp32 Total_Angle;
//	arm_set_t arm_set;  //���ȽǶ�
	fp32 pitch_position;
	fp32 roll_position;
}ArmData_t;

typedef struct
{
	fp32 DM_CC_receive;
	int32_t DJ_CC_receive;
}Custom_controller_t;


extern Custom_controller_t Custom_RC[6];
extern	ArmData_t ArmData;

void Arm_L_Task(void const * argument);
void Arm_L_Init(void);
void data_updata(void);
uint8_t GetArmMode(void);

/*ģʽ*/
void Arm_Key_Control_mode(void);  // ����ģʽ

void Arm_L_Movement_J1_J4(void);
void Arm_return_initial_position(void);
void Chassis_Movement(void);
void Arm_Powerless(void);
void TAKE_WATER_1(void);
void PASS_WATER_1(void);
//void TAKE_GRIP(void);
void ARM_RESET_1(void);
void TAKE_Grip_1(void);
void limitPosition(fp32* position, fp32 max_position, fp32 min_position);  // ��λ����



#endif
