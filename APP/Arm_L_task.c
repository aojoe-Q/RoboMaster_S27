#include "Arm_L_task.h"
#include "Arm_USB_task.h"
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
#include "multi_turn_store.h"
#include "multi_turn_restore.h"
#include "math.h"
/* ---- Âß¼­×ø±ê²Î¿¼µã£ºraw ±ê¶¨³£Á¿ + ±¾ÖáÉÏµçÆ«ÒÆ b ---- */
/* b = Joint_Boot_Offset[i] °Ñµç»úÉÏµçºóµÄ raw Î»ÖÃ»¹Ô­³ÉÂß¼­×ø±ê£¬
   ËùÒÔÁãµã/ÏŞÎ»/Ê¾½ÌµãÕâĞ©¡°ÔÚ raw ÏÂÁ¿³öÀ´µÄ³£Á¿¡±ÔÚ¿ØÖÆ²ã¶¼Òª +b¡£ */
#define J1_zero_L    (J1_zero_position    + Joint_Boot_Offset[0])
#define J1_max_L     (J1_Max_position     + Joint_Boot_Offset[0])
#define J1_min_L     (J1_Min_position     + Joint_Boot_Offset[0])
#define J1_reset_L   (J1_reset_position   + Joint_Boot_Offset[0])
#define J1_take_L    (J1_TAKE_position    + Joint_Boot_Offset[0])
#define J2_zero_L    (J2_zero_position    + Joint_Boot_Offset[1])
#define J2_max_L     (J2_Max_position     + Joint_Boot_Offset[1])
#define J2_min_L     (J2_Min_position     + Joint_Boot_Offset[1])
#define J2_reset_L   (J2_reset_position   + Joint_Boot_Offset[1])
#define J2_take_L    (J2_TAKE_position    + Joint_Boot_Offset[1])
#define J3_zero_L    (J3_zero_position    + Joint_Boot_Offset[2])
#define J3_max_L     (J3_Max_position     + Joint_Boot_Offset[2])
#define J3_min_L     (J3_Min_position     + Joint_Boot_Offset[2])
#define J3_reset_L   (J3_reset_position   + Joint_Boot_Offset[2])
#define J3_take_L    (J3_TAKE_position    + Joint_Boot_Offset[2])
#define J4_zero_L    (J4_zero_position    + Joint_Boot_Offset[3])
#define J4_max_L     (J4_Max_position     + Joint_Boot_Offset[3])
#define J4_min_L     (J4_Min_position     + Joint_Boot_Offset[3])
#define J4_reset_L   (J4_reset_position   + Joint_Boot_Offset[3])
#define J4_take_L    (J4_TAKE_position    + Joint_Boot_Offset[3])
#define J5_zero_L    (J5_zero_position    + Joint_Boot_Offset[4])
#define J5_max_L     (J5_Max_position     + Joint_Boot_Offset[4])
#define J5_min_L     (J5_Min_position     + Joint_Boot_Offset[4])
#define J5_reset_L   (J5_reset_position   + Joint_Boot_Offset[4])
#define J5_take_L    (J5_TAKE_position    + Joint_Boot_Offset[4])
#define J6_zero_L    (J6_zero_position    + Joint_Boot_Offset[5])
#define J6_max_L     (J6_Max_position     + Joint_Boot_Offset[5])
#define J6_min_L     (J6_Min_position     + Joint_Boot_Offset[5])
#define J6_reset_L   (J6_reset_position   + Joint_Boot_Offset[5])
#define J6_take_L    (J6_TAKE_position    + Joint_Boot_Offset[5])
#define grip_zero_L  (grip_zero_position  + Joint_Boot_Offset[6])
#define Grip_max_L   (Grip_Max_position   + Joint_Boot_Offset[6])
#define Grip_min_L   (Grip_Min_position   + Joint_Boot_Offset[6])
#define Grip_take_L  (Grip_TAKE_position_1 + Joint_Boot_Offset[6])
#define J1_zone_L    (J1_J2_ZONE_J1_MIN   + Joint_Boot_Offset[0])
#define J2_zone_L    (J1_J2_ZONE_J2_MAX   + Joint_Boot_Offset[1])

/**
 * @brief ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Åºï¿½Ó¦ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ÆµÄºê¶¨ï¿½å¡£
 *
 * ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ÅºÅµÄ¾ï¿½ï¿½ï¿½Öµï¿½ï¿½ï¿½Ú¸ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Öµï¿½ï¿½dealineï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ÅºÅµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ÅºÅ£ï¿½
 * ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Åºï¿½Îªï¿½ã¡£ï¿½ï¿½ï¿½Ú·ï¿½Ö¹Ğ¡ï¿½ï¿½ï¿½Èµï¿½ï¿½ï¿½ï¿½ë²¨ï¿½ï¿½ï¿½ï¿½ÏµÍ³ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Òªï¿½ï¿½ï¿½ï¿½Ó¦ï¿½ï¿½
 *
 * @param input ï¿½ï¿½ï¿½ï¿½ï¿½ÅºÅ¡ï¿½
 * @param output ï¿½ï¿½ï¿½ï¿½ÅºÅ£ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä½ï¿½ï¿½ï¿½ï¿½
 * @param dealine ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Öµï¿½ï¿½
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
  
  /*Ò»ï¿½ï¿½ï¿½ï¿½×ªJ4(roll)*/
  ramp_function_source_t J4_L_key2_1_set;     
  ramp_function_source_t J4_L_key2_0_set;    
  
  ButtonCounter left_button_btn;  
  ButtonCounter right_button_btn;
  ArmData_t ArmData;
  volatile uint8_t Arm_L_Ready = 0;  /* set when Arm_L_Init() has finished */
  ArmCommand_t ArmCommand;           /* å‘ç»™ Moter_Task çš„æ•´å¸§å‘½ä»¤ï¼ˆé€»è¾‘åæ ‡ï¼‰ */
  RC_contorl_t RC_contorl;
  
  joint_motor_measure_t joint_L_motor[7];
  
  
  /*ï¿½ï¿½Ê¼ï¿½ï¿½Ì¬*/
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
  /*ï¿½ï¿½ï¿½Ø½ï¿½Ğ±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½*/
  double J1_ramp=0,J2_ramp=0,J3_ramp=0,J4_ramp=0,J5_ramp=0,J6_ramp=0,Grip_ramp=0;
  
  int i=1;//ï¿½ï¿½ï¿½Ô£ï¿½
	
void Arm_L_Task(void const * argument)
{
	static uint8_t prev_ArmMode = 0xFF;
	Arm_L_Init();
	while (1)
	{
		data_updata();

		/* ï¿½ï¿½âµ½ï¿½ï¿½Ä£Ê½ï¿½Ğ»ï¿½ï¿½ï¿½ï¿½Éµï¿½Ç°Î»ï¿½Ã³ï¿½Ê¼ï¿½ï¿½ RC_Sum_Valueï¿½ï¿½ï¿½Ø±ï¿½ï¿½ï¿½ï¿½ë£©ï¿½ï¿½ï¿½ï¿½Ö¹Î»ï¿½ï¿½ï¿½ï¿½Ô¾ */
		if (ArmData.ArmMode != prev_ArmMode)
		{
			if (ArmData.ArmMode == ARM_L_MOVEMENT_J1_J4)
			{
				ArmData.J1.RC_Sum_Value = ArmData.J1.position - J1_zero_L;
				ArmData.J2.RC_Sum_Value = ArmData.J2.position - J2_zero_L;
				ArmData.J3.RC_Sum_Value = ArmData.J3.position - J3_zero_L;
				ArmData.J4.RC_Sum_Value = ArmData.J4.position - J4_zero_L;
				ArmData.J5.RC_Sum_Value = ArmData.J5.position - J5_zero_L - Custom_RC[4].DM_CC_receive - ArmData.J5.Key_Control_Value;
				ArmData.J6.RC_Sum_Value = ArmData.J6.position - J6_zero_L - Custom_RC[5].DM_CC_receive - ArmData.J6.Key_Control_Value;
				ArmData.Grip.RC_Sum_Value = ArmData.Grip.position - grip_zero_L;
				/* ¼Ğ×¦Ğ±ÆÂ´Ó½øÈëÊ±µÄÊµ¼ÊÎ»ÖÃÆğËã£¬±ÜÃâµÚÒ»Ö¡´Ó¾ÉÄ¿±êÆğÌø */
				grip_L_position_set.out = ArmData.Grip.position;
			}
			else if (ArmData.ArmMode == ARM_MODE_PC_CONTROL)
			{
				Arm_PC_Control_Enter();
			}
			else if (ArmData.ArmMode == ARM_RETURN_INITIAL_POSTURE)
			{
				/* ¸´Î»Ä£Ê½£¨·Ç×èÈû£©£ºĞ±ÆÂ¶ÔÆëµ½µ±Ç°Êµ¼ÊÎ»ÖÃ£¬Ö®ºóÃ¿ÖÜÆÚÏòÁãµã×ßÒ»²½ */
				J1_L_position_set.out = ArmData.J1.position;
				J2_L_position_set.out = ArmData.J2.position;
				J3_L_position_set.out = ArmData.J3.position;
				J4_L_position_set.out = ArmData.J4.position;
				J5_L_position_set.out = ArmData.J5.position;
				J6_L_position_set.out = ArmData.J6.position;
				grip_L_position_set.out = ArmData.Grip.position;
			}
			/* Leaving PC mode (or never in it): an enable from before the
			   switch must not move the arm when it comes back. */
			if (ArmData.ArmMode != ARM_MODE_PC_CONTROL)
			{
				Arm_USB_PC_Disarm();
			}
			prev_ArmMode = ArmData.ArmMode;
		}

		switch (ArmData.ArmMode)
        {
					
			case TAKE_WATER:
				TAKE_WATER_1();
				break;
			case PASS_WATER:
				PASS_WATER_1();
				break;
			case ARM_RESET:
				ARM_RESET_1();
				break;
			case TAKE_Grip:
				TAKE_Grip_1();
			break;
			case ARM_L_MOVEMENT_J1_J4:
				Arm_L_Movement_J1_J4();
			break;
			case ARM_MODE_PC_CONTROL:
				Arm_PC_Control();
			break;

			case ARM_RETURN_INITIAL_POSTURE:
				Arm_return_initial_position();
			break;
			case ARM_L_MOVEMENT_J5_GRIP:
				/* ¸ÃÄ£Ê½ÉĞÎ´ÊµÏÖ£º±£³ÖÎŞÁ¦£¬±ÜÃâÎó¶¯×÷ */
				Arm_Powerless();
			break;
			case ARM_POWERLESS:
				Arm_Powerless();
			break;
			// Ä¬ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Í£Ö¹ï¿½ï¿½ï¿½Ë¶ï¿½
			default:
				Arm_Powerless();
            break;
        }
		Arm_Command_Publish();
		/* µôµç±£³Ö£ºÈÎÒâÄ£Ê½ÏÂ¾²Ö¹¼´¼ÇÂ¼Âß¼­Î»ÖÃ£¨¾²Ö¹/·ù¶È/¼ä¸ôÓÉ MT_Restore_Process ÄÚ²¿ÅĞ¶¨£©£¬¼Ğ×¦ÓĞÍâÁ¦¾ØÊ±²»´æ */
		MT_Restore_Process((uint8_t)(Arm_L_Ready && !Arm_Grip_Torque_Request()));
		osDelay(8);
		
	}
}
	
  

/************************* ï¿½ï¿½Ê¼ï¿½ï¿½ *************************/
void Arm_L_Init(void)
{
	osDelay(950);
	button_init(&left_button_btn);
	button_init(&right_button_btn);
	/* æ‰ç”µä¿æŒæ—¥å¿—ï¼šæ‰«æä¸æ¢åŒºæ“¦é™¤å¿…é¡»åœ¨ç”µæœºä½¿èƒ½ä¹‹å‰å®Œæˆ */
	MT_Store_Boot_Maintenance();
	
	/*ï¿½Ø½Úµï¿½ï¿½Ê¹ï¿½ï¿½*/
	osDelay(15);
	enable_motor_mode(&hfdcan1, 0x01, 0);
	osDelay(15);
	enable_motor_mode(&hfdcan1, 0x02, 0);
	osDelay(15);
	enable_motor_mode(&hfdcan1, 0x03, 0);
	osDelay(10);
	enable_motor_mode(&hfdcan1, 0x04, 0);
	osDelay(5);
	enable_motor_mode(&hfdcan1, 0x05, 0);
	osDelay(5);
	enable_motor_mode(&hfdcan1, 0x06, 0);
	osDelay(5);
	enable_motor_mode(&hfdcan1, 0x07, 0);
	osDelay(5);
	/* ç­‰å¾… 7 è½´åé¦ˆå¹¶è®¡ç®—é€»è¾‘åæ ‡åç§» bï¼ˆMT_RESTORE_ENABLE=0 æ—¶ b=0ï¼‰ */
	MT_Restore_Boot();
	
	
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
//	save_motor_zero(&hfdcan1, 0X06, 0);
//	osDelay(5);
//	save_motor_zero(&hfdcan1, 0X07, 0);
//	osDelay(5);

	//ï¿½ï¿½ï¿½Öµï¿½ï¿½Î»ï¿½ï¿½
	ArmData.J1.send_set=JOINT_LOGICAL_POSITION(0);
	ArmData.J2.send_set=JOINT_LOGICAL_POSITION(1);
	ArmData.J3.send_set=JOINT_LOGICAL_POSITION(2);
	ArmData.J4.send_set=JOINT_LOGICAL_POSITION(3);
	ArmData.J5.send_set=JOINT_LOGICAL_POSITION(4);
	ArmData.J6.send_set=JOINT_LOGICAL_POSITION(5);
	ArmData.Grip.send_set=JOINT_LOGICAL_POSITION(6);
	vTaskDelay(3600);
	ArmData.J1.position=JOINT_LOGICAL_POSITION(0);
	ArmData.J2.position=JOINT_LOGICAL_POSITION(1);
	ArmData.J3.position=JOINT_LOGICAL_POSITION(2);
	ArmData.J4.position=JOINT_LOGICAL_POSITION(3);
	ArmData.J5.position=JOINT_LOGICAL_POSITION(4);
	ArmData.J6.position=JOINT_LOGICAL_POSITION(5);
	ArmData.Grip.position=JOINT_LOGICAL_POSITION(6);

	ArmData.J1.send_set=JOINT_LOGICAL_POSITION(0);
	ArmData.J2.send_set=JOINT_LOGICAL_POSITION(1);
	ArmData.J3.send_set=JOINT_LOGICAL_POSITION(2);
	ArmData.J4.send_set=JOINT_LOGICAL_POSITION(3);
	ArmData.J5.send_set=JOINT_LOGICAL_POSITION(4);
	ArmData.J6.send_set=JOINT_LOGICAL_POSITION(5);
	ArmData.Grip.send_set=JOINT_LOGICAL_POSITION(6);

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
	arm_ramp_calc(&J1_L_position_set,0.0038,J1_zero_L);
	arm_ramp_calc(&J2_L_position_set,0.0035,J2_zero_L);
	arm_ramp_calc(&J3_L_position_set,0.0035,J3_zero_L);
	arm_ramp_calc(&J4_L_position_set,0.0035,J4_zero_L);
	arm_ramp_calc(&J5_L_position_set,0.0035,J5_zero_L);
	arm_ramp_calc(&J6_L_position_set,0.0035,J6_zero_L);
	arm_ramp_calc(&grip_L_position_set,0.0035,grip_zero_L);
	
	ArmData.J1.send_set = J1_L_position_set.out;
	ArmData.J2.send_set = J2_L_position_set.out;
	ArmData.J3.send_set = J3_L_position_set.out;
	ArmData.J4.send_set = J4_L_position_set.out;
	ArmData.J5.send_set = J5_L_position_set.out;
	ArmData.J6.send_set = J6_L_position_set.out;
	ArmData.Grip.send_set = grip_L_position_set.out;
	
	/* »ØÁãÄ¿±ê¾ÍÊÇ±ê¶¨Áãµã£ºÕâÀï²»ÄÜÔÙ°´ J2_Min/J3_Min=0 ÕâÀà¹¤×÷ÏŞÎ»Ç¯Î»£¬
	   ·ñÔòÍ£ÔÚ¸º½Ç¶ÈµÄÖáÓÀÔ¶»Ø²»µ½Áãµã£¨do-while ÅĞ¾İÓÃµÄÊÇÎ´Ç¯Î»µÄ out£¬»á¼Ù×°µ½Î»£© */
//	limitPosition(&ArmData.J1.send_set,J1_Max_position,J1_Min_position);
//	limitPosition(&ArmData.J2.send_set,J2_Max_position,J2_Min_position);
//	limitPosition(&ArmData.J3.send_set,J3_Max_position,J3_Min_position);
//	limitPosition(&ArmData.J4.send_set,J4_Max_position,J4_Min_position);
//	limitPosition(&ArmData.J5.send_set,J5_Max_position,J5_Min_position);
//	limitPosition(&ArmData.J6.send_set,J6_Max_position,J6_Min_position);
//	limitPosition(&ArmData.Grip.send_set,Grip_Max_position,Grip_Min_position);
	Arm_Command_Publish();

	//ï¿½Ğ¶ï¿½ï¿½Ç·ï¿½Øµï¿½ï¿½ï¿½Ê¼ï¿½ï¿½Ì¬
	if(J1_L_position_set.out == J1_zero_L && J2_L_position_set.out == J2_zero_L && J3_L_position_set.out == J3_zero_L && J4_L_position_set.out == J4_zero_L && J5_L_position_set.out == J5_zero_L && J6_L_position_set.out == J6_zero_L && grip_L_position_set.out == grip_zero_L)
	{
			J1_J2_J3_J4_J5_J6_J7_init_flag = 0;
	}
	osDelay(5);
}while(J1_J2_J3_J4_J5_J6_J7_init_flag);


	/*ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½*/
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

    arm_data_ramp_init(&J1_L_position_set,1,ArmData.J1.position);
	arm_data_ramp_init(&J2_L_position_set,1,ArmData.J2.position);
	arm_data_ramp_init(&J3_L_position_set,1,ArmData.J3.position);
	arm_data_ramp_init(&J4_L_position_set,1,ArmData.J4.position);
	arm_data_ramp_init(&J5_L_position_set,1,ArmData.J5.position);
	arm_data_ramp_init(&J6_L_position_set,1,ArmData.J6.position);
	arm_data_ramp_init(&grip_L_position_set,1,ArmData.Grip.position);
	vTaskDelay(10);
	Arm_L_Ready = 1;
}

/************************* ï¿½ï¿½ï¿½İ¸ï¿½ï¿½ï¿½ *************************/
void data_updata(void)
{
//	RC_contorl.RC = get_RC_measure_point();
//	left_button_stop_state = RC_contorl.RC->rc.left_button;
//	right_button_stop_state = RC_contorl.RC->rc.right_button;
//	// ï¿½ï¿½ï¿½Â°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//    button_update(&left_button_btn, left_button_stop_state);
//	button_update(&right_button_btn, right_button_stop_state);
//	// ï¿½ï¿½È¡ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//    left_button_press_count = button_get_count(&left_button_btn);
//	right_button_press_count = button_get_count(&right_button_btn);
	
	ArmData.ArmMode = GetArmMode();
	
	/* é€»è¾‘ä½ç½® P = p + bï¼›åŸå§‹åé¦ˆä»ä¿ç•™åœ¨ joint_L_motor[] */
	ArmData.J1.position = JOINT_LOGICAL_POSITION(0);
	ArmData.J2.position = JOINT_LOGICAL_POSITION(1);
	ArmData.J3.position = JOINT_LOGICAL_POSITION(2);
	ArmData.J4.position = JOINT_LOGICAL_POSITION(3);
	ArmData.J5.position = JOINT_LOGICAL_POSITION(4);
	ArmData.J6.position = JOINT_LOGICAL_POSITION(5);
	ArmData.Grip.position = JOINT_LOGICAL_POSITION(6);
}

/************************* å‘½ä»¤å‘å¸ƒ *************************/
/* å³ä¸Šå·¦ä¸­ï¼šå¤¹çˆªåŠ›çŸ©å¤¹å–ï¼ˆåŸ Moter_Task è¯» DT7 åŸå§‹æ‹¨æ†ï¼Œæ”¹ä¸ºä¸æ¨¡å¼åˆ¤æ–­åŒä¸€ RCDataï¼‰ */
uint8_t Arm_Grip_Torque_Request(void)
{
	return (uint8_t)(switch_is_up(RCData.rc.s[RC_Right]) && switch_is_mid(RCData.rc.s[RC_Left]));
}

void Arm_Command_Publish(void)
{
	const DM_contorl_t *joint[7] = {
		&ArmData.J1, &ArmData.J2, &ArmData.J3, &ArmData.J4,
		&ArmData.J5, &ArmData.J6, &ArmData.Grip
	};
	uint8_t grip_torque = Arm_Grip_Torque_Request();

	taskENTER_CRITICAL();
	for (uint8_t i = 0; i < 7; i++)
	{
		ArmCommand.position[i] = joint[i]->send_set;
		ArmCommand.kp[i]       = joint[i]->Kp;
		ArmCommand.kd[i]       = joint[i]->Kd;
		ArmCommand.torque[i]   = 0.0f;
	}
	if (grip_torque)
	{
		/* ä¸åŸå‘é€ä¸€è‡´ï¼šå¤¹çˆª Kp=Kd=0ï¼Œåªç»™åŠ›çŸ© */
		ArmCommand.position[6] = ArmData.Grip.position;
		ArmCommand.kp[6]       = 0.0f;
		ArmCommand.kd[6]       = 0.0f;
		ArmCommand.torque[6]   = ARM_GRIP_TORQUE;
	}
	ArmCommand.tick = xTaskGetTickCount();
	ArmCommand.seq++;
	if (ArmCommand.seq == 0U)
	{
		ArmCommand.seq = 1U;
	}
	taskEXIT_CRITICAL();
}

/************************* æ¨¡å¼åˆ¤æ–­ *************************/
uint8_t GetArmMode(void)
{
    uint8_t mode = 0;

    /* å·¦ä¸Šå³ä¸Šï¼šåªæ‰§è¡Œ PC æ§åˆ¶ï¼ˆè¦†ç›–åŸ TAKE_WATERï¼‰ã€‚PC æœªä½¿èƒ½ã€æŒ‡ä»¤è¶…æ—¶ã€
       USB æ–­å¼€æˆ– CAN åé¦ˆä¸¢å¤±æ—¶ï¼ŒArm_PC_Control() ä¿æŒé”å­˜çš„å½“å‰ä½ç½®ã€‚ */
	if (switch_is_up(RCData.rc.s[RC_Right]) && switch_is_up(RCData.rc.s[RC_Left]))
	{
			mode = ARM_MODE_PC_CONTROL;
	}
	else if (switch_is_mid(RCData.rc.s[RC_Right])&&switch_is_up(RCData.rc.s[RC_Left]))  //left_button_press_count % 2 != 0&&RC_contorl.RC->rc.mode_sw == 0
	{
			mode = PASS_WATER;
	}
	else if (switch_is_mid(RCData.rc.s[RC_Right])&&switch_is_mid(RCData.rc.s[RC_Left]))  //RC_contorl.RC->rc.shutter == 1  //switch_is_up(RCData.rc.s[RC_Right])&&switch_is_up(RCData.rc.s[RC_Left])
	{
			mode = ARM_RESET;
	}
	else if (switch_is_mid(RCData.rc.s[RC_Right])&&switch_is_down(RCData.rc.s[RC_Left]))  //left_button_press_count % 2 != 0&&RC_contorl.RC->rc.mode_sw == 2
	{
			mode = TAKE_Grip;/*ï¿½Ø¹ï¿½ï¿½Ê¼ï¿½ï¿½Ì¬*/
	}
		else if (switch_is_down(RCData.rc.s[RC_Right])&&switch_is_up(RCData.rc.s[RC_Left]))  //left_button_press_count % 2 != 0&&RC_contorl.RC->rc.mode_sw == 2
	{
			mode = ARM_L_MOVEMENT_J1_J4;/*ï¿½Ø¹ï¿½ï¿½Ê¼ï¿½ï¿½Ì¬*/
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

/*************************** Ä£Ê½ ****************************/
/*  ï¿½ï¿½ï¿½ï¿½Ä£Ê½  */
void TAKE_Grip_1(void)
{
	ArmData.J1.Kp = J1_PID_KP;ArmData.J1.Kd = J1_PID_KD;
	ArmData.J2.Kp = J2_PID_KP;ArmData.J2.Kd = J2_PID_KD;
	ArmData.J3.Kp = J3_PID_KP;ArmData.J3.Kd = J3_PID_KD;
	ArmData.J4.Kp = J4_PID_KP;ArmData.J4.Kd = J4_PID_KD;
	ArmData.J5.Kp = J5_PID_KP;ArmData.J5.Kd = J5_PID_KD;
	ArmData.J6.Kp = J6_PID_KP;ArmData.J6.Kd = J6_PID_KD;
	ArmData.Grip.Kp = J6_PID_KP;ArmData.Grip.Kd = J6_PID_KD;

	J1_send_set = J1_reset_L;
	J2_send_set = J2_reset_L;
	J3_send_set = J3_reset_L;
	J4_send_set = J4_reset_L;
	J5_send_set = J5_reset_L;
	J6_send_set = J6_reset_L;
	Grip_send_set = grip_zero_L;

	J1_ramp= 0.0090,J2_ramp= 0.003,J3_ramp= 0.0045,J4_ramp= 0.0042,J5_ramp= 0.003,J6_ramp= 0.003,Grip_ramp= 0.030;  //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½

	/*ï¿½ï¿½×¶ï¿½Ò»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Øµï¿½Î»ï¿½ï¿½ï¿½Ù±Õºï¿½ï¿½ï¿½×¦È¡Ë®*/

	/*ï¿½ï¿½Î»*/
	limitPosition(&J1_send_set,J1_max_L,J1_min_L);
	limitPosition(&J2_send_set,J2_max_L,J2_min_L);
	limitPosition(&J3_send_set,J3_max_L,J3_min_L);
	limitPosition(&J4_send_set,J4_max_L,J4_min_L);
	limitPosition(&J5_send_set,J5_max_L,J5_min_L);
	limitPosition(&J6_send_set,J6_max_L,J6_min_L);
	limitPosition(&Grip_send_set,Grip_max_L,Grip_min_L);

	/*Ğ±ï¿½Âºï¿½ï¿½ï¿½*/
	arm_ramp_calc(&J1_L_position_set,J1_ramp,J1_send_set);
	arm_ramp_calc(&J2_L_position_set,J2_ramp,J2_send_set);
	arm_ramp_calc(&J3_L_position_set,J3_ramp,J3_send_set);
	arm_ramp_calc(&J4_L_position_set,J4_ramp,J4_send_set);
	arm_ramp_calc(&J5_L_position_set,J5_ramp,J5_send_set);
	arm_ramp_calc(&J6_L_position_set,J6_ramp,J6_send_set);
	arm_ramp_calc(&grip_L_position_set,Grip_ramp,Grip_send_set);

	/*ï¿½ï¿½ï¿½Î»ï¿½Ã¸ï¿½Öµ*/
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

	J1_send_set = J1_reset_L;
	J2_send_set = J2_reset_L;
	J3_send_set = J3_reset_L;
	J4_send_set = J4_reset_L;
	J5_send_set = J5_reset_L;
	J6_send_set = J6_reset_L;
	Grip_send_set = Grip_take_L;

	J1_ramp= 0.0090,J2_ramp= 0.003,J3_ramp= 0.0045,J4_ramp= 0.0042,J5_ramp= 0.003,J6_ramp= 0.003,Grip_ramp= 0.030;  //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½

	/*ï¿½ï¿½×¶ï¿½Ò»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Øµï¿½Î»ï¿½ï¿½ï¿½Ù±Õºï¿½ï¿½ï¿½×¦È¡Ë®*/

	/*ï¿½ï¿½Î»*/
	limitPosition(&J1_send_set,J1_max_L,J1_min_L);
	limitPosition(&J2_send_set,J2_max_L,J2_min_L);
	limitPosition(&J3_send_set,J3_max_L,J3_min_L);
	limitPosition(&J4_send_set,J4_max_L,J4_min_L);
	limitPosition(&J5_send_set,J5_max_L,J5_min_L);
	limitPosition(&J6_send_set,J6_max_L,J6_min_L);
	limitPosition(&Grip_send_set,Grip_max_L,Grip_min_L);

	/*Ğ±ï¿½Âºï¿½ï¿½ï¿½*/
	arm_ramp_calc(&J1_L_position_set,J1_ramp,J1_send_set);
	arm_ramp_calc(&J2_L_position_set,J2_ramp,J2_send_set);
	arm_ramp_calc(&J3_L_position_set,J3_ramp,J3_send_set);
	arm_ramp_calc(&J4_L_position_set,J4_ramp,J4_send_set);
	arm_ramp_calc(&J5_L_position_set,J5_ramp,J5_send_set);
	arm_ramp_calc(&J6_L_position_set,J6_ramp,J6_send_set);
	arm_ramp_calc(&grip_L_position_set,Grip_ramp,Grip_send_set);

	/*ï¿½ï¿½ï¿½Î»ï¿½Ã¸ï¿½Öµ*/
	ArmData.J1.send_set = J1_L_position_set.out;
	ArmData.J2.send_set = J2_L_position_set.out;
	ArmData.J3.send_set = J3_L_position_set.out;
	ArmData.J4.send_set = J4_L_position_set.out;
	ArmData.J5.send_set = J5_L_position_set.out;
	ArmData.J6.send_set = J6_L_position_set.out;
	ArmData.Grip.send_set = grip_L_position_set.out;

}

void PASS_WATER_1(void)
{
	ArmData.J1.Kp = J1_PID_KP;ArmData.J1.Kd = J1_PID_KD;
	ArmData.J2.Kp = J2_PID_KP;ArmData.J2.Kd = J2_PID_KD;
	ArmData.J3.Kp = J3_PID_KP;ArmData.J3.Kd = J3_PID_KD;
	ArmData.J4.Kp = J4_PID_KP;ArmData.J4.Kd = J4_PID_KD;
	ArmData.J5.Kp = J5_PID_KP;ArmData.J5.Kd = J5_PID_KD;
	ArmData.J6.Kp = J6_PID_KP;ArmData.J6.Kd = J6_PID_KD;
	ArmData.Grip.Kp = J6_PID_KP;ArmData.Grip.Kd = J6_PID_KD;

	J1_send_set = J1_take_L;
	J2_send_set = J2_take_L;
	J3_send_set = J3_take_L;
	J4_send_set = J4_take_L;
	J5_send_set = J5_take_L;
	J6_send_set = J6_take_L;
	Grip_send_set = Grip_take_L;

	J1_ramp= 0.0090,J2_ramp= 0.003,J3_ramp= 0.0045,J4_ramp= 0.0042,J5_ramp= 0.003,J6_ramp= 0.003,Grip_ramp= 0.030;  

	
	limitPosition(&J1_send_set,J1_max_L,J1_min_L);
	limitPosition(&J2_send_set,J2_max_L,J2_min_L);
	limitPosition(&J3_send_set,J3_max_L,J3_min_L);
	limitPosition(&J4_send_set,J4_max_L,J4_min_L);
	limitPosition(&J5_send_set,J5_max_L,J5_min_L);
	limitPosition(&J6_send_set,J6_max_L,J6_min_L);
	limitPosition(&Grip_send_set,Grip_max_L,Grip_min_L);

	/*Ğ±ï¿½Âºï¿½ï¿½ï¿½*/
	arm_ramp_calc(&J1_L_position_set,J1_ramp,J1_send_set);
	arm_ramp_calc(&J2_L_position_set,J2_ramp,J2_send_set);
	arm_ramp_calc(&J3_L_position_set,J3_ramp,J3_send_set);
	arm_ramp_calc(&J4_L_position_set,J4_ramp,J4_send_set);
	arm_ramp_calc(&J5_L_position_set,J5_ramp,J5_send_set);
	arm_ramp_calc(&J6_L_position_set,J6_ramp,J6_send_set);
	arm_ramp_calc(&grip_L_position_set,Grip_ramp,Grip_send_set);

	/*ï¿½ï¿½ï¿½Î»ï¿½Ã¸ï¿½Öµ*/
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

	J1_send_set = J1_take_L;
	J2_send_set = J2_take_L;
	J3_send_set = J3_take_L;
	J4_send_set = J4_take_L;
	J5_send_set = J5_take_L;
	J6_send_set = J6_take_L;
	Grip_send_set = grip_zero_L;

//	Grip_send_set = grip_zero_position;

	J1_ramp= 0.0090,J2_ramp= 0.003,J3_ramp= 0.0045,J4_ramp= 0.0042,J5_ramp= 0.003,J6_ramp= 0.003,Grip_ramp= 0.030;  //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½

	/*ï¿½ï¿½×¶ï¿½Ò»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Øµï¿½Î»ï¿½ï¿½ï¿½Ù±Õºï¿½ï¿½ï¿½×¦È¡Ë®*/

	/*ï¿½ï¿½Î»*/
	limitPosition(&J1_send_set,J1_max_L,J1_min_L);
	limitPosition(&J2_send_set,J2_max_L,J2_min_L);
	limitPosition(&J3_send_set,J3_max_L,J3_min_L);
	limitPosition(&J4_send_set,J4_max_L,J4_min_L);
	limitPosition(&J5_send_set,J5_max_L,J5_min_L);
	limitPosition(&J6_send_set,J6_max_L,J6_min_L);
	limitPosition(&Grip_send_set,Grip_max_L,Grip_min_L);

	/*Ğ±ï¿½Âºï¿½ï¿½ï¿½*/
	arm_ramp_calc(&J1_L_position_set,J1_ramp,J1_send_set);
	arm_ramp_calc(&J2_L_position_set,J2_ramp,J2_send_set);
	arm_ramp_calc(&J3_L_position_set,J3_ramp,J3_send_set);
	arm_ramp_calc(&J4_L_position_set,J4_ramp,J4_send_set);
	arm_ramp_calc(&J5_L_position_set,J5_ramp,J5_send_set);
	arm_ramp_calc(&J6_L_position_set,J6_ramp,J6_send_set);
	arm_ramp_calc(&grip_L_position_set,Grip_ramp,Grip_send_set);

	/*ï¿½ï¿½ï¿½Î»ï¿½Ã¸ï¿½Öµ*/
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
//	static fp32 J4_half = 0.0f;//J4×ªï¿½ï¿½
//	static int end_direction = -1; //
//	
//	/*ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½PDï¿½ï¿½Öµ*/
//	ArmData.J1.Kp = J1_PID_KP;ArmData.J1.Kd = J1_PID_KD;
//	ArmData.J2.Kp = J2_PID_KP;ArmData.J2.Kd = J2_PID_KD;
//	ArmData.J3.Kp = J3_PID_KP;ArmData.J3.Kd = J3_PID_KD;
//	ArmData.J4.Kp = J4_PID_KP;ArmData.J4.Kd = J4_PID_KD;
//	ArmData.J5.Kp = J5_PID_KP;ArmData.J5.Kd = J5_PID_KD;
//	ArmData.J6.Kp = J6_PID_KP;ArmData.J6.Kd = J6_PID_KD;
//	ArmData.Grip.Kp = J6_PID_KP;ArmData.Grip.Kd = J6_PID_KD;
//	
//	/*(ï¿½ï¿½)Ò»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ôª*/
//	if (((RCData.key.v & KEY_PRESSED_OFFSET_Q) && (RCData.mouse.press_l) && mode_lock == 0) || L_save_flag == 1){/*Ò»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ôª*/
//		if (mode_lock == 0) mode_lock = 1;
//		L_save_flag = 1;
//		switch (L_save_flag_move)
//        {	
//			case 1:/*ï¿½×¶ï¿½Ò»*/
//			{	
//				J1_send_set = L_J1_save_position_1;
//				J2_send_set = L_J2_save_position_1;
//				J3_send_set = L_J3_save_position_1;
//				J4_send_set = L_J4_save_position_1;
//				J5_send_set = L_J5_save_position_1;
//				J6_send_set = L_J6_save_position_1;
//				Grip_send_set = Grip_clamp_position;
//				
//				J1_ramp= 0.0090,J2_ramp= 0.003,J3_ramp= 0.0045,J4_ramp= 0.0042,J5_ramp= 0.003,J6_ramp= 0.003,Grip_ramp= 0.030;  //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					L_save_flag_move =2;
//				}
//			}
//				break;
//			case 2:/*ï¿½×¶Î¶ï¿½*///ï¿½ï¿½×¦ï¿½ï¿½ï¿½ï¿½ï¿½æ´¢ï¿½ï¿½ï¿½ï¿½(ï¿½ï¿½ÒªÏ¸ï¿½ï¿½ï¿½ï¿½)
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
//				J1_ramp= 0.00450,J2_ramp= 0.003,J3_ramp= 0.0015,J4_ramp= 0.0015,J5_ramp= 0.002,J6_ramp= 0.0015,Grip_ramp= 0.00350;  //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					L_save_flag_move =3;
//					}
//			}
//				break;
//			case 3:/*ï¿½×¶ï¿½ï¿½ï¿½*/
//			{
//				J1_send_set = L_J1_save_position_1;
//				J2_send_set = L_J2_save_position_1;
//				J3_send_set = L_J3_save_position_1;
//				J4_send_set = L_J4_save_position_1;
//				J5_send_set = L_J5_save_position_1;
//				J6_send_set = L_J6_save_position_1;
//				Grip_send_set = L_Grip_save_position_2;
//				
//				J1_ramp= 0.0090,J2_ramp= 0.003,J3_ramp= 0.0045,J4_ramp= 0.0015,J5_ramp= 0.003,J6_ramp= 0.0015,Grip_ramp= 0.030;  //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					L_save_flag_move = 4;
//				}
//			}
//				break;
//			case 4:/*ï¿½×¶ï¿½ï¿½ï¿½*/
//			{
//				J1_send_set = J1_zero_position;
//				J2_send_set = J2_zero_position;
//				J3_send_set = J3_zero_position;
//				J4_send_set = J4_zero_position;
//				J5_send_set = J5_zero_position;
//				J6_send_set = J6_zero_position;
//				Grip_send_set = grip_zero_position;
//				
//				J1_ramp= 0.01,J2_ramp= 0.0028,J3_ramp= 0.002,J4_ramp= 0.003,J5_ramp= 0.0035,J6_ramp= 0.004,Grip_ramp= 0.0030;  //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
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
//	/*(ï¿½ï¿½)Ò»ï¿½ï¿½È¡ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ôª*/
//	else if(((RCData.key.v & KEY_PRESSED_OFFSET_Q) && (RCData.mouse.press_r) && mode_lock == 0) || L_get_flag == 1){
//		if (mode_lock == 0) mode_lock = 1;
//	    L_get_flag = 1;
//		switch (L_get_flag_move)
//        {	
//			case 1:/*ï¿½×¶ï¿½Ò»*/
//			{
//				J1_send_set = L_J1_get_position_1;
//				J2_send_set = L_J2_get_position_1;
//				J3_send_set = L_J3_get_position_1;
//				J4_send_set = L_J4_get_position_1;
//				J5_send_set = L_J5_get_position_1;
//				J6_send_set = L_J6_get_position_1;
//				Grip_send_set = L_Grip_get_position_1;
//			
//				J1_ramp= 0.0090,J2_ramp= 0.002,J3_ramp= 0.003,J4_ramp= 0.005,J5_ramp= 0.003,J6_ramp= 0.002,Grip_ramp= 0.020;  //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					L_get_flag_move =2;
//				}
//			}
//				break;
//			case 2:/*ï¿½×¶Î¶ï¿½*/
//			{
//				J1_send_set = L_J1_get_position_2;
//				J2_send_set = L_J2_get_position_2;
//				J3_send_set = L_J3_get_position_2;
//				J4_send_set = L_J4_get_position_2;
//				J5_send_set = L_J5_get_position_2;
//				J6_send_set = L_J6_get_position_2;
//				Grip_send_set = L_Grip_get_position_1;
//				
//				J1_ramp= 0.0030,J2_ramp= 0.0015,J3_ramp= 0.001,J4_ramp= 0.001,J5_ramp= 0.0015,J6_ramp= 0.0023,Grip_ramp= 0.020;  //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					L_get_flag_move =3;
//				}
//			}
//				break;
//			case 3:/*ï¿½×¶ï¿½ï¿½ï¿½*///ï¿½ï¿½×¦ï¿½ï¿½×¡È¡ï¿½ï¿½ï¿½ï¿½Ôª
//			{
//				Grip_send_set = Grip_clamp_position;
//				Grip_ramp= 0.0150;  //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//				if(grip_L_position_set.out == Grip_send_set){
//					L_get_flag_move = 4;
//				}
//			}
//				break;
//			case 4:/*ï¿½×¶ï¿½ï¿½ï¿½*/
//			{
//				J1_send_set = L_J1_get_position_1;
//				J2_send_set = L_J2_get_position_1;
//				J3_send_set = L_J3_get_position_1;
//				J4_send_set = L_J4_get_position_1;
//				J5_send_set = L_J5_get_position_1;
//				J6_send_set = L_J6_get_position_1;
//				Grip_send_set = Grip_clamp_position;
//				J1_ramp= 0.0030,J2_ramp= 0.001,J3_ramp= 0.001,J4_ramp= 0.001,J5_ramp= 0.0015,J6_ramp= 0.0015,Grip_ramp= 0.0020;  //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					L_get_flag_move = 5;
//				}
//			}
//				break;
//			case 5:/*ï¿½×¶ï¿½ï¿½ï¿½*///Ì§ï¿½ï¿½J3(ï¿½ï¿½Ö¹ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ôªï¿½ï¿½Î§ï¿½ï¿½ï¿½ï¿½)
//			{
//				J3_send_set = 0.737690163f;
//				J1_ramp= 0.0050,J2_ramp= 0.002,J3_ramp= 0.003,J4_ramp= 0.002,J5_ramp= 0.002,J6_ramp= 0.003,Grip_ramp= 0.020;  //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//				if(J3_L_position_set.out == J3_send_set){
//					L_get_flag_move = 6;
//				}
//				break;
//			}
//			case 6:/*ï¿½×¶ï¿½ï¿½ï¿½*/
//			{
//				J1_send_set = J1_zero_position;
//				J2_send_set = J2_zero_position;
//				J3_send_set = J3_zero_position;
//				J4_send_set = J4_zero_position;
//				J5_send_set = J5_zero_position;
//				J6_send_set = J6_zero_position;
//				Grip_send_set = Grip_clamp_position;
//				J1_ramp= 0.01,J2_ramp= 0.0028,J3_ramp= 0.003,J4_ramp= 0.004,J5_ramp= 0.004,J6_ramp= 0.0036,Grip_ramp= 0.020;  //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
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
//	/*(ï¿½ï¿½)Ò»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ôª*/
//	else if (((RCData.key.v & KEY_PRESSED_OFFSET_E) && (RCData.mouse.press_l) && mode_lock == 0) || R_save_flag == 1){/*Ò»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ôª*/
//	if (mode_lock == 0) mode_lock = 1;
//		R_save_flag = 1;
//		switch (R_save_flag_move)
//        {	
//			case 1:/*ï¿½×¶ï¿½Ò»*/
//			{	
//				J1_send_set = R_J1_save_position_1;
//				J2_send_set = R_J2_save_position_1;
//				J3_send_set = R_J3_save_position_1;
//				J4_send_set = R_J4_save_position_1;
//				J5_send_set = R_J5_save_position_1;
//				J6_send_set = R_J6_save_position_1;
//				Grip_send_set = Grip_clamp_position;
//				
//				J1_ramp= 0.0090,J2_ramp= 0.003,J3_ramp= 0.0045,J4_ramp= 0.0042,J5_ramp= 0.0035,J6_ramp= 0.002,Grip_ramp= 0.030;  //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					R_save_flag_move = 2;
//				}
//			}
//				break;
//			case 2:/*ï¿½×¶Î¶ï¿½*///ï¿½ï¿½×¦ï¿½ï¿½ï¿½ï¿½ï¿½æ´¢ï¿½ï¿½ï¿½ï¿½(ï¿½ï¿½ÒªÏ¸ï¿½ï¿½ï¿½ï¿½)
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
//				J1_ramp= 0.00450,J2_ramp= 0.003,J3_ramp= 0.0015,J4_ramp= 0.0015,J5_ramp= 0.002,J6_ramp= 0.0015,Grip_ramp= 0.00450;  //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					R_save_flag_move =3;
//					}
//			}
//				break;
//			case 3:/*ï¿½×¶ï¿½ï¿½ï¿½*/
//			{
//				J1_send_set = R_J1_save_position_1;
//				J2_send_set = R_J2_save_position_1;
//				J3_send_set = R_J3_save_position_1;
//				J4_send_set = R_J4_save_position_1;
//				J5_send_set = R_J5_save_position_1;
//				J6_send_set = R_J6_save_position_1;
//				Grip_send_set = R_Grip_save_position_2;
//				
//				J1_ramp= 0.0090,J2_ramp= 0.003,J3_ramp= 0.0045,J4_ramp= 0.0015,J5_ramp= 0.003,J6_ramp= 0.0015,Grip_ramp= 0.030;  //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					R_save_flag_move = 4;
//				}
//			}
//				break;
//			case 4:/*ï¿½×¶ï¿½ï¿½ï¿½*/
//			{
//				J1_send_set = J1_zero_position;
//				J2_send_set = J2_zero_position;
//				J3_send_set = J3_zero_position;
//				J4_send_set = J4_zero_position;
//				J5_send_set = J5_zero_position;
//				J6_send_set = J6_zero_position;
//				Grip_send_set = grip_zero_position;
//				
//				J1_ramp= 0.01,J2_ramp= 0.0028,J3_ramp= 0.002,J4_ramp= 0.003,J5_ramp= 0.0035,J6_ramp= 0.004,Grip_ramp= 0.0030;  //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
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
//	/*(ï¿½ï¿½)Ò»ï¿½ï¿½È¡ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ôª*/
//	else if(((RCData.key.v & KEY_PRESSED_OFFSET_E) && (RCData.mouse.press_r) && mode_lock == 0) || R_get_flag == 1){
//	if (mode_lock == 0) mode_lock = 1;
//	    R_get_flag = 1;
//		switch (R_get_flag_move)
//        {	
//			case 1:/*ï¿½×¶ï¿½Ò»*/
//			{
//				J1_send_set = R_J1_get_position_1;
//				J2_send_set = R_J2_get_position_1;
//				J3_send_set = R_J3_get_position_1;
//				J4_send_set = R_J4_get_position_1;
//				J5_send_set = R_J5_get_position_1;
//				J6_send_set = R_J6_get_position_1;
//				Grip_send_set =R_Grip_get_position_1;
//			
//				J1_ramp= 0.0090,J2_ramp= 0.002,J3_ramp= 0.003,J4_ramp= 0.005,J5_ramp= 0.003,J6_ramp= 0.002,Grip_ramp= 0.020;  //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					R_get_flag_move =2;
//				}
//			}
//				break;
//			case 2:/*ï¿½×¶Î¶ï¿½*/
//			{
//				J1_send_set = R_J1_get_position_2;
//				J2_send_set = R_J2_get_position_2;
//				J3_send_set = R_J3_get_position_2;
//				J4_send_set = R_J4_get_position_2;
//				J5_send_set = R_J5_get_position_2;
//				J6_send_set = R_J6_get_position_2;
//				Grip_send_set = R_Grip_get_position_1;
//				
//				J1_ramp= 0.0030,J2_ramp= 0.0015,J3_ramp= 0.001,J4_ramp= 0.001,J5_ramp= 0.0015,J6_ramp= 0.0023,Grip_ramp= 0.020;  //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					R_get_flag_move = 3;
//				}
//			}
//				break;
//			case 3:/*ï¿½×¶ï¿½ï¿½ï¿½*///ï¿½ï¿½×¦ï¿½ï¿½×¡È¡ï¿½ï¿½ï¿½ï¿½Ôª
//			{
//				Grip_send_set = Grip_clamp_position;
//				Grip_ramp= 0.0150;  //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//				if(grip_L_position_set.out == Grip_send_set){
//					R_get_flag_move = 4;
//				}
//			}
//				break;
//			case 4:/*ï¿½×¶ï¿½ï¿½ï¿½*/
//			{
//				J1_send_set = R_J1_get_position_1;
//				J2_send_set = R_J2_get_position_1;
//				J3_send_set = R_J3_get_position_1;
//				J4_send_set = R_J4_get_position_1;
//				J5_send_set = R_J5_get_position_1;
//				J6_send_set = R_J6_get_position_1;
//				Grip_send_set = Grip_clamp_position;
//				
//				J1_ramp= 0.0030,J2_ramp= 0.001,J3_ramp= 0.001,J4_ramp= 0.001,J5_ramp= 0.0015,J6_ramp= 0.0015,Grip_ramp= 0.0020;  //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//				
//				if(J1_L_position_set.out == J1_send_set && J2_L_position_set.out == J2_send_set && J3_L_position_set.out == J3_send_set && J4_L_position_set.out == J4_send_set && J5_L_position_set.out == J5_send_set && J6_L_position_set.out == J6_send_set && grip_L_position_set.out == Grip_send_set){
//					R_get_flag_move = 5;
//				}
//			}
//				break;
//			case 5:/*ï¿½×¶ï¿½ï¿½ï¿½*///Ì§ï¿½ï¿½J3(ï¿½ï¿½Ö¹ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ôªï¿½ï¿½Î§ï¿½ï¿½ï¿½ï¿½)
//			{
//				J3_send_set = 0.725537453f;
//				J1_ramp= 0.0050,J2_ramp= 0.002,J3_ramp= 0.003,J4_ramp= 0.002,J5_ramp= 0.002,J6_ramp= 0.003,Grip_ramp= 0.020;  //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//				if(J3_L_position_set.out == J3_send_set){
//					R_get_flag_move = 6;
//				}
//				break;
//			}
//			case 6:/*ï¿½×¶ï¿½ï¿½ï¿½*/
//			{
//				J1_send_set = J1_zero_position;
//				J2_send_set = J2_zero_position;
//				J3_send_set = J3_zero_position;
//				J4_send_set = J4_zero_position;
//				J5_send_set = J5_zero_position;
//				J6_send_set = J6_zero_position;
//				Grip_send_set = Grip_clamp_position;
//				J1_ramp= 0.01,J2_ramp= 0.0028,J3_ramp= 0.002,J4_ramp= 0.004,J5_ramp= 0.004,J6_ramp= 0.0036,Grip_ramp= 0.020;  //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
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
//		/*ï¿½ï¿½×¦ï¿½ï¿½ï¿½ï¿½*/
//		if(CC_data.CC_Key_grip == 1)
//		Grip_send_set = grip_zero_position + Grip_clamp_position;
//		if(CC_data.CC_Key_grip == 0)
//		Grip_send_set = grip_zero_position;
//		
//		/*J4Ò»ï¿½ï¿½ï¿½ï¿½×ª*/
//		if(CC_data.CC_Key_J4 ==1){
//		J4_half = J4_half_turn;  //ï¿½ï¿½×ªÒ»È¦
//			end_direction = 1;
//			}
//		if(CC_data.CC_Key_J4 ==0){
//		J4_half = 0;  //ï¿½ï¿½×ªÒ»È¦
//			end_direction = -1;
//		}
//		
//		/*ï¿½ï¿½Î»*/
//		limitPosition(&CC_data.DM_CC_2_receive,J2_Max_position - ArmData.J2.Initial_Total_position,J2_Min_position - ArmData.J2.Initial_Total_position);
//		limitPosition(&CC_data.DM_CC_3_receive,J3_Max_position - ArmData.J3.Initial_Total_position,J3_Min_position - ArmData.J3.Initial_Total_position);
//		limitPosition(&CC_data.DM_CC_4_receive,J4_Max_position - ArmData.J4.Initial_Total_position,J4_Min_position - ArmData.J4.Initial_Total_position);
//		limitPosition(&limit_data5,J5_Max_position - ArmData.J5.Initial_Total_position,J5_Min_position - ArmData.J5.Initial_Total_position);
//		limitPosition(&limit_data6,J6_Max_position - ArmData.J6.Initial_Total_position,J6_Min_position - ArmData.J6.Initial_Total_position);
//		/*ï¿½ï¿½ï¿½Í¸ï¿½MITï¿½ï¿½ï¿½ï¿½Ä£Ê½ï¿½ÂµÄºï¿½ï¿½ï¿½ï¿½ï¿½Î»ï¿½ï¿½*/
//		J1_set_initial_data = J1_zero_position + CC_data.DM_CC_1_receive;  //J1_zero_position + ArmData.J1.RC_Sum_Value + Custom_RC[0].DM_CC_receive;
//		J2_set_initial_data = J2_zero_position + CC_data.DM_CC_2_receive;
//		J3_set_initial_data = J3_zero_position + CC_data.DM_CC_3_receive;
//		J4_set_initial_data = J4_zero_position + CC_data.DM_CC_4_receive * 4.0;
//		J5_set_initial_data = J5_zero_position + CC_data.DJ_CC_1_receive * 0.000019103116231;//2.48340511/130000
//		J6_set_initial_data = J6_zero_position - CC_data.DJ_CC_2_receive * 0.0008;
//	    // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Í¹ï¿½ï¿½È±ï¿½ï¿½ï¿½	
//		J1_send_set = J1_zero_position + CC_data.DM_CC_1_receive;    //J1_zero_position + ArmData.J1.RC_Sum_Value + Custom_RC[0].DM_CC_receive + ArmData.J1.Key_Control_Value;//ï¿½ï¿½nï¿½ï¿½ï¿½Ø½Úµï¿½ï¿½ï¿½ï¿½Ê¼Î»ï¿½ï¿½+Ò£ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½+ï¿½Ô¶ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//		J2_send_set = J2_zero_position + CC_data.DM_CC_2_receive;
//		J3_send_set = J3_zero_position + CC_data.DM_CC_3_receive;
//		J4_send_set = J4_zero_position + CC_data.DM_CC_4_receive * 4.0 + J4_half;

//		limit_data5 = CC_data.DJ_CC_1_receive * 0.000019103116231;
//		limit_data6 = CC_data.DJ_CC_2_receive * 0.0008;
//		
//		J5_send_set = end_direction * (J5_zero_position + limit_data5);//ï¿½ï¿½nï¿½ï¿½ï¿½Ø½Úµï¿½ï¿½ï¿½ï¿½Ê¼Î»ï¿½ï¿½+Ò£ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½+ï¿½Ô¶ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//		J6_send_set = J6_zero_position - limit_data6;
//		
//	}
//	
//	/*ï¿½ï¿½Î»*/
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
//	//Ğ±ï¿½Âºï¿½ï¿½ï¿½
//	arm_ramp_calc(&J1_L_position_set,J1_ramp,J1_send_set);
//	arm_ramp_calc(&J2_L_position_set,J2_ramp,J2_send_set);
//	arm_ramp_calc(&J3_L_position_set,J3_ramp,J3_send_set);
//	arm_ramp_calc(&J4_L_position_set,J4_ramp,J4_send_set);
//	arm_ramp_calc(&J5_L_position_set,J5_ramp,J5_send_set);
//	arm_ramp_calc(&J6_L_position_set,J6_ramp,J6_send_set);	
//	arm_ramp_calc(&grip_L_position_set,Grip_ramp,Grip_send_set);
//	/*ï¿½ï¿½ï¿½Î»ï¿½Ã¸ï¿½Öµ*/	
//	ArmData.J1.send_set = J1_L_position_set.out;
//	ArmData.J2.send_set = J2_L_position_set.out;
//	ArmData.J3.send_set = J3_L_position_set.out;
//	ArmData.J4.send_set = J4_L_position_set.out;
//	ArmData.J5.send_set = J5_L_position_set.out;
//	ArmData.J6.send_set = J6_L_position_set.out;
//	ArmData.Grip.send_set = grip_L_position_set.out;
//	
//	/*ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç°Î»ï¿½Ã¸ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä£Ê½*/
//	ArmData.J1.Now_set_position = ArmData.J1.send_set;
//	ArmData.J2.Now_set_position = ArmData.J2.send_set;
//	ArmData.J3.Now_set_position = ArmData.J3.send_set;
//	ArmData.J4.Now_set_position = ArmData.J4.send_set;
//	ArmData.J5.Now_set_position = ArmData.J5.send_set;
//	ArmData.J6.Now_set_position = ArmData.J6.send_set;
//	ArmData.Grip.Now_set_position = ArmData.Grip.send_set;
//}

/*ï¿½ï¿½ï¿½ï¿½Ä£Ê½*/
void Arm_Powerless(void)
{
	ArmData.J1.Kp = ArmData.J2.Kp = ArmData.J3.Kp = ArmData.J4.Kp = ArmData.J5.Kp = ArmData.J6.Kp = ArmData.Grip.Kp = ArmData.J1.Kd = ArmData.J2.Kd = ArmData.J3.Kd = ArmData.J4.Kd = ArmData.J5.Kd = ArmData.J6.Kd = ArmData.Grip.Kd = Joint_Atony_KP_KD;
	/*ï¿½ï¿½ï¿½Íµï¿½Ç°Î»ï¿½ï¿½*/
	ArmData.J1.send_set=ArmData.J1.position;
	ArmData.J2.send_set=ArmData.J2.position;
	ArmData.J3.send_set=ArmData.J3.position;	
	ArmData.J4.send_set=ArmData.J4.position;
	ArmData.J5.send_set=ArmData.J5.position;
	ArmData.J6.send_set=ArmData.J6.position;
	ArmData.Grip.send_set=ArmData.Grip.position;
	/*ï¿½ï¿½ï¿½èµ±Ç°Î»ï¿½ï¿½*/
	ArmData.J1.Initial_Total_position=ArmData.J1.position;
	ArmData.J2.Initial_Total_position=ArmData.J2.position;
	ArmData.J3.Initial_Total_position=ArmData.J3.position;
	ArmData.J4.Initial_Total_position=ArmData.J4.position;
	ArmData.J5.Initial_Total_position=ArmData.J5.position;
	ArmData.J6.Initial_Total_position=ArmData.J6.position;
	ArmData.Grip.Initial_Total_position=ArmData.Grip.position;
	
	/*ï¿½ï¿½ï¿½ï¿½Ğ±ï¿½Âºï¿½ï¿½ï¿½ï¿½á¹¹ï¿½åµ±Ç°Î»ï¿½ï¿½*/
	J1_L_position_set.out = ArmData.J1.position;
	J2_L_position_set.out = ArmData.J2.position;
	J3_L_position_set.out = ArmData.J3.position;
	J4_L_position_set.out = ArmData.J4.position;
	J5_L_position_set.out = ArmData.J5.position;
	J6_L_position_set.out = ArmData.J6.position;
	grip_L_position_set.out = ArmData.Grip.position;
	
	/*Ò£ï¿½ï¿½ï¿½Û¼ï¿½ï¿½ï¿½ï¿½ï¿½*/
	ArmData.J1.RC_Sum_Value = 0;
	ArmData.J2.RC_Sum_Value = 0;
	ArmData.J3.RC_Sum_Value = 0;
	ArmData.J4.RC_Sum_Value = 0;
	ArmData.J5.RC_Sum_Value = 0;
	ArmData.J6.RC_Sum_Value = 0;
	ArmData.Grip.RC_Sum_Value = 0;

}

/************************* PC æ§åˆ¶æ¨¡å¼ (ROS 2 / MoveIt) *************************/
static uint8_t pc_holding = 0;   /* 1: å·²é”å­˜ä¿æŒä½ç½® */

/* æ¯æ¬¡è¿›å…¥ PC æ¨¡å¼æ—¶è°ƒç”¨ï¼šä¸‹ä¸€æ¬¡æ²¡æœ‰æœ‰æ•ˆç›®æ ‡æ—¶é‡æ–°é”å­˜å½“å‰ä½ç½® */
void Arm_PC_Control_Enter(void)
{
	pc_holding = 0;
}

void Arm_PC_Control(void)
{
	static float pc_target[7];
	static float hold_position[7];
	uint8_t got_target;

	/* è®¾ç½®æ­£å¸¸ PID å¢ç›Šï¼ˆå‚æ•°ä¸å˜ï¼‰ */
	ArmData.J1.Kp = J1_PID_KP; ArmData.J1.Kd = J1_PID_KD;
	ArmData.J2.Kp = J2_PID_KP; ArmData.J2.Kd = J2_PID_KD;
	ArmData.J3.Kp = J3_PID_KP; ArmData.J3.Kd = J3_PID_KD;
	ArmData.J4.Kp = J4_PID_KP; ArmData.J4.Kd = J4_PID_KD;
	ArmData.J5.Kp = J5_PID_KP; ArmData.J5.Kd = J5_PID_KD;
	ArmData.J6.Kp = J6_PID_KP; ArmData.J6.Kd = J6_PID_KD;
	ArmData.Grip.Kp = J6_PID_KP; ArmData.Grip.Kd = J6_PID_KD;

	/* å°è¯•è¯»å– PC ç›®æ ‡ï¼ˆUSB ä»»åŠ¡å·²æ£€æŸ¥ä¼šè¯ã€ä½¿èƒ½ã€è¶…æ—¶ã€USB è¿æ¥ã€CAN åé¦ˆï¼‰ */
	got_target = Arm_USB_Get_PC_Target(pc_target);

	if (got_target)
	{
		/* ç›®æ ‡æœ‰æ•ˆï¼šè·Ÿéš PC æŒ‡ä»¤ */
		pc_holding = 0;
		ArmData.J1.send_set = pc_target[0];
		ArmData.J2.send_set = pc_target[1];
		ArmData.J3.send_set = pc_target[2];
		ArmData.J4.send_set = pc_target[3];
		ArmData.J5.send_set = pc_target[4];
		ArmData.J6.send_set = pc_target[5];
		ArmData.Grip.send_set = pc_target[6];
	}
	else
	{
		/* ç›®æ ‡æ— æ•ˆï¼ˆè¶…æ—¶/æ–­å¼€/CAN ä¸¢å¤±ï¼‰ï¼šé”å­˜å¹¶ä¿æŒå½“å‰ä½ç½®ï¼ˆå®‰å…¨åŠ¨ä½œ Aï¼‰ */
		if (!pc_holding)
		{
			hold_position[0] = ArmData.J1.position;
			hold_position[1] = ArmData.J2.position;
			hold_position[2] = ArmData.J3.position;
			hold_position[3] = ArmData.J4.position;
			hold_position[4] = ArmData.J5.position;
			hold_position[5] = ArmData.J6.position;
			hold_position[6] = ArmData.Grip.position;
			pc_holding = 1;
		}
		ArmData.J1.send_set = hold_position[0];
		ArmData.J2.send_set = hold_position[1];
		ArmData.J3.send_set = hold_position[2];
		ArmData.J4.send_set = hold_position[3];
		ArmData.J5.send_set = hold_position[4];
		ArmData.J6.send_set = hold_position[5];
		ArmData.Grip.send_set = hold_position[6];
	}

	/* æ ‡å‡†é™æ€é™ä½ï¼ˆé˜²å¾¡æ·±åº¦ï¼‰ã€‚PC æ¨¡å¼ä¸æ‰§è¡Œ J1/J2 ç¢°æ’åŒºé™ä½ï¼Œ
	   è¯¥é™ä½åªåœ¨é¥æ§æ¨¡å¼ä¸­ç”Ÿæ•ˆã€‚ */
	limitPosition(&ArmData.J1.send_set,J1_max_L,J1_min_L);
	limitPosition(&ArmData.J2.send_set,J2_max_L,J2_min_L);
	limitPosition(&ArmData.J3.send_set,J3_max_L,J3_min_L);
	limitPosition(&ArmData.J4.send_set,J4_max_L,J4_min_L);
	limitPosition(&ArmData.J5.send_set,J5_max_L,J5_min_L);
	limitPosition(&ArmData.J6.send_set,J6_max_L,J6_min_L);
	limitPosition(&ArmData.Grip.send_set,Grip_max_L,Grip_min_L);

	/* åŒæ­¥æ–œå¡çŠ¶æ€ï¼Œé˜²æ­¢åˆ‡å‡º PC æ¨¡å¼åè·³å˜ */
	J1_L_position_set.out = ArmData.J1.send_set;
	J2_L_position_set.out = ArmData.J2.send_set;
	J3_L_position_set.out = ArmData.J3.send_set;
	J4_L_position_set.out = ArmData.J4.send_set;
	J5_L_position_set.out = ArmData.J5.send_set;
	J6_L_position_set.out = ArmData.J6.send_set;
	grip_L_position_set.out = ArmData.Grip.send_set;
}

/*************************************Ò£ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½*******************************************/

void Arm_L_Movement_J1_J4(void)
{
	int16_t channel_0,channel_1,channel_2,channel_3;
	/*ï¿½ï¿½È¡Ò£ï¿½ï¿½ï¿½ï¿½Í¨ï¿½ï¿½ï¿½ï¿½ï¿½ë£¬Ó¦ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½*/
	rc_deadband_limit(RCData.rc.ch[0],channel_0,10);
	rc_deadband_limit(RCData.rc.ch[1],channel_1,10);
	rc_deadband_limit(RCData.rc.ch[2],channel_2,10);
	rc_deadband_limit(RCData.rc.ch[3],channel_3,10);
	/*ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½PDï¿½ï¿½Öµ*/
	ArmData.J1.Kp = J1_PID_KP;ArmData.J1.Kd = J1_PID_KD;
	ArmData.J2.Kp = J2_PID_KP;ArmData.J2.Kd = J2_PID_KD;
	ArmData.J3.Kp = J3_PID_KP;ArmData.J3.Kd = J3_PID_KD;
	ArmData.J4.Kp = J4_PID_KP;ArmData.J4.Kd = J4_PID_KD;
	ArmData.J5.Kp = J5_PID_KP;ArmData.J5.Kd = J5_PID_KD;
	ArmData.J6.Kp = J6_PID_KP;ArmData.J6.Kd = J6_PID_KD;
	ArmData.Grip.Kp = Grip_PID_KP;ArmData.Grip.Kd = Grip_PID_KD;
	/* ±¾Ä£Ê½Ã»ÓĞ±ğ´¦¸ø Grip_ramp ¸³Öµ£¬ÕâÀïÏÔÊ½¸ø³ö¼Ğ×¦Ğ±ÂÊ */
	Grip_ramp = 0.030;
	/*Ò£ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Û¼ï¿½Öµ*/                 
	/* ÀÛ¼ÓÏŞÎ»£º´°¿Ú = Á¿³Ì - Áãµã - ¸½¼ÓÆ«ÖÃ£¨Custom_RC / Key£©¡£
	   ÕâÀï¿ÌÒâ²»ÓÃ Initial_Total_position£ºÄÇÊÇÉÏµçÊ±µÄÒ»´ÎĞÔ×ËÌ¬£¬»á°ÑÕû¸ö´°¿ÚÆ½ÒÆ£¬
	   ½á¹ûÊÇÖ¸ÁîÒÑ¾­¶¥µ½Á¿³Ì¡¢ÀÛ¼ÓÆ÷»¹ÔÚ¼ÌĞø³¤£¬·´ÏòÊ±Òª¿ÕÍÆÒ»¶Î²Å¶¯¡£ */
	ArmData.J1.RC_Sum_Value += channel_0*0.00001;
	ArmData.J2.RC_Sum_Value += channel_1*0.00001;
	ArmData.J3.RC_Sum_Value += channel_0*0.00001;
	ArmData.J4.RC_Sum_Value += channel_1*0.00002;
	ArmData.J5.RC_Sum_Value += channel_2*0.00002;
	ArmData.J6.RC_Sum_Value += channel_2*0.00002;
	ArmData.Grip.RC_Sum_Value += channel_3*0.00005;

	if(ArmData.J2.position < J2_zone_L)
	{
		limitPosition(&ArmData.J1.RC_Sum_Value,J1_J2_ZONE_J1_MIN - J1_zero_position,J1_Min_position - J1_zero_position);
	}
	else
	{
		limitPosition(&ArmData.J1.RC_Sum_Value,J1_Max_position - J1_zero_position,J1_Min_position - J1_zero_position);
	}
	limitPosition(&ArmData.J2.RC_Sum_Value,J2_Max_position - J2_zero_position,J2_Min_position - J2_zero_position);
	limitPosition(&ArmData.J3.RC_Sum_Value,J3_Max_position - J3_zero_position,J3_Min_position - J3_zero_position);
	limitPosition(&ArmData.J4.RC_Sum_Value,J4_Max_position - J4_zero_position,J4_Min_position - J4_zero_position);
	limitPosition(&ArmData.J5.RC_Sum_Value,J5_Max_position - J5_zero_position - Custom_RC[4].DM_CC_receive - ArmData.J5.Key_Control_Value,J5_Min_position - J5_zero_position - Custom_RC[4].DM_CC_receive - ArmData.J5.Key_Control_Value);
	limitPosition(&ArmData.J6.RC_Sum_Value,J6_Max_position - J6_zero_position - Custom_RC[5].DM_CC_receive - ArmData.J6.Key_Control_Value,J6_Min_position - J6_zero_position - Custom_RC[5].DM_CC_receive - ArmData.J6.Key_Control_Value);
	limitPosition(&ArmData.Grip.RC_Sum_Value,Grip_Max_position - grip_zero_position - ArmData.Grip.Key_Control_Value,Grip_Min_position - grip_zero_position - ArmData.Grip.Key_Control_Value);
	
	/*ï¿½ï¿½ï¿½Í¸ï¿½MITï¿½ï¿½ï¿½ï¿½Ä£Ê½ï¿½ÂµÄºï¿½ï¿½ï¿½ï¿½ï¿½Î»ï¿½ï¿½*/
	J1_set_initial_data = J1_zero_L + ArmData.J1.RC_Sum_Value;  //J1_zero_L + ArmData.J1.RC_Sum_Value + Custom_RC[0].DM_CC_receive;
	J2_set_initial_data = J2_zero_L + ArmData.J2.RC_Sum_Value;
	J3_set_initial_data = J3_zero_L + ArmData.J3.RC_Sum_Value;
	J4_set_initial_data = J4_zero_L + ArmData.J4.RC_Sum_Value;
	J5_set_initial_data = J5_zero_L + ArmData.J5.RC_Sum_Value + Custom_RC[4].DM_CC_receive;
	J6_set_initial_data = J6_zero_L + ArmData.J6.RC_Sum_Value + Custom_RC[5].DM_CC_receive;
	Grip_set_initial_data = grip_zero_L + ArmData.Grip.RC_Sum_Value;

	J1_send_set = J1_zero_L + ArmData.J1.RC_Sum_Value;    //J1_zero_L + ArmData.J1.RC_Sum_Value + Custom_RC[0].DM_CC_receive + ArmData.J1.Key_Control_Value;//ï¿½ï¿½nï¿½ï¿½ï¿½Ø½Úµï¿½ï¿½ï¿½ï¿½Ê¼Î»ï¿½ï¿½+Ò£ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½+ï¿½Ô¶ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
	J2_send_set = J2_zero_L + ArmData.J2.RC_Sum_Value;
	J3_send_set = J3_zero_L + ArmData.J3.RC_Sum_Value;
	J4_send_set = J4_zero_L + ArmData.J4.RC_Sum_Value;
	J5_send_set = J5_zero_L + ArmData.J5.RC_Sum_Value + Custom_RC[4].DM_CC_receive + ArmData.J5.Key_Control_Value;
	J6_send_set = J6_zero_L + ArmData.J6.RC_Sum_Value + Custom_RC[5].DM_CC_receive + ArmData.J6.Key_Control_Value;
	Grip_send_set = grip_zero_L + ArmData.Grip.RC_Sum_Value + ArmData.Grip.Key_Control_Value;
	
	if(ArmData.J2.position < J2_zone_L)
	{
		limitPosition(&J1_send_set,J1_max_L,J1_zone_L);
	}	
	else
	{
		limitPosition(&J1_send_set,J1_max_L,J1_min_L);
	}  
	/*ï¿½ï¿½ï¿½Î»ï¿½Ã¸ï¿½Öµ*/
	ArmData.J1.send_set = J1_send_set;
	ArmData.J2.send_set = J2_send_set;
	ArmData.J3.send_set = J3_send_set;
	ArmData.J4.send_set = J4_send_set;
	ArmData.J5.send_set = J5_send_set;
	ArmData.J6.send_set = J6_send_set;
	ArmData.Grip.send_set = Grip_send_set;

	/* Í¬²½Ğ±ÆÂ×´Ì¬Á¿µ½µ±Ç°Êµ¼ÊÎ»ÖÃ£¬±ÜÃâÇĞÄ£Ê½Ê±´ÓÁãµãÆğÌø£¨¼Ğ×¦ÀıÍâ£º¼Ğ×¦×ßĞ±ÆÂ£© */
	J1_L_position_set.out = ArmData.J1.position;
	J2_L_position_set.out = ArmData.J2.position;
	J3_L_position_set.out = ArmData.J3.position;
	J4_L_position_set.out = ArmData.J4.position;
	J5_L_position_set.out = ArmData.J5.position;
	J6_L_position_set.out = ArmData.J6.position;

//	Arm_Key_Control_mode();

	/*ÏŞÎ»*/
	/* ¸÷Öá RC_Sum µÄÇ¯Î»ÒÑÍ³Ò»Å²µ½ÉÏÃæµÄÀÛ¼Ó´¦£¬ÕâÀï²»ÔÙÖØ¸´ */

//				J1_send_set = R_J1_get_position_1;
//				J2_send_set = R_J2_get_position_1;
//				J3_send_set = R_J3_get_position_1;
//				J4_send_set = R_J4_get_position_1;
//				J5_send_set = R_J5_get_position_1;
//				J6_send_set = R_J6_get_position_1;
//				Grip_send_set =R_Grip_get_position_1;	
	if(ArmData.J2.position < J2_zone_L)
	{
		limitPosition(&ArmData.J1.send_set,J1_max_L,J1_zone_L);
	}	
	else
	{
		limitPosition(&ArmData.J1.send_set,J1_max_L,J1_min_L);
	}  
	limitPosition(&ArmData.J1.send_set,J1_max_L,J1_min_L);
	limitPosition(&ArmData.J2.send_set,J2_max_L,J2_min_L);
	limitPosition(&ArmData.J3.send_set,J3_max_L,J3_min_L);
	limitPosition(&ArmData.J4.send_set,J4_max_L,J4_min_L);
	limitPosition(&ArmData.J5.send_set,J5_max_L,J5_min_L);
	limitPosition(&ArmData.J6.send_set,J6_max_L,J6_min_L);
	limitPosition(&ArmData.Grip.send_set,Grip_max_L,Grip_min_L);

	/* ¼Ğ×¦Ğ±ÆÂ£º±¾Ä£Ê½ÒÔÇ°Ã»ÓĞµ÷ÓÃ arm_ramp_calc£¬Grip_ramp ĞÎÍ¬ĞéÉè */
	arm_ramp_calc(&grip_L_position_set,Grip_ramp,ArmData.Grip.send_set);
	ArmData.Grip.send_set = grip_L_position_set.out;
	
	/*ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç°Î»ï¿½Ã¸ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä£Ê½*/
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
	/* ·Ç×èÈû¸´Î»£ºÃ¿¸ö 8 ms ÖÜÆÚÏòÁãµãÍÆ½øÒ»²½£¬ÓÉ Arm_L_Task Ö÷Ñ­»·µ÷ÓÃ£»
	   ½øÈë±¾Ä£Ê½Ê±Ä£Ê½ÇĞ»»¿éÒÑ°Ñ¸÷ÖáĞ±ÆÂ¶ÔÆëµ½Êµ¼ÊÎ»ÖÃ¡£ */
	ArmData.J1.Kp = J1_PID_KP;ArmData.J1.Kd = J1_PID_KD;
	ArmData.J2.Kp = J2_PID_KP;ArmData.J2.Kd = J2_PID_KD;
	ArmData.J3.Kp = J3_PID_KP;ArmData.J3.Kd = J3_PID_KD;
	ArmData.J4.Kp = J4_PID_KP;ArmData.J4.Kd = J4_PID_KD;
	ArmData.J5.Kp = J5_PID_KP;ArmData.J5.Kd = J5_PID_KD;
	ArmData.J6.Kp = J6_PID_KP;ArmData.J6.Kd = J6_PID_KD;
	ArmData.Grip.Kp = Grip_PID_KP;ArmData.Grip.Kd = Grip_PID_KD;

	arm_ramp_calc(&J1_L_position_set,0.002,J1_zero_L);
	arm_ramp_calc(&J2_L_position_set,0.002,J2_zero_L);
	arm_ramp_calc(&J3_L_position_set,0.002,J3_zero_L);
	arm_ramp_calc(&J4_L_position_set,0.002,J4_zero_L);
	arm_ramp_calc(&J5_L_position_set,0.002,J5_zero_L);
	arm_ramp_calc(&J6_L_position_set,0.002,J6_zero_L);
	arm_ramp_calc(&grip_L_position_set,0.002,grip_zero_L);

	ArmData.J1.send_set = J1_L_position_set.out;
	ArmData.J2.send_set = J2_L_position_set.out;
	ArmData.J3.send_set = J3_L_position_set.out;
	ArmData.J4.send_set = J4_L_position_set.out;
	ArmData.J5.send_set = J5_L_position_set.out;
	ArmData.J6.send_set = J6_L_position_set.out;
	ArmData.Grip.send_set = grip_L_position_set.out;
	/* »ØÁãÍ¬Ñù²»ÄÜÓÃ J2_Min/J3_Min=0 Ö®ÀàµÄ¹¤×÷ÏŞÎ»Ç¯Î»£¬·ñÔòµ½²»ÁËÁãµã */
}

void Chassis_Movement(void)
{
	
}

/*  ï¿½ï¿½Î»ï¿½ï¿½ï¿½ï¿½  */
void limitPosition(fp32* position, fp32 max_position, fp32 min_position) 
{
    if (*position >= max_position) 
    {
        *position = max_position;  // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Öµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Öµ
    } 
    else if (*position <= min_position) 
    {
        *position = min_position;  // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ğ¡Öµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ğ¡Öµ
    }
}
  
  
