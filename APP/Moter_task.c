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
#include "multi_turn_restore.h"
#include "math.h"

  extern FDCAN_HandleTypeDef hfdcan1;
  extern FDCAN_HandleTypeDef hfdcan2;
  extern FDCAN_HandleTypeDef hfdcan3;
  
  extern DT7_RC_ctrl_t DT7_rc_ctrl;    //DT7

/* 最终输出门控：原设计为每帧发送前重新检查、不通过时 7 轴统一发 Kp=Kd=力矩=0；
   现已按联调要求停用（Moter_Send 里保留被动监视），下面的状态量仍可 watch。 */
uint8_t  Moter_Gate_State;      /* 最近一次检查的原因位，0 = 放行 */
uint8_t  Moter_Gate_Latched;    /* 有力输出被中断后锁存，需先经过全零增益命令 */
uint32_t Moter_Gate_Blocked;    /* 被拦截的帧数 */

static ArmCommand_t moter_cmd;   /* 本批次使用的整帧命令 */
static float   moter_p_cmd[7];   /* 协议坐标 p_cmd = P_target - b */
static uint8_t moter_cmd_ok;     /* 本批次 7 轴数值全部有效 */

/* 每批开始取一次整帧命令，不逐轴拼读 ArmData */
static void Moter_Command_Snapshot(void)
{
	uint8_t powered = 0;

	taskENTER_CRITICAL();
	moter_cmd = ArmCommand;
	taskEXIT_CRITICAL();

	moter_cmd_ok = 1;
	for (uint8_t i = 0; i < 7; i++)
	{
		moter_p_cmd[i] = moter_cmd.position[i] - Joint_Boot_Offset[i];
		if (!isfinite(moter_p_cmd[i]) || moter_p_cmd[i] < Joint_P_MIN[i] || moter_p_cmd[i] > Joint_P_MAX[i] ||
		    !(moter_cmd.kp[i] >= KP_MIN && moter_cmd.kp[i] <= KP_MAX) ||
		    !(moter_cmd.kd[i] >= KD_MIN && moter_cmd.kd[i] <= KD_MAX) ||
		    !(moter_cmd.torque[i] >= T_MIN && moter_cmd.torque[i] <= T_MAX))
		{
			moter_cmd_ok = 0;   /* 任一轴越界或非有限数：整组拒绝，不靠打包回绕 */
		}
		if (moter_cmd.kp[i] != 0.0f || moter_cmd.kd[i] != 0.0f || moter_cmd.torque[i] != 0.0f)
		{
			powered = 1;
		}
	}
	/* 门控停用后必须在打包前钳位：DM_motor 的 float_to_uint 不做范围检查，越界会回绕成乱帧 */
	for (uint8_t i = 0; i < 7; i++)
	{
		if (!isfinite(moter_p_cmd[i])) moter_p_cmd[i] = 0.0f;
		if (moter_p_cmd[i] < Joint_P_MIN[i]) moter_p_cmd[i] = Joint_P_MIN[i];
		if (moter_p_cmd[i] > Joint_P_MAX[i]) moter_p_cmd[i] = Joint_P_MAX[i];

		if (!isfinite(moter_cmd.kp[i])) moter_cmd.kp[i] = 0.0f;
		if (moter_cmd.kp[i] < KP_MIN) moter_cmd.kp[i] = KP_MIN;
		if (moter_cmd.kp[i] > KP_MAX) moter_cmd.kp[i] = KP_MAX;

		if (!isfinite(moter_cmd.kd[i])) moter_cmd.kd[i] = 0.0f;
		if (moter_cmd.kd[i] < KD_MIN) moter_cmd.kd[i] = KD_MIN;
		if (moter_cmd.kd[i] > KD_MAX) moter_cmd.kd[i] = KD_MAX;

		if (!isfinite(moter_cmd.torque[i])) moter_cmd.torque[i] = 0.0f;
		if (moter_cmd.torque[i] < T_MIN) moter_cmd.torque[i] = T_MIN;
		if (moter_cmd.torque[i] > T_MAX) moter_cmd.torque[i] = T_MAX;
	}

	/* 只有全零增益、全零力矩的新命令（无力模式已把斜坡同步到当前位置）才能解除锁存 */
	/* 锁存逻辑保留，仅在门控重新启用时才有意义 */
	if (Moter_Gate_Latched && !powered && moter_cmd_ok && moter_cmd.seq != 0U)
	{
		Moter_Gate_Latched = 0;
	}
}

static uint8_t Moter_Output_Allowed(void)
{
	uint32_t now = xTaskGetTickCount();
	uint8_t reason = 0;

	if (moter_cmd.seq == 0U || (now - moter_cmd.tick) > ARM_CMD_TIMEOUT_MS)
	{
		reason |= 0x01;     /* 命令未发布或过期 */
	}
	if (APP_Joint_Fresh_Mask(now) != JOINT_ALL_MASK)
	{
		reason |= 0x02;     /* 任一轴反馈过期或报故障 */
	}
	if (!MT_Restore_Output_Allowed())
	{
		reason |= 0x04;     /* 逻辑坐标未就绪 */
	}
	if (!moter_cmd_ok)
	{
		reason |= 0x08;     /* 命令数值无效 */
	}
	if (Moter_Gate_Latched)
	{
		reason |= 0x10;
	}
	Moter_Gate_State = reason;
	return (uint8_t)(reason == 0);
}

static void Moter_Send(uint16_t motor_id, uint8_t i)
{
	/* 输出门控已按联调要求停用：这里只做被动监视，帧一律照发；
	   原来的拦截分支整段注释在下面，要恢复时把 // 去掉即可。 */
	if (!Moter_Output_Allowed())
	{
		Moter_Gate_Blocked++;   /* 只统计被拦的帧数，不阻断输出 */
	}

	mit_ctrl_range(&hfdcan1, motor_id, moter_p_cmd[i], 0, moter_cmd.kp[i], moter_cmd.kd[i], moter_cmd.torque[i], Joint_P_MIN[i], Joint_P_MAX[i]);

//	else
//	{
//		/* 有力命令被拦截：锁存，恢复后不能直接追旧目标 */
//		if (moter_cmd.kp[i] != 0.0f || moter_cmd.kd[i] != 0.0f || moter_cmd.torque[i] != 0.0f)
//		{
//			Moter_Gate_Latched = 1;
//		}
//		Moter_Gate_Blocked++;
//		/* 无力帧：位置用原始反馈占位（Kp=Kd=0 时不产生力），仍可让电机回反馈 */
//		float p = joint_L_motor[i].position;
//		if (!isfinite(p) || p < P_MIN || p > P_MAX)
//		{
//			p = 0.0f;
//		}
//		mit_ctrl(&hfdcan1, motor_id, p, 0, 0, 0, 0);
//	}
}

/*实时控制电机任务*/
void Moter_Task(void const * argument)   
{
	osDelay(100);
	while(1)
	{
	    /* Arm_L moter*/
		Moter_Command_Snapshot();
		Moter_Send(0x01, 0);
		osDelay(2);
		Moter_Send(0x02, 1);
		osDelay(2);
		Moter_Send(0x03, 2);
		osDelay(2);
		Moter_Send(0x04, 3);
		osDelay(2);
		Moter_Send(0x05, 4);
		osDelay(2);
		Moter_Send(0x06, 5);
		osDelay(2);
		/* 夹爪：右上左中力矩夹取已由 Arm_Command_Publish 写入命令帧，同样经过门控 */
		Moter_Send(0x07, 6);  //夹爪
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
