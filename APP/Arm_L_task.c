/* ============================================================================
 * Arm_L_task.c —— 机械臂（左臂）主控制任务
 * ----------------------------------------------------------------------------
 * 职责：控制链路的上层。把「遥控器 / 上位机」的上层指令，结合限位与斜坡
 *       (arm_ramp) 换算成 7 个关节的目标位置，交给输出层
 *       （Arm_Publish -> arm_output -> 输出任务）按 MIT 模式周期下发到 CAN1。
 *
 * 任务：Arm_L_Task() 由 osThread 创建，优先级 osPriorityHigh；主循环用
 *       vTaskDelayUntil(..., 8 ms) 定周期，约 125 Hz。这个 8 ms 只是“目标
 *       刷新率”，真正的 CAN 发帧由输出层按自己的 4 ms 周期完成。
 *
 * 数据流：
 *   遥控器(VT13/DT7) ─┐
 *                     ├─> GetArmMode() 选模式 ─> TAKE/PASS/RESET/遥控/PC/Powerless
 *   上位机(USB/PC)  ──┘                                      │
 *                                                            ▼
 *                        ArmData.Jx.send_set / Kp / Kd / torque（7 个关节）
 *                                                            │
 *                                     Arm_Publish() 打成快照
 *                                                            ▼
 *                        arm_output（使能状态机、反馈超时、重试）-> MIT 帧 -> CAN1
 *
 * 关节：J1..J6 + 夹爪，电机 ID 0x01..0x07（数组下标 0..6），CAN 经典帧 1 Mbit/s。
 *       角度单位统一 rad，力矩单位 N·m。
 *
 * 读代码前先认这几个成员：
 *   send_set                本拍目标位置(rad)
 *   position                实测位置(rad)，由 data_updata() 从 joint_L_motor[] 拷贝
 *   Initial_Total_position  增量控制的基准位置
 *   RC_Sum_Value            遥控累加出来的相对角度（相对零点）
 *   Key_Control_Value       按键累加出来的相对角度（旧版按键模式遗留）
 *   *_L_position_set        斜坡状态量，.out 才是最终输出
 *   Kp/Kd/torque            MIT 模式的三个参数，随每帧一起下发
 * ==========================================================================*/
#include "arm_output.h"
#include "arm_motion.h"
#include "FreeRTOS.h"
#include "task.h"
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
#include "math.h"
/**
 * @brief 锟斤拷锟斤拷锟斤拷锟脚猴拷应锟斤拷锟斤拷锟斤拷锟斤拷锟狡的宏定锟藉。
 *
 * ��������źŵľ���ֵ���ڸ�����������ֵ��dealine����������źŵ��������źţ�
 * ��������ź�Ϊ�㡣���ڷ�ֹС���ȵ����벨����ϵͳ��������Ҫ����Ӧ��
 *
 * @param input 锟斤拷锟斤拷锟脚号★拷
 * @param output ����źţ���������������Ľ����
 * @param dealine 锟斤拷锟斤拷锟斤拷值锟斤拷
 */
/* rc_deadband_limit(input, output, dealine)：摇杆去死区——|input| 不超过 dealine 时
 * output 置 0，否则原样输出；用来滤掉摇杆回中附近的小抖振。 */
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
  
/* 三路 FDCAN 句柄：本文件只用 hfdcan1（机械臂 7 个电机挂在这条总线上）。 */
  extern FDCAN_HandleTypeDef hfdcan1;
  extern FDCAN_HandleTypeDef hfdcan2;
  extern FDCAN_HandleTypeDef hfdcan3;
  
  extern DT7_RC_ctrl_t DT7_rc_ctrl;    //DT7
  extern VT13_RC_ctrl_t VT13_rc_ctrl;  //VT13

/* 7 个关节的斜坡状态量（ramp_function_source_t，定义见 user_lib.h）
 * 用法：arm_ramp_calc(&x, 每拍步长, 目标值) 之后取 x.out 作为本拍目标；
 *       各模式共用同一组状态量，所以模式切换时要同步 .out，避免从旧目标起跳。 */
  ramp_function_source_t J1_L_position_set;      //DM_4340p  
  ramp_function_source_t J2_L_position_set;	     //DM_6248p
  ramp_function_source_t J3_L_position_set;      //DM_6248p
  ramp_function_source_t J4_L_position_set;      //DM_4310P
  ramp_function_source_t J5_L_position_set;      //DM_4310P
  ramp_function_source_t J6_L_position_set;      //DM_4310P
  ramp_function_source_t grip_L_position_set;    //DM_4310
  
/* 备用的一组斜坡状态量：全文件没有任何读写，保留占位。 */
  ramp_function_source_t J1_L_top_set;      //DM_4340p  
  ramp_function_source_t J2_L_top_set;	     //DM_J6248p
  ramp_function_source_t J3_L_top_set;      //DM_8009
  ramp_function_source_t J4_L_top_set;      //DM_4310P
  ramp_function_source_t J5_L_top_set;      //DM_4310P
  ramp_function_source_t J6_L_top_set;      //DM_4310P
  ramp_function_source_t grip_L_top_set;    //DM_4310
  
/* J4 单轴按键微调用的两条斜坡：只在 Arm_L_Init 里初始化过，之后未使用。 */
  ramp_function_source_t J4_L_key2_1_set;     
  ramp_function_source_t J4_L_key2_0_set;    
  
/* ---------------- 全局运行状态量 ---------------- */
  ButtonCounter left_button_btn;  
  ButtonCounter right_button_btn;
  ArmData_t ArmData; // 全部关节的控制量与当前模式（输出层也从这里读）
  volatile uint8_t Arm_L_Ready = 0;  /* set when Arm_L_Init() has finished */
  RC_contorl_t RC_contorl; // VT13 遥控器封装；当前实际使用的是 RCData，见 GetArmMode()
  
  joint_motor_measure_t joint_L_motor[7]; // 7 个电机的反馈，下标 0..6 ↔ ID 1..7，由 CAN 接收路径写入
  
  
/* 上电默认姿态(rad)：除 J6 = 0.4335 外均为 0，用作回零/模式切换的初始基准。 */

float J1_L_initial_posture = 0.0,J2_L_initial_posture = 0.0f,
        J3_L_initial_posture = 0.0f,J4_L_initial_posture = 0.0f,
        J5_L_initial_posture = 0.0f,J6_L_initial_posture = 0.43354702f,
        grip_L_initial_posture = 0.0f;
/* 目标值中间变量：各模式函数先算到这里，限位/斜坡处理后再写回 ArmData.*.send_set。 */
  fp32 J1_send_set, J2_send_set, J3_send_set, J4_send_set, J5_send_set, J6_send_set, Grip_send_set,
       J1_set_initial_data, J2_set_initial_data, J3_set_initial_data, J4_set_initial_data, J5_set_initial_data, J6_set_initial_data, Grip_set_initial_data;
/* 初始化标志：J1..J7_init_flag 在 Arm_L_Init 的回零循环里当循环条件。 */
  fp32 grip_pawl_flag = 1,J1_J2_J3_J4_J5_J6_J7_init_flag = 1;
  fp32 slackawl_flag = 1; // 当前无引用的历史变量
  uint8_t left_button_stop_state, right_button_stop_state;
  uint32_t left_button_press_count, right_button_press_count;
  fp32 limit_data5,limit_data6; // J5/J6 自定义控制器的限位中间量（只在注释掉的旧代码里出现）
  float Init_position; // 无引用的历史变量
  float Last_position;
/* 各关节斜坡步长(rad/拍)：注意类型是 double，一拍 = 一次模式函数调用。 */
  /*锟斤拷锟截斤拷斜锟斤拷锟斤拷锟斤拷*/
  double J1_ramp=0,J2_ramp=0,J3_ramp=0,J4_ramp=0,J5_ramp=0,J6_ramp=0,Grip_ramp=0; // 每拍最大转角，各模式函数里会按需重新赋值
  
  int i=1;//锟斤拷锟皆ｏ拷
	
/* Only Arm_L_Task builds snapshots, including during startup/calibration. */
static uint8_t grip_torque_mode;
/* Arm_Publish() —— 把 ArmData 里 7 个关节的控制量打成快照交给输出层
 * 参数/返回：无
 * 调用：主循环每拍调用；初始化、堵转标零等阻塞流程里也会在中间步骤调用。
 * 说明：1) 快照结构见 arm_output.h，输出层只认 pos/kp/kd/torque 四个数组；
 *       2) 夹爪有一段“软过渡”：grip_torque_mode 打开时按 grip_blend 渐隐夹爪的
 *          Kp/Kd，并把力矩渐变到 0.2 N·m（纯力矩夹紧），关闭时反向恢复；
 *       3) 总线被禁(bus_inhibit)或启动失败时 blend 直接归零，不把坏参数发出去。
 */
static void Arm_Publish(void)
{
    Arm_Control_Snapshot cmd = {0};
    const DM_contorl_t *joint[7] = {&ArmData.J1,&ArmData.J2,&ArmData.J3,
        &ArmData.J4,&ArmData.J5,&ArmData.J6,&ArmData.Grip}; // 指针数组统一遍历 7 个关节（下标 6 是夹爪）
    static float grip_blend; // 夹爪“位置环 -> 力矩环”过渡系数 0..1
    static uint32_t last_ms; // 上一拍时间戳(ms)
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    float step = fminf((float)(now-last_ms) * 0.01f, 0.16f); /* 100 ms blend */
    last_ms = now;
    if (Arm_L_Ready && grip_torque_mode) grip_blend = fminf(1.0f, grip_blend + step); // 允许力矩模式时 blend 上升（每 ms 0.01，约 100 ms 到 1）
    else grip_blend = fmaxf(0.0f, grip_blend - step);
    if (arm_output_diag.bus_inhibit || arm_output_diag.startup_failed) grip_blend = 0.0f; // 总线被禁或启动失败：立刻清零，不带错误参数出门
    for (unsigned j=0; j<7; ++j) {
        cmd.pos[j]=joint[j]->send_set; // 快照只取 send_set/Kp/Kd/torque 四项
        cmd.kp[j]=joint[j]->Kp;
        cmd.kd[j]=joint[j]->Kd;
        cmd.torque[j]=joint[j]->torque;
    }
    if (Arm_L_Ready) {
        cmd.kp[6] *= 1.0f-grip_blend; // 夹爪 Kp 按 (1-blend) 渐隐：blend=1 时位置环完全让位给力矩
        cmd.kd[6] *= 1.0f-grip_blend;
        cmd.torque[6] = cmd.torque[6]*(1.0f-grip_blend)+0.2f*grip_blend; // 力矩从位置环输出渐变到 0.2 N·m 的恒定夹紧力
    }
    cmd.tick_ms=now; // 时间戳供输出层判断快照新鲜度
    Arm_Control_Publish(&cmd);
}

/* Arm_L_Task() —— 机械臂控制任务主循环（osPriorityHigh，8 ms 周期 ≈ 125 Hz）
 * 参数：argument 未使用（任务签名要求）
 * 流程：Arm_L_Init() 一次性初始化（含回零与夹爪标零）-> 无限循环：
 *       data_updata() 取模式与实测 -> 未就绪/总线异常时强制 ARM_POWERLESS ->
 *       模式切换沿做一次状态同步 -> switch 分派各模式函数 -> Arm_Publish() 下发 ->
 *       从 POWERLESS 退出时请求输出层重试使能 -> vTaskDelayUntil 保持周期
 * 注意：本函数里的每个延时都会推迟整条控制链路，阻塞式流程（如
 *       Arm_return_initial_position）不能直接在主循环里调用。
 */
void Arm_L_Task(void const * argument)
{
    (void)argument;
	static uint8_t prev_ArmMode = 0xFF; // 上一拍模式；0xFF 保证第一拍必定走“切换沿”分支
    static uint8_t was_powerless; // 上一拍是否已处于无力模式（只在上沿请求一次重试）
	Arm_L_Init(); // 初始化会长时间阻塞（见 Arm_L_Init），返回时 Arm_L_Ready=1
    TickType_t next = xTaskGetTickCount();
	while (1)
	{
        data_updata(); // 刷新模式与 7 个关节实测位置（临界区内整体拷贝）
        uint8_t selected_mode = ArmData.ArmMode; // 上层请求的模式；若下面被强制成 POWERLESS，靠它检测“进入”沿
        if (!Arm_L_Ready || arm_output_diag.bus_inhibit) {
            ArmData.ArmMode = ARM_POWERLESS; // 未就绪或总线被禁：无条件转无力模式（最高优先级的安全兜底）
            grip_torque_mode = 0U;
        }

		/* ��⵽��ģʽ�л����ɵ�ǰλ�ó�ʼ�� RC_Sum_Value���ر����룩����ֹλ����Ծ */
/* 模式切换沿：把各轴增量基准对齐到当前实测位置，防止切模式瞬间目标跳变。 */
		if (ArmData.ArmMode != prev_ArmMode)
		{
/* 进入遥控模式：用“实测 - 零点 - 已有叠加量”反推累加器，使本拍目标正好等于当前姿态。 */
			if (ArmData.ArmMode == ARM_L_MOVEMENT_J1_J4)
			{
				ArmData.J1.RC_Sum_Value = ArmData.J1.position - J1_zero_position;
				ArmData.J2.RC_Sum_Value = ArmData.J2.position - J2_zero_position;
				ArmData.J3.RC_Sum_Value = ArmData.J3.position - J3_zero_position;
				ArmData.J4.RC_Sum_Value = ArmData.J4.position - J4_zero_position;
				ArmData.J5.RC_Sum_Value = ArmData.J5.position - J5_zero_position - Custom_RC[4].DM_CC_receive - ArmData.J5.Key_Control_Value;
				ArmData.J6.RC_Sum_Value = ArmData.J6.position - J6_zero_position - Custom_RC[5].DM_CC_receive - ArmData.J6.Key_Control_Value; // J5/J6 还要扣掉自定义控制器与按键分量，反推出的才是纯遥控累加值
				ArmData.Grip.RC_Sum_Value = ArmData.Grip.position - grip_zero_position;
			}
/* 进入 PC 模式：清插补器与保持标志，下一帧有效目标重新起算。 */
			else if (ArmData.ArmMode == ARM_MODE_PC_CONTROL)
			{
				Arm_PC_Control_Enter();
			}
			/* Leaving PC mode (or never in it): an enable from before the
			   switch must not move the arm when it comes back. */
			if (ArmData.ArmMode != ARM_MODE_PC_CONTROL)
			{
				Arm_USB_PC_Disarm(); // 离开 PC 模式时撤销上位机使能，下次进入必须重新使能
			}
			prev_ArmMode = ArmData.ArmMode;
		}

		switch (ArmData.ArmMode) // 按模式分派；未列出的模式落到 default（无力）
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

			case ARM_POWERLESS:
				Arm_Powerless();
			break;
			// Ĭ�������ֹͣ���˶�
			default:
				Arm_Powerless();
            break;
        }
        Arm_Publish();
        if (Arm_L_Ready && selected_mode == ARM_POWERLESS && !was_powerless) { // 从“上层请求 POWERLESS”进入时请求输出层重试使能（只在沿上做一次）
            Arm_Output_RequestRetry();
            was_powerless = 1U;
        }
        if (selected_mode != ARM_POWERLESS) was_powerless = 0U; // 一离开 POWERLESS 就复位，下次再进入还能再触发一次重试
        vTaskDelayUntil(&next, pdMS_TO_TICKS(8)); // 绝对延时 8 ms：扣掉本拍处理时间后补齐周期
		
	}
}
	
  

/************************* 锟斤拷始锟斤拷 *************************/
/* ======================= 夹爪堵转标零 ======================= */
/* 上电后夹爪的零点未知。用恒定力矩朝 GRIP_ZERO_DIR 方向顶（不是位置环）：
   位置连续 5 拍（约 100 ms）不变，就认为顶到机械死点，把该位置存成零点（0xFE），然后立刻撤掉力矩。
   先要求它确实动过 0.02 rad 才算数，避免一上电就被误判；若 2 s 内完全没动就放弃，不标零。
   注意：如果中途被别的物体挡住，同样会判成“顶死”并在此标零，所以标零时要保证夹爪空载。 */
/* 下面 8 个宏只影响 Arm_Grip_Stall_Zero()；改阈值/超时前先读上面的说明。 */
#define GRIP_ZERO_DIR        (+1) /* +1: push in +direction, -1: push in -direction */
#define GRIP_ZERO_TORQUE     (0.50f) /* 纯力矩顶，单位 N·m，只受这个宏控制 */
#define GRIP_ZERO_MOVE_EPS   (0.02f) /* 先确认确实动过，避免一上电就被判"顶死" */
#define GRIP_ZERO_STILL_EPS  (0.001f) /* 20 ms 内位移小于这个值算"没动" */
#define GRIP_ZERO_STILL_CNT  (5U)     /* 连续 5 拍没动 = 到死点 */
#define GRIP_ZERO_MOVE_STEPS (100U)   /* 100 拍（2 s）还不动就放弃，不标零 */
#define GRIP_ZERO_STEP_MS    (20U)
#define GRIP_ZERO_MAX_STEP   (600U)   /* 20 ms x 600 = 12 s 上限 */

/* Arm_Grip_Stall_Zero() —— 夹爪堵转找零（上电初始化时调用一次）
 * 参数/返回：无
 * 做法：Kp/Kd=0、只给恒定力矩 -> 每 20 ms 采一次反馈 -> 统计“不再移动”的连续拍数
 *       -> 达到 GRIP_ZERO_STILL_CNT 认为顶到死点 -> 发 EEPROM 存零帧(0xFE) -> 撤力矩
 * 失败处理：整个过程最多 12 s；超时、使能丢失、反馈超时、存零后读回校验不过，
 *           都会置 arm_output_diag.startup_failed，由 Arm_L_Init() 转 POWERLESS。
 * 注意：期间 Arm_Publish() 照常发布，恒定力矩通过快照的 torque 字段生效。
 */
static void Arm_Grip_Stall_Zero(void)
{
    float reference = joint_L_motor[6].position;
    float last_position = reference;
    uint32_t last_count = joint_L_motor[6].update_count;
    uint8_t moved = 0U, still = 0U, zero_confirmed = 0U;
    ArmData.Grip.Kp = ArmData.Grip.Kd = 0.0f; // 关掉位置环：否则会与恒定力矩对抗
    ArmData.Grip.send_set = reference;
    ArmData.Grip.torque = (float)GRIP_ZERO_DIR * GRIP_ZERO_TORQUE; // 恒定力矩朝死点方向顶，正负由 GRIP_ZERO_DIR 决定
    for (uint32_t n=0; n<GRIP_ZERO_MAX_STEP; ++n) {
        Arm_Publish();
        osDelay(GRIP_ZERO_STEP_MS); // 20 ms 采样节拍
        joint_motor_measure_t feedback;
/* 反馈由 CAN 接收路径更新，需进临界区整体拷贝，避免读到半更新的结构。 */
        taskENTER_CRITICAL();
        feedback = joint_L_motor[6];
        taskEXIT_CRITICAL();
        if (!Arm_Output_AllEnabled() ||
            xTaskGetTickCount()*portTICK_PERIOD_MS-feedback.update_tick > 40U) break; // 使能丢失或反馈超过 40 ms 未更新：立刻退出，不再标零
        if (feedback.update_count == last_count) continue; /* no stale stall samples */ // 没有新帧就跳过，避免用重复样本误判“没动”
        last_count = feedback.update_count;
        if (!moved) {
            if (fabsf(feedback.position-reference) > GRIP_ZERO_MOVE_EPS) { // 先确认真的动过 0.02 rad 以上
                moved=1U; last_position=feedback.position;
            } else if (n>=GRIP_ZERO_MOVE_STEPS) break; // 2 s 还没动：放弃标零
        } else {
            if (fabsf(feedback.position-last_position)<GRIP_ZERO_STILL_EPS) still++; // 位移小于 0.001 rad 记一次“没动”
            else { still=0U; last_position=feedback.position; }
            if (still>=GRIP_ZERO_STILL_CNT) break; // 连续 5 拍没动 = 顶到死点，退出
        }
    }
    ArmData.Grip.torque=0.0f; // 先撤力矩再判断结果，避免长时间顶着死点
    Arm_Publish();
    osDelay(8);
    if (moved && still>=GRIP_ZERO_STILL_CNT && Arm_Output_AllEnabled()) { // 必须“动过 + 顶死 + 仍使能”才写零点
        uint8_t zero_frame[8]={255,255,255,255,255,255,255,254}; // 0xFE = 把当前位置保存为零点（DM 电机 EEPROM 命令帧）
        uint32_t sent_ms=xTaskGetTickCount()*portTICK_PERIOD_MS;
        if (canx_send_data(&hfdcan1,7U,zero_frame,8U)==HAL_OK) { // 存零帧只发一次，之后靠读回确认
            /* EEPROM command is sent once; require a fresh zero-position reply. */
            for (unsigned n=0;n<10;++n) {
                Arm_Publish();osDelay(10);
                joint_motor_measure_t feedback;
                taskENTER_CRITICAL(); feedback=joint_L_motor[6]; taskEXIT_CRITICAL();
                if (!Arm_Output_AllEnabled()) break;
                if (feedback.update_count!=last_count &&
                    (int32_t)(feedback.update_tick-sent_ms)>0 && fabsf(feedback.position)<0.05f) { // 必须是存零之后的新帧且读数接近 0 才认可
                    zero_confirmed=1U;break;
                }
            }
        }
    }
    if (!zero_confirmed) arm_output_diag.startup_failed=1U; // 标零失败：置启动失败，Arm_L_Init 会转 POWERLESS
    ArmData.Grip.send_set=joint_L_motor[6].position; // 无论成败都把目标同步到最新实测值
    Arm_Publish();
}

/* Arm_L_Init() —— 机械臂上电初始化（只在任务启动时调用一次）
 * 流程：先空发布几次 -> 等 950 ms 让电机上电稳定 -> 申请输出层使能
 *       -> 等全部电机反馈（最多 1.5 s，超时置 startup_failed 并转 POWERLESS）
 *       -> 用实测位置填 position / send_set / Initial_Total_position
 *       -> Arm_Grip_Stall_Zero() 夹爪堵转标零
 *       -> 斜坡状态量对齐实测位置，再写入各关节 PD 增益
 *       -> do-while 以 0.0038/0.0035 rad 每拍把各关节收回 zero_position，
 *          每拍发布一次并 osDelay(5)，最多 20 s；超时同样转 POWERLESS
 *       -> 清空累加器、斜坡状态量重置到零点、置 Arm_L_Ready=1
 * 注意：全程阻塞调用者（可能十几秒），此时主循环尚未开始。
 */
void Arm_L_Init(void)
{
    Arm_Publish();
    osDelay(950); /* motor power-on settling window */
    button_init(&left_button_btn); // 按键对象初始化（历史遗留，当前无人读取计数）
    button_init(&right_button_btn);
    Arm_Publish();
    Arm_Output_Start(); // 让输出层开始逐电机发使能帧（使能间隔在输出层/输出任务里）
    uint32_t start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    while (!Arm_Output_AllEnabled() && // 等 7 个电机全部使能成功，最多 1.5 s
           xTaskGetTickCount()*portTICK_PERIOD_MS - start_ms < 1500U) {
        Arm_Publish();
        osDelay(8);
    }
    if (!Arm_Output_AllEnabled()) {
        arm_output_diag.startup_failed = 1U; // 启动失败：置标志、转无力、直接返回（需要复位重试）
        ArmData.ArmMode = ARM_POWERLESS;
        Arm_Powerless();
        Arm_Publish();
        return; /* reset required; no homing on partial feedback */
    }
/* 实测位置：CAN 反馈的电机角度(rad)，来自 joint_L_motor[]。 */
	ArmData.J1.position=joint_L_motor[0].position;
	ArmData.J2.position=joint_L_motor[1].position;
	ArmData.J3.position=joint_L_motor[2].position;
	ArmData.J4.position=joint_L_motor[3].position;
	ArmData.J5.position=joint_L_motor[4].position;
	ArmData.J6.position=joint_L_motor[5].position;
	ArmData.Grip.position=joint_L_motor[6].position;

/* 目标位置先同步为实测：保证使能后第一帧不产生位置阶跃。 */
	ArmData.J1.send_set=joint_L_motor[0].position;
	ArmData.J2.send_set=joint_L_motor[1].position;
	ArmData.J3.send_set=joint_L_motor[2].position;
	ArmData.J4.send_set=joint_L_motor[3].position;
	ArmData.J5.send_set=joint_L_motor[4].position;
	ArmData.J6.send_set=joint_L_motor[5].position;
	ArmData.Grip.send_set=joint_L_motor[6].position;

/* 增量控制基准也同步为实测：后续 RC_Sum_Value 都相对这里。 */
	ArmData.J1.Initial_Total_position=ArmData.J1.position;
	ArmData.J2.Initial_Total_position=ArmData.J2.position;	
	ArmData.J3.Initial_Total_position=ArmData.J3.position;
	ArmData.J4.Initial_Total_position=ArmData.J4.position;	
	ArmData.J5.Initial_Total_position=ArmData.J5.position;
	ArmData.J6.Initial_Total_position=ArmData.J6.position;	 
	ArmData.Grip.Initial_Total_position=ArmData.Grip.position;	
	
//	RC_contorl.RC = get_RC_measure_point();  //VT13_rc_ctrl
	


	/* 夹爪：堵转找机械死点 -> 存零点 -> 撤力矩；再把初始位置同步成零点后的读数，
	   这样下面的回零斜坡对夹爪是原地不动，不会顶着死点硬挤。 */
	Arm_Grip_Stall_Zero();
    if (arm_output_diag.startup_failed || !Arm_Output_AllEnabled()) {
        arm_output_diag.startup_failed = 1U;
        Arm_Powerless();
        Arm_Publish();
        return;
    }
	ArmData.Grip.position = joint_L_motor[6].position;
	ArmData.Grip.send_set = ArmData.Grip.position;
	ArmData.Grip.Initial_Total_position = ArmData.Grip.position;
/* 斜坡状态量起点 = 当前实测位置；frame_period=1 表示步长参数就是每拍增量。 */
	arm_data_ramp_init(&J1_L_position_set,1,ArmData.J1.Initial_Total_position);	
	arm_data_ramp_init(&J2_L_position_set,1,ArmData.J2.Initial_Total_position); 
	arm_data_ramp_init(&J3_L_position_set,1,ArmData.J3.Initial_Total_position);	
	arm_data_ramp_init(&J4_L_position_set,1,ArmData.J4.Initial_Total_position); 
	arm_data_ramp_init(&J5_L_position_set,1,ArmData.J5.Initial_Total_position);	
	arm_data_ramp_init(&J6_L_position_set,1,ArmData.J6.Initial_Total_position);
	arm_data_ramp_init(&grip_L_position_set,1,ArmData.Grip.Initial_Total_position);
	
/* J4 按键微调的两条斜坡一并初始化（当前无其他引用）。 */
	arm_data_ramp_init(&J4_L_key2_1_set,1,ArmData.J4.Initial_Total_position);
	arm_data_ramp_init(&J4_L_key2_0_set,1,ArmData.J4.Initial_Total_position);
	
/* 关键顺序：目标位置必须先同步成实测，再给 Kp/Kd；否则使能瞬间位置环会猛拉。 */
	ArmData.J1.Kp = J1_PID_KP;ArmData.J1.Kd = J1_PID_KD;
	ArmData.J2.Kp = J2_PID_KP;ArmData.J2.Kd = J2_PID_KD;
	ArmData.J3.Kp = J3_PID_KP;ArmData.J3.Kd = J3_PID_KD;
	ArmData.J4.Kp = J4_PID_KP;ArmData.J4.Kd = J4_PID_KD;
	ArmData.J5.Kp = J5_PID_KP;ArmData.J5.Kd = J5_PID_KD;
	ArmData.J6.Kp = J6_PID_KP;ArmData.J6.Kd = J6_PID_KD;
	ArmData.Grip.Kp = Grip_PID_KP;ArmData.Grip.Kd = Grip_PID_KD;

uint32_t home_start = xTaskGetTickCount()*portTICK_PERIOD_MS; // 回零超时计时起点（上限 20 s）
do{
    if (!Arm_Output_AllEnabled() || xTaskGetTickCount()*portTICK_PERIOD_MS-home_start > 20000U) { // 任一拍失能或超时：中止回零，转无力模式
        arm_output_diag.startup_failed = 1U;
        Arm_Powerless();
        Arm_Publish();
        return;
    }
/* 每拍朝 zero_position 走固定步长：0.0038/0.0035 rad/拍（5 ms 一拍）。 */
	arm_ramp_calc(&J1_L_position_set,0.0038,J1_zero_position);
	arm_ramp_calc(&J2_L_position_set,0.0035,J2_zero_position);
	arm_ramp_calc(&J3_L_position_set,0.0035,J3_zero_position);
	arm_ramp_calc(&J4_L_position_set,0.0035,J4_zero_position);
	arm_ramp_calc(&J5_L_position_set,0.0035,J5_zero_position);
	arm_ramp_calc(&J6_L_position_set,0.0035,J6_zero_position);
	arm_ramp_calc(&grip_L_position_set,0.0035,grip_zero_position);
	
	ArmData.J1.send_set = J1_L_position_set.out; // 斜坡输出即本拍目标
	ArmData.J2.send_set = J2_L_position_set.out;
	ArmData.J3.send_set = J3_L_position_set.out;
	ArmData.J4.send_set = J4_L_position_set.out;
	ArmData.J5.send_set = J5_L_position_set.out;
	ArmData.J6.send_set = J6_L_position_set.out;
	ArmData.Grip.send_set = grip_L_position_set.out;
	
	limitPosition(&ArmData.J1.send_set,J1_Max_position,J1_Min_position); // 回零路径也要过一遍硬限位，保证不超程
	limitPosition(&ArmData.J2.send_set,J2_Max_position,J2_Min_position);
	limitPosition(&ArmData.J3.send_set,J3_Max_position,J3_Min_position);
	limitPosition(&ArmData.J4.send_set,J4_Max_position,J4_Min_position);
	limitPosition(&ArmData.J5.send_set,J5_Max_position,J5_Min_position);
	limitPosition(&ArmData.J6.send_set,J6_Max_position,J6_Min_position);
	limitPosition(&ArmData.Grip.send_set,Grip_Max_position,Grip_Min_position);

	//�ж��Ƿ�ص���ʼ��̬
/* 7 轴斜坡输出全部等于零点时结束回零循环。 */
	if(J1_L_position_set.out == J1_zero_position && J2_L_position_set.out == J2_zero_position && J3_L_position_set.out == J3_zero_position && J4_L_position_set.out == J4_zero_position && J5_L_position_set.out == J5_zero_position && J6_L_position_set.out == J6_zero_position && grip_L_position_set.out == grip_zero_position)
	{
			J1_J2_J3_J4_J5_J6_J7_init_flag = 0; // 清循环标志，do-while 退出
	}
	Arm_Publish();
	osDelay(5); // 5 ms 一拍，比主循环的 8 ms 更细，回零更平滑
}while(J1_J2_J3_J4_J5_J6_J7_init_flag);


	/*锟斤拷锟斤拷锟斤拷锟斤拷*/
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

/* 回零结束：斜坡状态量重置到零点，下一次模式切换从零位起算。 */
    arm_data_ramp_init(&J1_L_position_set,1,0);	
	arm_data_ramp_init(&J2_L_position_set,1,0); 
	arm_data_ramp_init(&J3_L_position_set,1,J3_zero_position);	
	arm_data_ramp_init(&J4_L_position_set,1,0); 
	arm_data_ramp_init(&J5_L_position_set,1,0);	
	arm_data_ramp_init(&J6_L_position_set,1,0);
	arm_data_ramp_init(&grip_L_position_set,1,0);
	Arm_Publish();
	vTaskDelay(10); // 让最后一帧快照发出去再置就绪
	Arm_L_Ready = 1; // 就绪标志：主循环与输出层据此允许正常控制
}

/************************* 锟斤拷锟捷革拷锟斤拷 *************************/
/* data_updata() —— 每拍刷新“模式 + 实测位置”
 * 参数/返回：无
 * 调用：Arm_L_Task() 主循环每拍的第一件事。
 * 说明：模式由 GetArmMode() 从遥控器拨杆解析；实测位置在临界区内从
 *       joint_L_motor[] 整体拷贝到 ArmData，避免读到 CAN 接收路径写了一半的结构。
 */
void data_updata(void)
{
//	RC_contorl.RC = get_RC_measure_point();
//	left_button_stop_state = RC_contorl.RC->rc.left_button;
//	right_button_stop_state = RC_contorl.RC->rc.right_button;
//	// 锟斤拷锟铰帮拷锟斤拷锟斤拷锟斤拷锟斤拷
//    button_update(&left_button_btn, left_button_stop_state);
//	button_update(&right_button_btn, right_button_stop_state);
//	// 锟斤拷取锟斤拷锟斤拷锟斤拷锟斤拷
//    left_button_press_count = button_get_count(&left_button_btn);
//	right_button_press_count = button_get_count(&right_button_btn);
	
	ArmData.ArmMode = GetArmMode(); // 拨杆组合 -> 模式（3 拍消抖，遥控失联立即无力）
	
	taskENTER_CRITICAL(); // 临界区：反馈结构由 CAN 接收路径更新，必须整体一致地拷贝
	ArmData.J1.position = joint_L_motor[0].position;
	ArmData.J2.position = joint_L_motor[1].position;
	ArmData.J3.position = joint_L_motor[2].position;
	ArmData.J4.position = joint_L_motor[3].position;
	ArmData.J5.position = joint_L_motor[4].position;
	ArmData.J6.position = joint_L_motor[5].position;
	ArmData.Grip.position = joint_L_motor[6].position;
    taskEXIT_CRITICAL(); // 退出临界区
}

/************************* 妯″紡鍒ゆ柇 *************************/
/* GetArmMode() —— 由 VT13 遥控器两个拨杆的组合解析工作模式
 * 参数/返回：返回消抖后的稳定模式（见 Arm_mode_t）
 * 调用：data_updata() 每拍调用
 * 组合（右拨杆 + 左拨杆）：
 *   上 + 上 -> ARM_MODE_PC_CONTROL （上位机控制，覆盖原 TAKE_WATER）
 *   中 + 上 -> PASS_WATER
 *   中 + 中 -> ARM_RESET
 *   中 + 下 -> TAKE_Grip
 *   下 + 上 -> ARM_L_MOVEMENT_J1_J4（遥控手动）
 *   下 + 下 -> ARM_POWERLESS
 *   其他    -> ARM_POWERLESS
 * 消抖：模式与夹爪力矩开关各自要连续 3 拍一致才生效；遥控失联（任一拨杆数据为 0）
 *       立即生效，不消抖。
 */
uint8_t GetArmMode(void)
{
    uint8_t mode = 0;
    uint8_t right, left;
    taskENTER_CRITICAL(); // 拨杆值在临界区内取，避免与中断更新撕裂
    right=(uint8_t)RCData.rc.s[RC_Right]; left=(uint8_t)RCData.rc.s[RC_Left];
    taskEXIT_CRITICAL();

    /* 左上右上：只执行 PC 控制（覆盖原 TAKE_WATER）。PC 未使能、指令超时、
       USB 断开或 CAN 反馈丢失时，Arm_PC_Control() 保持锁存的当前位置。 */
	if (switch_is_up(right) && switch_is_up(left))
	{
			mode = ARM_MODE_PC_CONTROL;
	}
	else if (switch_is_mid(right)&&switch_is_up(left))  //left_button_press_count % 2 != 0&&RC_contorl.RC->rc.mode_sw == 0
	{
			mode = PASS_WATER;
	}
	else if (switch_is_mid(right)&&switch_is_mid(left))  //RC_contorl.RC->rc.shutter == 1  //switch_is_up(right)&&switch_is_up(left)
	{
			mode = ARM_RESET;
	}
	else if (switch_is_mid(right)&&switch_is_down(left))  //left_button_press_count % 2 != 0&&RC_contorl.RC->rc.mode_sw == 2
	{
			mode = TAKE_Grip;/*�ع��ʼ��̬*/
	}
		else if (switch_is_down(right)&&switch_is_up(left))  //left_button_press_count % 2 != 0&&RC_contorl.RC->rc.mode_sw == 2
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
	else if (switch_is_down(right)&&switch_is_down(left))  //RC_contorl.RC->rc.shutter == 1
	{
			mode = ARM_POWERLESS;
	}
	
	else
	{
			mode = ARM_POWERLESS;
	}
    static uint8_t candidate = ARM_POWERLESS, stable = ARM_POWERLESS, samples; // 3 拍一致消抖：候选模式连续 3 次相同才更新 stable
    static uint8_t grip_candidate, grip_samples;
    uint8_t grip = switch_is_up(right) && switch_is_mid(left); // 夹爪力矩模式开关：右上 + 左中 组合
    if (mode != candidate) { candidate = mode; samples = 1U; }
    else if (samples < 3U) samples++;
    if (samples >= 3U) stable = candidate;
    if (grip != grip_candidate) { grip_candidate = grip; grip_samples = 1U; }
    else if (grip_samples < 3U) grip_samples++;
    if (grip_samples >= 3U) grip_torque_mode = grip_candidate; // 同样 3 拍一致才写 grip_torque_mode（影响 Arm_Publish 的夹爪过渡）
    /* Lost remote is immediately powerless, without debounce delay. */
    if (!right || !left) { // 遥控失联：立即无力，不走消抖
        stable = ARM_POWERLESS; grip_torque_mode = 0U;
    }
    return stable;
}

/* ==================== 预设位姿模式函数 ====================
 * TAKE_Grip_1 / ARM_RESET_1 / PASS_WATER_1 / TAKE_WATER_1 结构完全一致：
 * 写 PD -> 取宏定义的预设目标 -> 设各轴斜坡步长 -> 硬限位 -> 斜坡
 * -> 写 ArmData.*.send_set。它们每拍被调用一次、目标固定，所以机械臂会以
 * 固定步长平滑收敛到预设位姿；本身不带状态机，也不阻塞。
 */
/*************************** 模式 ****************************/

/* TAKE_Grip_1() —— 预设位姿：取夹爪
 * 参数/返回：无
 * 调用：主循环 switch 的 TAKE_Grip 分支，每拍一次
 * 目标：各关节到 J*_reset_position，夹爪 Grip_send_set = 0（全张）
 * 注意：夹爪这里用的是 J6 的 PD 宏。
 */
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

/* 每拍最大转角(rad/拍)：J1 最快，其余按关节惯量给不同步长。 */
	J1_ramp= 0.0090,J2_ramp= 0.003,J3_ramp= 0.0045,J4_ramp= 0.0042,J5_ramp= 0.003,J6_ramp= 0.003,Grip_ramp= 0.030;  //锟斤拷锟斤拷锟斤拷

/* 预设位姿也要过限位：防止宏被改后越界。 */
	limitPosition(&J1_send_set,J1_Max_position,J1_Min_position);
	limitPosition(&J2_send_set,J2_Max_position,J2_Min_position);
	limitPosition(&J3_send_set,J3_Max_position,J3_Min_position);
	limitPosition(&J4_send_set,J4_Max_position,J4_Min_position);
	limitPosition(&J5_send_set,J5_Max_position,J5_Min_position);
	limitPosition(&J6_send_set,J6_Max_position,J6_Min_position);
	limitPosition(&Grip_send_set,Grip_Max_position,Grip_Min_position);

	/*斜锟铰猴拷锟斤拷*/
	arm_ramp_calc(&J1_L_position_set,J1_ramp,J1_send_set);
	arm_ramp_calc(&J2_L_position_set,J2_ramp,J2_send_set);
	arm_ramp_calc(&J3_L_position_set,J3_ramp,J3_send_set);
	arm_ramp_calc(&J4_L_position_set,J4_ramp,J4_send_set);
	arm_ramp_calc(&J5_L_position_set,J5_ramp,J5_send_set);
	arm_ramp_calc(&J6_L_position_set,J6_ramp,J6_send_set);
	arm_ramp_calc(&grip_L_position_set,Grip_ramp,Grip_send_set);

	
	ArmData.J1.send_set = J1_L_position_set.out;
	ArmData.J2.send_set = J2_L_position_set.out;
	ArmData.J3.send_set = J3_L_position_set.out;
	ArmData.J4.send_set = J4_L_position_set.out;
	ArmData.J5.send_set = J5_L_position_set.out;
	ArmData.J6.send_set = J6_L_position_set.out;
	ArmData.Grip.send_set = grip_L_position_set.out;

}

/* ARM_RESET_1() —— 预设位姿：复位
 * 参数/返回：无
 * 调用：主循环 switch 的 ARM_RESET 分支，每拍一次
 * 目标：各关节到 J*_reset_position，夹爪到 Grip_TAKE_position_1
 *       （与 TAKE_Grip_1 的唯一区别就是夹爪目标，这里夹在取物位）。
 */
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

	J1_ramp= 0.0090,J2_ramp= 0.003,J3_ramp= 0.0045,J4_ramp= 0.0042,J5_ramp= 0.003,J6_ramp= 0.003,Grip_ramp= 0.030;  //锟斤拷锟斤拷锟斤拷

	limitPosition(&J1_send_set,J1_Max_position,J1_Min_position);
	limitPosition(&J2_send_set,J2_Max_position,J2_Min_position);
	limitPosition(&J3_send_set,J3_Max_position,J3_Min_position);
	limitPosition(&J4_send_set,J4_Max_position,J4_Min_position);
	limitPosition(&J5_send_set,J5_Max_position,J5_Min_position);
	limitPosition(&J6_send_set,J6_Max_position,J6_Min_position);
	limitPosition(&Grip_send_set,Grip_Max_position,Grip_Min_position);

	arm_ramp_calc(&J1_L_position_set,J1_ramp,J1_send_set);
	arm_ramp_calc(&J2_L_position_set,J2_ramp,J2_send_set);
	arm_ramp_calc(&J3_L_position_set,J3_ramp,J3_send_set);
	arm_ramp_calc(&J4_L_position_set,J4_ramp,J4_send_set);
	arm_ramp_calc(&J5_L_position_set,J5_ramp,J5_send_set);
	arm_ramp_calc(&J6_L_position_set,J6_ramp,J6_send_set);
	arm_ramp_calc(&grip_L_position_set,Grip_ramp,Grip_send_set);

	ArmData.J1.send_set = J1_L_position_set.out;
	ArmData.J2.send_set = J2_L_position_set.out;
	ArmData.J3.send_set = J3_L_position_set.out;
	ArmData.J4.send_set = J4_L_position_set.out;
	ArmData.J5.send_set = J5_L_position_set.out;
	ArmData.J6.send_set = J6_L_position_set.out;
	ArmData.Grip.send_set = grip_L_position_set.out;

}

/* PASS_WATER_1() —— 预设位姿：递水
 * 参数/返回：无
 * 调用：主循环 switch 的 PASS_WATER 分支，每拍一次
 * 目标：各关节到 J*_TAKE_position，夹爪到 Grip_TAKE_position_1。
 */
void PASS_WATER_1(void)
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

	J1_ramp= 0.0090,J2_ramp= 0.003,J3_ramp= 0.0045,J4_ramp= 0.0042,J5_ramp= 0.003,J6_ramp= 0.003,Grip_ramp= 0.030;  

	
	limitPosition(&J1_send_set,J1_Max_position,J1_Min_position);
	limitPosition(&J2_send_set,J2_Max_position,J2_Min_position);
	limitPosition(&J3_send_set,J3_Max_position,J3_Min_position);
	limitPosition(&J4_send_set,J4_Max_position,J4_Min_position);
	limitPosition(&J5_send_set,J5_Max_position,J5_Min_position);
	limitPosition(&J6_send_set,J6_Max_position,J6_Min_position);
	limitPosition(&Grip_send_set,Grip_Max_position,Grip_Min_position);

	/*斜锟铰猴拷锟斤拷*/
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

/* TAKE_WATER_1() —— 预设位姿：取水
 * 参数/返回：无
 * 调用：主循环 switch 的 TAKE_WATER 分支，每拍一次
 * 目标：各关节到 J*_TAKE_position，夹爪 Grip_send_set = 0（全张）
 * 注意：与 PASS_WATER_1 的关节目标相同，差别只在夹爪开合；该模式已被 PC
 *       控制覆盖（双上拨杆现在进 PC 模式），保留作为回退路径。
 */
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

	J1_ramp= 0.0090,J2_ramp= 0.003,J3_ramp= 0.0045,J4_ramp= 0.0042,J5_ramp= 0.003,J6_ramp= 0.003,Grip_ramp= 0.030;  //锟斤拷锟斤拷锟斤拷

	/*��׶�һ���������ص�λ���ٱպ���צȡˮ*/

	/*锟斤拷位*/
	limitPosition(&J1_send_set,J1_Max_position,J1_Min_position);
	limitPosition(&J2_send_set,J2_Max_position,J2_Min_position);
	limitPosition(&J3_send_set,J3_Max_position,J3_Min_position);
	limitPosition(&J4_send_set,J4_Max_position,J4_Min_position);
	limitPosition(&J5_send_set,J5_Max_position,J5_Min_position);
	limitPosition(&J6_send_set,J6_Max_position,J6_Min_position);
	limitPosition(&Grip_send_set,Grip_Max_position,Grip_Min_position);

	/*斜锟铰猴拷锟斤拷*/
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


/* Arm_Powerless() —— 无力模式：撤掉所有出力，只保持通信
 * 做法：Kp/Kd 全部写 Joint_Atony_KP_KD(=0)；目标位置取当前实测位置；
 *       位置基准、斜坡状态、遥控累加器一并归零，保证切回其他模式时
 *       不会因为残留目标而突然甩臂。
 * 调用：Arm_L_Task() 在 ArmData.ArmMode == ARM_POWERLESS 时每拍调用。
 */
void Arm_Powerless(void)
{
    ArmData.Grip.torque = 0.0f;
	ArmData.J1.Kp = ArmData.J2.Kp = ArmData.J3.Kp = ArmData.J4.Kp = ArmData.J5.Kp = ArmData.J6.Kp = ArmData.Grip.Kp = ArmData.J1.Kd = ArmData.J2.Kd = ArmData.J3.Kd = ArmData.J4.Kd = ArmData.J5.Kd = ArmData.J6.Kd = ArmData.Grip.Kd = Joint_Atony_KP_KD; // Kp/Kd 全置 0：MIT 模式下等效“不出力”

	ArmData.J1.send_set=ArmData.J1.position;
	ArmData.J2.send_set=ArmData.J2.position;
	ArmData.J3.send_set=ArmData.J3.position;	
	ArmData.J4.send_set=ArmData.J4.position;
	ArmData.J5.send_set=ArmData.J5.position;
	ArmData.J6.send_set=ArmData.J6.position;
	ArmData.Grip.send_set=ArmData.Grip.position;
/* 位置基准重置为实测值：后续增量控制都相对这里累加。 */

	ArmData.J1.Initial_Total_position=ArmData.J1.position;
	ArmData.J2.Initial_Total_position=ArmData.J2.position;
	ArmData.J3.Initial_Total_position=ArmData.J3.position;
	ArmData.J4.Initial_Total_position=ArmData.J4.position;
	ArmData.J5.Initial_Total_position=ArmData.J5.position;
	ArmData.J6.Initial_Total_position=ArmData.J6.position;
	ArmData.Grip.Initial_Total_position=ArmData.Grip.position;
	
/* 斜坡状态 out 同步到实测位置：切回遥控模式时不会从旧目标起跳。 */

	J1_L_position_set.out = ArmData.J1.position;
	J2_L_position_set.out = ArmData.J2.position;
	J3_L_position_set.out = ArmData.J3.position;
	J4_L_position_set.out = ArmData.J4.position;
	J5_L_position_set.out = ArmData.J5.position;
	J6_L_position_set.out = ArmData.J6.position;
	grip_L_position_set.out = ArmData.Grip.position;
	
/* 遥控/按键累加器清零：下次进入遥控模式从当前位置重新开始累加。 */
	ArmData.J1.RC_Sum_Value = 0;
	ArmData.J2.RC_Sum_Value = 0;
	ArmData.J3.RC_Sum_Value = 0;
	ArmData.J4.RC_Sum_Value = 0;
	ArmData.J5.RC_Sum_Value = 0;
	ArmData.J6.RC_Sum_Value = 0;
	ArmData.Grip.RC_Sum_Value = 0;

}

/************************* PC 鎺у埗妯″紡 (ROS 2 / MoveIt) *************************/
/* PC 控制链路的持久状态（static，仅本文件可见）
 *   pc_motion[7]     每关节一维运动插补器（实现见 arm_motion.h）
 *   pc_last_ms       上一拍时间戳(ms)，用于计算 dt
 *   pc_motion_valid  上一拍是否拿到有效目标；0 时下一拍用实测位置重置插补器
 *   pc_holding       是否已锁存保持位置：丢包/超时后原地保持，避免掉臂
 */
static ArmMotion pc_motion[7];
static uint32_t pc_last_ms;
static uint8_t pc_motion_valid;
static uint8_t pc_holding = 0;   /* 1: 已锁存保持位置 */

/* 每次进入 PC 模式时调用：下一次没有有效目标时重新锁存当前位置 */
/* Arm_PC_Control_Enter() —— 进入 PC 模式时的一次性复位
 * 参数/返回：无
 * 调用：模式切换检测中，切入 ARM_MODE_PC_CONTROL 的那一拍调用一次。
 * 作用：清 pc_holding 与 pc_motion_valid，强制下一帧有效目标到达时用实测位置
 *       重建插补器，避免沿用进入前的旧目标。
 */
void Arm_PC_Control_Enter(void)
{
	pc_holding = 0;
    pc_motion_valid = 0U;
    pc_last_ms = xTaskGetTickCount()*portTICK_PERIOD_MS; // 记录时间基准：下一拍的 dt 从这里起算
}

/* Arm_PC_Control() —— 上位机（ROS 2 / MoveIt）控制
 * 输入：Arm_USB_Get_PC_Command() 返回目标位置 pc_target[7] 与本拍速度上限 vmax[7]
 * 流程：写 PD 增益 -> 取目标 -> 每关节限速限加速插补 -> 赋 send_set -> 静态限位
 *       -> 同步斜坡状态量
 * 注意：目标有效时不做 arm_ramp 斜坡（插补已在 Arm_Motion_Step 里做）；丢包/超时/
 *       断开时锁存进入失效时的位置并保持（安全动作），不会掉臂。
 */
void Arm_PC_Control(void)
{
	static float pc_target[7]; // 本拍 PC 目标位置（下标 0..6 ↔ 电机 ID 1..7）
	static float hold_position[7]; // 首次失效时锁存的保持位置
	uint8_t got_target;
    float vmax[7];

	/* 璁剧疆姝ｅ父 PID 澧炵泭锛堝弬鏁颁笉鍙橈級 */
	ArmData.J1.Kp = J1_PID_KP; ArmData.J1.Kd = J1_PID_KD; // 每拍重写 PD 增益：保证从其他模式切回后增益正确
	ArmData.J2.Kp = J2_PID_KP; ArmData.J2.Kd = J2_PID_KD;
	ArmData.J3.Kp = J3_PID_KP; ArmData.J3.Kd = J3_PID_KD;
	ArmData.J4.Kp = J4_PID_KP; ArmData.J4.Kd = J4_PID_KD;
	ArmData.J5.Kp = J5_PID_KP; ArmData.J5.Kd = J5_PID_KD;
	ArmData.J6.Kp = J6_PID_KP; ArmData.J6.Kd = J6_PID_KD;
	ArmData.Grip.Kp = J6_PID_KP; ArmData.Grip.Kd = J6_PID_KD; // 夹爪这里沿用 J6 的 PD 宏（Grip 自己的宏在本函数未使用）

	/* 尝试读取 PC 目标（USB 任务已检查会话、使能、超时、USB 连接、CAN 反馈） */
	got_target = Arm_USB_Get_PC_Command(pc_target, vmax); // USB 侧已做完会话/使能/超时/USB 连接/CAN 反馈检查；返回 0 表示本拍无有效目标
    uint32_t now_ms = xTaskGetTickCount()*portTICK_PERIOD_MS;
    float dt = (float)(now_ms-pc_last_ms)*0.001f; // 与上一拍的时间差（s），用于插补积分
    pc_last_ms = now_ms;
    if (got_target) { // 只有拿到有效目标才推进插补器
        const DM_contorl_t *joint[7] = {&ArmData.J1,&ArmData.J2,&ArmData.J3,
            &ArmData.J4,&ArmData.J5,&ArmData.J6,&ArmData.Grip};
        for (unsigned n=0; n<7; ++n) {
            if (!pc_motion_valid) { pc_motion[n].position=joint[n]->position; pc_motion[n].velocity=0.0f; } // 刚进 PC 模式或上次无目标：用实测位置重置插补器，速度清零
            pc_target[n] = Arm_Motion_Step(&pc_motion[n], pc_target[n], vmax[n], dt); // 逐关节插补：速度上限 vmax[n]、加速度上限 ARM_PC_AMAX_RAD_S2
        }
        pc_motion_valid=1U;
    } else pc_motion_valid=0U; // 目标无效：下一拍重新用实测位置起算

	if (got_target)
	{
		/* 目标有效：跟随 PC 指令 */
		pc_holding = 0; // 目标恢复有效：退出保持状态
		ArmData.J1.send_set = pc_target[0]; // 直接赋 send_set：插补已在上面的 Arm_Motion_Step 完成，这里不再走 arm_ramp
		ArmData.J2.send_set = pc_target[1];
		ArmData.J3.send_set = pc_target[2];
		ArmData.J4.send_set = pc_target[3];
		ArmData.J5.send_set = pc_target[4];
		ArmData.J6.send_set = pc_target[5];
		ArmData.Grip.send_set = pc_target[6];
	}
	else
	{
		/* 目标无效（超时/断开/CAN 丢失）：锁存并保持当前位置（安全动作 A） */
		if (!pc_holding)
		{
			hold_position[0] = ArmData.J1.position; // 只在失效的第一拍锁存一次，之后一直沿用这组位置
			hold_position[1] = ArmData.J2.position;
			hold_position[2] = ArmData.J3.position;
			hold_position[3] = ArmData.J4.position;
			hold_position[4] = ArmData.J5.position;
			hold_position[5] = ArmData.J6.position;
			hold_position[6] = ArmData.Grip.position;
			pc_holding = 1;
		}
		ArmData.J1.send_set = hold_position[0]; // 保持位置不变：既不跟随 PC，也不继续插补
		ArmData.J2.send_set = hold_position[1];
		ArmData.J3.send_set = hold_position[2];
		ArmData.J4.send_set = hold_position[3];
		ArmData.J5.send_set = hold_position[4];
		ArmData.J6.send_set = hold_position[5];
		ArmData.Grip.send_set = hold_position[6];
	}

	/* 标准静态限位（防御深度）。PC 模式不执行 J1/J2 碰撞区限位，
	   该限位只在遥控模式中生效。 */
	limitPosition(&ArmData.J1.send_set, J1_Max_position, J1_Min_position); // 静态限位（防御性）：PC 模式不启用 J1/J2 碰撞区收紧
	limitPosition(&ArmData.J2.send_set, J2_Max_position, J2_Min_position);
	limitPosition(&ArmData.J3.send_set, J3_Max_position, J3_Min_position);
	limitPosition(&ArmData.J4.send_set, J4_Max_position, J4_Min_position);
	limitPosition(&ArmData.J5.send_set, J5_Max_position, J5_Min_position);
	limitPosition(&ArmData.J6.send_set, J6_Max_position, J6_Min_position);
	limitPosition(&ArmData.Grip.send_set, Grip_Max_position, Grip_Min_position);

	/* 同步斜坡状态，防止切出 PC 模式后跳变 */
	J1_L_position_set.out = ArmData.J1.send_set;
	J2_L_position_set.out = ArmData.J2.send_set;
	J3_L_position_set.out = ArmData.J3.send_set;
	J4_L_position_set.out = ArmData.J4.send_set;
	J5_L_position_set.out = ArmData.J5.send_set;
	J6_L_position_set.out = ArmData.J6.send_set;
	grip_L_position_set.out = ArmData.Grip.send_set;
}

/*************************************遥锟斤拷锟斤拷锟斤拷锟斤拷*******************************************/

/* Arm_L_Movement_J1_J4() —— 遥控模式下的主力程
 * 输入：VT13 遥控器通道 0..3（先经 rc_deadband_limit 去死区）
 * 映射：ch0 -> J1/J3，ch1 -> J2/J4，ch2 -> J5/J6，ch3 -> 夹爪
 * 流程：累加遥控增量 -> 增量限位（相对基准位置）-> 叠加零点/自定义控制器
 *       -> 硬限位 -> 写 send_set
 * 注意：本模式只有夹爪走 arm_ramp 斜坡，其余轴直接跟手；J1 的限位在
 *       J2 低于碰撞阈值时会收紧（见 J1_J2_ZONE_*）。
 */
void Arm_L_Movement_J1_J4(void)
{
	int16_t channel_0,channel_1,channel_2,channel_3;
	/*锟斤拷取遥锟斤拷锟斤拷通锟斤拷锟斤拷锟诫，应锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷*/
/* 四个通道先过 ±10 的死区，避免摇杆回中抖动被累加进目标。 */
	rc_deadband_limit(RCData.rc.ch[0],channel_0,10);
	rc_deadband_limit(RCData.rc.ch[1],channel_1,10);
	rc_deadband_limit(RCData.rc.ch[2],channel_2,10);
	rc_deadband_limit(RCData.rc.ch[3],channel_3,10);
/* 每个控制周期都重写 PD 增益（数值是常量，确保从其他模式切回后仍正确）。 */
	/*锟斤拷锟斤拷锟斤拷PD锟斤拷值*/
	ArmData.J1.Kp = J1_PID_KP;ArmData.J1.Kd = J1_PID_KD;
	ArmData.J2.Kp = J2_PID_KP;ArmData.J2.Kd = J2_PID_KD;
	ArmData.J3.Kp = J3_PID_KP;ArmData.J3.Kd = J3_PID_KD;
	ArmData.J4.Kp = J4_PID_KP;ArmData.J4.Kd = J4_PID_KD;
	ArmData.J5.Kp = J5_PID_KP;ArmData.J5.Kd = J5_PID_KD;
	ArmData.J6.Kp = J6_PID_KP;ArmData.J6.Kd = J6_PID_KD;
	ArmData.Grip.Kp = Grip_PID_KP;ArmData.Grip.Kd = Grip_PID_KD;	/* 原来误用 J6 的 PD */
/* 夹爪斜坡步长 0.030 rad/拍：本模式只有夹爪限速，其余轴直接跟手。 */
	Grip_ramp = 0.030;							/* 本模式原来完全不设夹爪斜坡 */
	/*遥锟斤拷锟斤拷锟斤拷锟桔硷拷值*/                 
	ArmData.J1.RC_Sum_Value += channel_0*0.00001; // 每拍角度增量 = 通道值 × 系数；ch0 同时驱动 J1/J3，ch1 同时驱动 J2/J4，ch2 驱动 J5/J6
	ArmData.J2.RC_Sum_Value += channel_1*0.00001;
	ArmData.J3.RC_Sum_Value += channel_0*0.00001;
	ArmData.J4.RC_Sum_Value += channel_1*0.00002; // J4 的累加系数是 J2 的两倍
	ArmData.J5.RC_Sum_Value += channel_2*0.00002;
	ArmData.J6.RC_Sum_Value += channel_2*0.00002;
	ArmData.Grip.RC_Sum_Value += channel_3*0.00005;
	/* 夹爪累加器用自己的限位窗口：Grip_Max/Min 减零点（再减按键量）。
	   原来这里没有夹爪自己的钳位，全靠下面那行错用 J6 的限位，所以会被 J6 的零位卡住。 */
	limitPosition(&ArmData.Grip.RC_Sum_Value,
	              Grip_Max_position - grip_zero_position - ArmData.Grip.Key_Control_Value,
	              Grip_Min_position - grip_zero_position - ArmData.Grip.Key_Control_Value);

	if(joint_L_motor[1].position < J1_J2_ZONE_J2_MAX) // J2 实测低于碰撞阈值：J1 的下限收紧为 J1_J2_ZONE_J1_MIN
	{
		limitPosition(&ArmData.J1.RC_Sum_Value,J1_Max_position - ArmData.J1.Initial_Total_position,J1_J2_ZONE_J1_MIN - ArmData.J1.Initial_Total_position); // 增量限位：上限/下限都相对基准位置 Initial_Total_position 换算
	}	
	else
	{
		limitPosition(&ArmData.J1.RC_Sum_Value,J1_Max_position - ArmData.J1.Initial_Total_position,J1_Min_position - ArmData.J1.Initial_Total_position); // 正常情况：J1 使用完整的静态限位
	}  
	
	/*锟斤拷锟酵革拷MIT锟斤拷锟斤拷模式锟铰的猴拷锟斤拷锟斤拷位锟斤拷*/
/* 目标位置 = 零点偏置 + 遥控累加值（J5/J6 再叠加自定义控制器微调）。 */
	J1_set_initial_data = J1_zero_position + ArmData.J1.RC_Sum_Value;  //J1_zero_position + ArmData.J1.RC_Sum_Value + Custom_RC[0].DM_CC_receive;
	J2_set_initial_data = J2_zero_position + ArmData.J2.RC_Sum_Value;
	J3_set_initial_data = J3_zero_position + ArmData.J3.RC_Sum_Value;
	J4_set_initial_data = J4_zero_position + ArmData.J4.RC_Sum_Value;
	J5_set_initial_data = J5_zero_position + ArmData.J5.RC_Sum_Value + Custom_RC[4].DM_CC_receive; // J5 叠加自定义控制器通道 4 的微调
	J6_set_initial_data = J6_zero_position + ArmData.J6.RC_Sum_Value + Custom_RC[5].DM_CC_receive; // J6 叠加自定义控制器通道 5 的微调
	Grip_set_initial_data = grip_zero_position + ArmData.Grip.RC_Sum_Value;

	J1_send_set = J1_zero_position + ArmData.J1.RC_Sum_Value;    //J1_zero_position + ArmData.J1.RC_Sum_Value + Custom_RC[0].DM_CC_receive + ArmData.J1.Key_Control_Value;//��n���ؽڵ����ʼλ��+ң������������+�Զ����������������
	J2_send_set = J2_zero_position + ArmData.J2.RC_Sum_Value;
	J3_send_set = J3_zero_position + ArmData.J3.RC_Sum_Value;
	J4_send_set = J4_zero_position + ArmData.J4.RC_Sum_Value;
	J5_send_set = J5_zero_position + ArmData.J5.RC_Sum_Value + Custom_RC[4].DM_CC_receive + ArmData.J5.Key_Control_Value;
	J6_send_set = J6_zero_position + ArmData.J6.RC_Sum_Value + Custom_RC[5].DM_CC_receive + ArmData.J6.Key_Control_Value;
	Grip_send_set = grip_zero_position + ArmData.Grip.RC_Sum_Value + ArmData.Grip.Key_Control_Value;
	
	if(joint_L_motor[1].position < J1_J2_ZONE_J2_MAX) // 与前面一致：按 J2 位置决定 J1 是否收紧
	{
		limitPosition(&J1_send_set,J1_Max_position,J1_J2_ZONE_J1_MIN);
	}	
	else
	{
		limitPosition(&J1_send_set,J1_Max_position,J1_Min_position);
	}  
	/*���λ�ø�ֵ*/
/* 把本拍目标写回 ArmData，供 Arm_Publish() 组装 MIT 帧下发。 */
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
	/* 夹爪不同步斜坡：本模式夹爪要走自己的斜坡，见函数末尾 arm_ramp_calc */

//	Arm_Key_Control_mode();  //�������
	
	/*锟斤拷位*/  
/* 增量限位：把 RC_Sum_Value 限制在“基准位置 ± 行程”范围内。 */
	limitPosition(&ArmData.J1.RC_Sum_Value,J1_Max_position - ArmData.J1.Initial_Total_position,J1_Min_position - ArmData.J1.Initial_Total_position);
	limitPosition(&ArmData.J2.RC_Sum_Value,J2_Max_position - ArmData.J2.Initial_Total_position - ArmData.J2.Key_Control_Value,J2_Min_position - ArmData.J2.Initial_Total_position - ArmData.J2.Key_Control_Value);
	limitPosition(&ArmData.J3.RC_Sum_Value,J3_Max_position - ArmData.J3.Initial_Total_position,J3_Min_position - ArmData.J3.Initial_Total_position);
	limitPosition(&ArmData.J4.RC_Sum_Value,J4_Max_position - ArmData.J4.Initial_Total_position - ArmData.J4.Key_Control_Value,J4_Min_position - ArmData.J4.Initial_Total_position - ArmData.J4.Key_Control_Value);
	limitPosition(&ArmData.J5.RC_Sum_Value,J5_Max_position - ArmData.J5.Initial_Total_position,J5_Min_position - ArmData.J5.Initial_Total_position);
	limitPosition(&ArmData.J6.RC_Sum_Value,J6_Max_position - ArmData.J6.Initial_Total_position - ArmData.J6.Key_Control_Value,J6_Min_position - ArmData.J6.Initial_Total_position - ArmData.J6.Key_Control_Value);
	/* 夹爪的 RC_Sum 已经在上面用夹爪自己的限位钳过，这里不再重复 */

//				J1_send_set = R_J1_get_position_1;
//				J2_send_set = R_J2_get_position_1;
//				J3_send_set = R_J3_get_position_1;
//				J4_send_set = R_J4_get_position_1;
//				J5_send_set = R_J5_get_position_1;
//				J6_send_set = R_J6_get_position_1;
//				Grip_send_set =R_Grip_get_position_1;	
	if(joint_L_motor[1].position < J1_J2_ZONE_J2_MAX) // 第三次按 J2 位置收紧 J1（本函数里同一判断共出现三处）
	{
		limitPosition(&ArmData.J1.send_set,J1_Max_position,J1_J2_ZONE_J1_MIN);
	}	
	else
	{
		limitPosition(&ArmData.J1.send_set,J1_Max_position,J1_Min_position);
	}  
	limitPosition(&ArmData.J1.send_set,J1_Max_position,J1_Min_position); // J1 再走一次完整限位：前面收紧后的值仍在其区间内，不会被放宽
	limitPosition(&ArmData.J2.send_set,J2_Max_position,J2_Min_position);
	limitPosition(&ArmData.J3.send_set,J3_Max_position,J3_Min_position);
	limitPosition(&ArmData.J4.send_set,J4_Max_position,J4_Min_position);
	limitPosition(&ArmData.J5.send_set,J5_Max_position,J5_Min_position);
	limitPosition(&ArmData.J6.send_set,J6_Max_position,J6_Min_position);
	limitPosition(&ArmData.Grip.send_set,Grip_Max_position,Grip_Min_position);

	/* 夹爪走斜坡：本模式其它轴都是直接跟手，只有夹爪限速，避免夹伤/冲击 */
	arm_ramp_calc(&grip_L_position_set,Grip_ramp,ArmData.Grip.send_set); // 夹爪单轴斜坡：步长 Grip_ramp
	ArmData.Grip.send_set = grip_L_position_set.out;
	
	/*锟斤拷锟斤拷锟斤拷锟斤拷前位锟矫革拷锟斤拷锟斤拷锟斤拷模式*/
/* 记录本拍实际下发值（仅调试回读用）。 */
	ArmData.J1.Now_set_position = ArmData.J1.send_set;
	ArmData.J2.Now_set_position = ArmData.J2.send_set;
	ArmData.J3.Now_set_position = ArmData.J3.send_set;
	ArmData.J4.Now_set_position = ArmData.J4.send_set;
	ArmData.J5.Now_set_position = ArmData.J5.send_set;
	ArmData.J6.Now_set_position = ArmData.J6.send_set;
	ArmData.Grip.Now_set_position = ArmData.Grip.send_set;
	
}

/* Arm_return_initial_position() —— 把机械臂平滑收回零点姿态
 * 特点：内部自带 osDelay(500) 与 do-while + osDelay(5) 的阻塞循环，会长时间占用
 *       Arm_L_Task（打断 8 ms 控制周期），因此主循环中当前并未调用它。
 * 流程：目标置为实测 -> 等 500 ms -> 刷新 position/基准 -> 重置斜坡 -> 下发 PD
 *       -> 以 0.002 rad/拍向零点收敛，直到 7 轴全部到位。
 */
void Arm_return_initial_position(void)
{
	ArmData.J1.send_set=joint_L_motor[0].position;
	ArmData.J2.send_set=joint_L_motor[1].position;
	ArmData.J3.send_set=joint_L_motor[2].position;
	ArmData.J4.send_set=joint_L_motor[3].position;
	ArmData.J5.send_set=joint_L_motor[4].position;
	ArmData.J6.send_set=joint_L_motor[5].position;
	ArmData.Grip.send_set=joint_L_motor[6].position;
	osDelay(500); // 阻塞等待 500 ms，让电机先跟上目标
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

/* 以当前实测位置刷新增量控制基准。 */
	ArmData.J1.Initial_Total_position=ArmData.J1.position;
	ArmData.J2.Initial_Total_position=ArmData.J2.position;	
	ArmData.J3.Initial_Total_position=ArmData.J3.position;
	ArmData.J4.Initial_Total_position=ArmData.J4.position;	
	ArmData.J5.Initial_Total_position=ArmData.J5.position;
	ArmData.J6.Initial_Total_position=ArmData.J6.position;	 
	ArmData.Grip.Initial_Total_position=ArmData.Grip.position;	

	arm_data_ramp_init(&J1_L_position_set,1,ArmData.J1.Initial_Total_position);	 // 斜坡状态以实测位置为起点（frame_period=1，语义见 user_lib.c）
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
	
	fp32 J1_J2_J3_J4_J5_J6_J7_flag = 1; // 循环标志：1=仍在收拢，0=已全部到达零点

	do{	
		arm_ramp_calc(&J1_L_position_set,0.002,J1_zero_position); // 每拍步长 0.002 rad（约 0.4 rad/s）
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
/* 7 轴斜坡输出全部等于零点即认为到位。 */
		if(J1_L_position_set.out == J1_zero_position && J2_L_position_set.out == J2_zero_position && J3_L_position_set.out == J3_zero_position && J4_L_position_set.out == J4_zero_position && J5_L_position_set.out == J5_zero_position && J6_L_position_set.out == J6_zero_position && grip_L_position_set.out == grip_zero_position)
		{
				J1_J2_J3_J4_J5_J6_J7_flag = 0;
		}
		osDelay(5); // 5 ms 一步，动作总耗时取决于最远轴的距离
	}while(J1_J2_J3_J4_J5_J6_J7_flag);
}

/* Chassis_Movement() —— 底盘运动时保持机械臂（当前为空实现，仅占位）。 */
void Chassis_Movement(void)
{
	
}

/*  锟斤拷位锟斤拷锟斤拷  */
/* limitPosition() —— 单值硬限位钳制
 * 参数：position      待钳位目标指针（就地修改）
 *       max_position  上限(rad)
 *       min_position  下限(rad)
 * 返回：无
 * 行为：>= 上限取上限，<= 下限取下限。所有 send_set / 累加器写回 ArmData 之前
 *       都先过一遍，防止超程撞机械限位。
 */
void limitPosition(fp32* position, fp32 max_position, fp32 min_position) 
{
    if (*position >= max_position)  // 超上限：直接截断（无减速包络）
    {
        *position = max_position;  // ����������ֵ�����������ֵ
    } 
    else if (*position <= min_position)  // 超下限
    {
        *position = min_position;  // ���������Сֵ����������Сֵ
    }
}
  
  
