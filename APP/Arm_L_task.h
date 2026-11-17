/* Arm_L_task.h —— 左臂（Arm_L）任务的数据结构与接口声明
 *
 * 内容分三类：
 *   1) 标定/工艺参数：关节位置硬限位、PD 增益、零点偏置、预设位姿宏；
 *   2) 数据结构：DM_contorl_t（单关节控制量）、ArmData_t（整臂总表）、
 *      RC_contorl_t / Custom_controller_t（遥控输入缓存）、Arm_mode_t（模式枚举）；
 *   3) 对外接口：Arm_L_Task() 任务入口与各模式处理函数。
 *
 * 注意：Arm_mode_t 新增成员只能追加在末尾，已有枚举值可能被上位机依赖；
 *       本文件中部分历史注释为乱码，保持原样，仅在其旁补充说明。李沛哲是我杨奥琪的爹
 */
#ifndef __ARM_L_TASK_H
#define __ARM_L_TASK_H

#include "main.h"
#include "can_bsp.h"
#include "bsp_delay.h"
#include "VT13_rc_ctrl.h"

#define M_PI 3.14159265358979323846f // 圆周率（单精度版）：本工程用弧度计算，避免混用 double 常量
#define LEFT 0 // 左臂标志（部分机构按左右对称处理）
#define RIGHT 1 // 右臂标志李沛哲是我杨奥琪的爹
#define STOP 0 // 任务停止/启动标志的 0/1 取值
#define START 1 // 启动


/* 关节位置硬限位（rad，电机侧读数）：limitPosition() 用它把目标钳在机械可达范围内，
 */
#define J1_Max_position 2.81695309f
#define J1_Min_position -2.96054893f
#define J2_Max_position 0.0f
#define J2_Min_position -2.05669899f
#define J3_Max_position 0.0f
#define J3_Min_position -3.00016972f
#define J4_Max_position 3.00567291f
#define J4_Min_position 0.0f
#define J5_Max_position 1.40938965f
#define J5_Min_position -0.904316101f
#define J6_Max_position 2.32408657f
#define J6_Min_position -0.650879326f
#define Grip_Max_position 0.0f
#define Grip_Min_position -12.0f		 // 夹爪量程下限：负方向为闭合，数值取得很大，只作兜底


/* 各关节 MIT 模式的 PD 增益：Kp [N·m/rad]，Kd [N·m·s/rad]。
 * Arm_L_Init() 在电机使能且目标位置与实测同步完成之后才写入这些增益，
 * 避免上电瞬间目标跳变造成力矩冲击；之后由 Arm_Publish() 每拍随快照下发。
 */
#define J1_PID_KP 68.0f		//J1 KP
#define J1_PID_KD 4.0f		//J1 KD
#define J2_PID_KP 400.0f	//J2 KP
#define J2_PID_KD 5.2f		//J2 KD
#define J3_PID_KP 100.0f	//J3 KP 200
#define J3_PID_KD 3.0f		//J3 KD
#define J4_PID_KP 32.0f		//J4 KP
#define J4_PID_KD 0.5f		//J4 KD
#define J5_PID_KP 40.0f		//J5 KP
#define J5_PID_KD 0.5f		//J5 KD
#define J6_PID_KP 32.0f		//J6 KP
#define J6_PID_KD 1.0f		//J6 KD
#define Grip_PID_KP 31.0f	//Grip KP
#define Grip_PID_KD 1.0f	//Grip KD

/* 无力（ARM_POWERLESS）模式使用的 KP/KD：全 0，等效只发位置、不出力矩。 */
#define Joint_Atony_KP_KD 0.0f		
/* 各关节零点偏置（rad）：关节角 = 电机读数 - 该值。J3 的 0.1f 为标定后保留的固定偏置。 */
#define J1_zero_position 0.0f
#define J2_zero_position 0.0f
#define J3_zero_position 0.0f//判断电机正反后修改偏重
#define J4_zero_position 0.0f
#define J5_zero_position 0.0f
#define J6_zero_position 0.0f
#define grip_zero_position 0.0f

#define Grip_clamp_position 0.1f // 夹紧位置



/* 预设位姿表（rad）：左侧工位 L_* / 右侧工位 R_*，每个工位两个候选点（_1/_2）。
 * save_position 为“放置/存放”位姿，get_position 为“取物”位姿；
 * 每行的 Grip 字段是该位姿下夹爪的目标开度：0.67 约为夹紧，0.01/0 约为张开。
 */
#define L_J1_save_position_1 -2.87270164f
#define L_J2_save_position_1 0.82826561f
#define L_J3_save_position_1 0.648248634f
#define L_J4_save_position_1 -0.157740593f
#define L_J5_save_position_1 -1.09578896f
#define L_J6_save_position_1 0.803196907f7
#define L_Grip_save_position_1 0.67f // 0.67：夹紧（咬住水杯）

#define L_J1_save_position_2 -2.91139366f
#define L_J2_save_position_2 0.94166754f
#define L_J3_save_position_2 0.516672264f
#define L_J4_save_position_2 -0.0753412247f
#define L_J5_save_position_2 -0.892843246f
#define L_J6_save_position_2 0.804341316f
#define L_Grip_save_position_2 0.01f // 0.01：接近全开

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

//TAKE POSE_1
#define J1_POSE_position 0.0f
#define J2_POSE_position -0.56973362f
#define J3_POSE_position -0.309948921f
#define J4_POSE_position -0.0f
#define J5_POSE_position 0.0783929825f
#define J6_POSE_position  2.22877121f
#define Grip_POSE_position_1 -1.0f
//RESET ARM
/* 复位（ARM_RESET）模式的目标位姿：把机械臂收到收纳姿态，夹爪保持约夹紧位。 */
#define J1_reset_position 0.0f
#define J2_reset_position 0.0852594376f
#define J3_reset_position 0.151254654f	
#define J4_reset_position -0.119974136f
#define J5_reset_position -0.0f
#define J6_reset_position 0.0f
#define Grip_reset_position 0.714694023f
//TAKE Grip

/* 遥控输入缓存：RC 指向 VT13 遥控数据帧，RC_mode 缓存本拍解析出的拨杆组合。 */
typedef  struct
{
	const VT13_RC_ctrl_t *RC; // 指向 VT13 遥控数据（只读）
	uint16_t RC_mode; // 拨杆/开关组合编码（GetArmMode() 的结果缓存）
}RC_contorl_t;

/* 工作模式枚举：GetArmMode() 根据遥控拨杆（以及 PC 控制心跳）决定，
 * Arm_L_Task() 每拍据此分派到对应处理函数。新成员只能追加在末尾，
 * 避免改变已有模式的数值（PC 端按数值下发模式）。 */
typedef enum 
{
	ARM_L_MOVEMENT_J1_J4,        
	ARM_L_MOVEMENT_J5_GRIP,      
	ARM_KEY_CONTROL,             
	ARM_RETURN_INITIAL_POSTURE,  
	CHASSIS_MOVEMENT_ARM_KEEP,   
	ARM_POWERLESS, // 无力：Kp/Kd=0，不下发力矩，只保持通信
	TAKE_POSE,// 取水流程（TAKE_WATER_1）
	PASS_WATER, // 递水流程（PASS_WATER_1）
	ARM_RESET, // 复位流程（ARM_RESET_1）
	TAKE_Grip, // 夹爪取物流程（TAKE_Grip_1）
	ARM_MODE_PC_CONTROL,         /* PC (ROS 2 / MoveIt) control, appended: keeps existing values */

}Arm_mode_t;

/* 单关节控制量（DM 电机）：一次控制周期内“遥控/按键增量 -> 目标 -> 限位 -> 斜坡”
 * 的中间状态都记在这里，send_set 是最终下发值；Kp/Kd/torque 组成 MIT 帧的增益与前馈。
 * Error / Now_theta / Angle_Sum_Value 等为历史解算中间量，当前主流程已不再使用。
 */
typedef  struct
{
		fp32 Initial_Total_position; // 进入模式/上电时的位置基准，用于增量控制与限位换算
		fp32 position; // 实测位置（rad，来自电机反馈）
		fp32 send_set; // 最终下发目标位置（rad）
		fp32 Error; // 目标与实测的偏差（当前主流程未使用）
		fp32 Now_set_position; // 上一拍下发目标的快照（仅赋值，当前无读取）
		fp32 Now_theta; // 解算中间角度（当前未使用）
		fp32 Angle_Sum_Value; // 角度累计量（旧版绝对角解算，当前未使用）
		fp32 RC_Sum_Value; // 遥控模式的累计目标：在位置基准上累加遥控增量
		fp32 Key_Control_Value; // 按键/自定义控制器产生的增量
    fp32 Kp; // MIT 位置增益 Kp
    fp32 Kd; // MIT 速度增益 Kd
    fp32 torque; // MIT 前馈力矩（N·m）；夹爪堵转标零时用它施加固定力矩
}DM_contorl_t;

/* 底盘/舵机（DJ）通道的控制量：字段比 DM 少，用于非 MIT 的通道；当前机械臂主流程未使用。 */
typedef  struct
{
		fp32 Initial_Total_position; // 位置基准
		fp32 send_set; // 下发目标
		fp32 Now_set_position; // 上一拍目标快照
		fp32 Total_Angle; // 总角度
		fp32 sent_current; // 下发电流（电流环通道）
		fp32 Key_Control_Value; // 按键增量
}DJ_contorl_t;



/* 整臂数据总表：全局唯一实例 ArmData（定义在 Arm_L_task.c），Arm_L_Task() 每拍更新，
 * 各模式处理函数通过它交接状态。所有访问都在同一个任务里串行发生，无需加锁。
 */
typedef struct 
{
	uint8_t ArmMode; // 当前工作模式（Arm_mode_t）
	DM_contorl_t J1,J2,J3,J4,J5,J6,Grip; // 7 个关节控制量：J1..J6 + 夹爪（电机 ID 1..7）
	fp32 Total_Angle; // 底座累计总角度（rad，当前主流程未使用）
//	arm_set_t arm_set;  
	fp32 pitch_position; // 俯仰角（预留，当前未使用）
	fp32 roll_position; // 横滚角（预留，当前未使用）
}ArmData_t;

/* 自定义控制器（遥控扩展通道）的原始输入缓存，Custom_RC[6] 定义在遥控模块；
 * 当前 [4]/[5] 分别作为 J5/J6 的微调增量叠加到目标上。 */
typedef struct
{
	fp32 DM_CC_receive; // 浮点通道原始值（rad 增量）
	int32_t DJ_CC_receive; // 整型通道原始值
}Custom_controller_t;


extern Custom_controller_t Custom_RC[6]; // 6 路自定义控制器输入（下标 0..5）
extern	ArmData_t ArmData; // 全局机械臂数据（定义在 Arm_L_task.c）
extern volatile uint8_t Arm_L_Ready; // 初始化完成标志：1 = 已使能且目标已与实测同步

/* 夹爪找零诊断：只供 Watch 观察，结果保留到下次找零，不参与控制。 */
typedef enum {
    GRIP_ZERO_IDLE = 0,
    GRIP_ZERO_RUNNING = 1,
    GRIP_ZERO_OK = 2,
    GRIP_ZERO_ENABLE_LOST = 3,       // 找限位期间七轴就绪检查失败
    GRIP_ZERO_FEEDBACK_STALE = 4,    // 夹爪反馈超过 40 ms
    /* 原诊断值 5（位移不足）已取消，其他诊断编号保持不变。 */
    GRIP_ZERO_STALL_TIMEOUT = 6,     // 12 s 内未完成停稳确认
    GRIP_ZERO_SEND_FAILED = 7,       // 存零帧未成功入队
    GRIP_ZERO_READBACK_TIMEOUT = 8,  // 存零后未收到符合要求的零点反馈
    GRIP_ZERO_VERIFY_ENABLE_LOST = 9 // 停稳后或存零校验期间失去七轴就绪
} ArmGripZeroResult;

typedef struct {
    ArmGripZeroResult result;
    float start_position, last_position, max_travel;
    uint32_t elapsed_ms, feedback_age_ms;
    uint8_t still, zero_sent;
} ArmGripZeroDiagnostics;
extern volatile ArmGripZeroDiagnostics arm_grip_zero_diag;

/* 回零结果：0 未开始，1 回零中，2 成功，3 回零中失去就绪，4 超时，
 * 5 最终就绪检查失败，6 最终反馈过期，7 最终位置/速度超限。
 * 下标 0..6 对应 J1..J6、夹爪；失败后保留现场，不被无力模式覆盖。 */
typedef struct {
    uint32_t elapsed_ms, settle_ms;
    float error[7], velocity[7];
    uint32_t feedback_age_ms[7];
    uint8_t state[7], status[7];
    uint8_t result, ramps_done, in_window, has_new_feedback;
} ArmHomeDiagnostics;
extern volatile ArmHomeDiagnostics arm_home_diag;

/* 对外接口
 * Arm_L_Task 是 FreeRTOS 任务入口（osPriorityHigh，主循环 osDelay(8) ≈ 125 Hz），
 * 内部按 ArmData.ArmMode 分派；其余函数是各模式的具体实现，通常只被该任务调用。
 */
void Arm_L_Task();
void Arm_L_Init(void); // 一次性初始化：等电机上电、逐个使能、同步目标、写入 PD 增益
void data_updata(void); // 每拍把电机反馈与解算结果写回 ArmData
uint8_t GetArmMode(void); // 解析遥控拨杆/按键，返回目标模式

/* 各模式处理函数（按 Arm_mode_t 分派） */
/*ģʽ*/
void Arm_Key_Control_mode(void);  // ����ģʽ

void Arm_L_Movement_J1_J4(void); // 遥控：J1-J4 跟随遥控拨杆运动
void Arm_return_initial_position(void); // 回到初始姿态（当前无调用者）
void Chassis_Movement(void); // 底盘运动时保持机械臂姿态（空实现，占位）
void Arm_Powerless(void); // 无力模式：Kp/Kd=0，输出零力矩
void Arm_PC_Control(void);        // PC control mode (ROS 2 / MoveIt) // PC（ROS 2 / MoveIt）控制：目标直接赋 send_set，不做斜坡
void Arm_PC_Control_Enter(void);  // call once when entering PC mode // 进入 PC 模式时调用一次：初始化 PC 侧的运动插补状态
void TAKE_POSE_1(void); // 取水动作
void PASS_WATER_1(void); // 递水动作
//void TAKE_GRIP(void);
void ARM_RESET_1(void); // 复位动作
void TAKE_Grip_1(void); // 夹爪取物动作
void limitPosition(fp32* position, fp32 max_position, fp32 min_position);  // ��λ���� // 把 *position 钳位到 [min_position, max_position]



#endif // __ARM_L_TASK_H
