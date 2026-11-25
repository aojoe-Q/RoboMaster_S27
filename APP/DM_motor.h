/*
 * 文件：DM_motor.h
 * 达妙（DM）电机 CAN 协议层声明：模式号、量程宏、反馈结构体、控制接口。
 *
 * 协议要点：
 *   - 控制帧 ID = 电机 ID + 模式号：MIT=0x000、位置速度=0x100、速度=0x200；
 *   - MIT 帧 8 字节：16 位 pos、12 位 vel、12 位 kp、12 位 kd、12 位 torque；
 *   - 使能/失能/存零点：数据 FF FF FF FF FF FF FF FC/FD/FE，ID 为电机 ID；
 *   - 反馈帧 D0 低 4 位为 ID、高 4 位为错误码（1=已使能，8..14=故障）。
 *
 * 量程说明：本文件里的 P_MIN_DM/V_MIN_DM/T_MIN_DM（以及 *2 后缀的宏）
 * 当前全工程未引用；MIT 打包与反馈解码实际使用 can_bsp.h 的
 * P_MIN..T_MAX（P ±12.5 rad、V ±45 rad/s、T ±54 N·m）。
 * KP/KD 宏带 #ifndef，与 can_bsp.h 的同名宏兼容（同值）。
 *
 * 注意：编码依赖定长位域与量程，修改任何 *_MIN 与 *_MAX 都要同步核对
 * DM_motor.c 的打包/解包以及电机实际能力，否则会出现方向反转或饱和。
 */
#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include "struct_typedef.h"
#include "can_bsp.h"
/* 模式号：实际发送 ID = 电机 ID + 这里的偏移；MIT 模式的偏移为 0。 */
#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPEED_MODE		0x200

/* 旧型号量程（*_DM 后缀）：当前未被任何代码引用。
 * 注意 T_MIN_DM/T_MAX_DM = ±10 N·m 与 can_bsp.h 的 T_MIN/T_MAX = ±54 不同，
 * 真正生效的是 can_bsp.h 那一组。 */
#define P_MIN_DM -12.5f
#define P_MAX_DM 12.5f
#define V_MIN_DM -30.0f
#define V_MAX_DM 30.0f
/* KP/KD 量程带 #ifndef 保护：can_bsp.h 先包含时沿用其定义（同值），
 * 只在本头文件被单独包含时兜底，避免重复定义告警。 */
#ifndef KP_MIN
#define KP_MIN 0.0f
#endif
#ifndef KP_MAX
#define KP_MAX 700.0f
#endif
#ifndef KD_MIN
#define KD_MIN 0.0f
#endif
#ifndef KD_MAX
#define KD_MAX 10.0f
#endif
#define T_MIN_DM -10.0f
#define T_MAX_DM 10.0f


/* motor_fbpara_t: DM 反馈解码结果。
 * id/state 来自 D0；p_int/v_int/t_int 为未解码原始值；pos/vel/tor 为物理量；
 * kp_int/kd_int/Kp/Kd 为预留字段（DM 反馈帧不携带增益）；
 * Tmos/Tcoil 为 MOS 与线圈温度（℃）。 */
typedef struct 
{
	uint16_t id; // 电机 ID（1..7）
	uint16_t state; // 状态/错误码：1=已使能，8..14=故障
	int p_int; // 位置原始值（16 位）
	int v_int; // 速度原始值（12 位）
	int t_int; // 力矩原始值（12 位）
	int kp_int; // 预留：反馈帧不携带 kp
	int kd_int; // 预留：反馈帧不携带 kd
	float pos; // 位置（rad）
	float vel; // 速度（rad/s）
	float tor; // 力矩（N·m）
	float Kp; // 预留
	float Kd; // 预留
	float Tmos; // MOS 温度（℃）
	float Tcoil; // 线圈温度（℃）
}motor_fbpara_t;
/* Joint_Motor_t: 一个关节的“当前模式 + 最近一次反馈”。
 * dm8009_fbdata() 只填 para；mode 由上层按控制模式自行维护。 */
typedef struct
{
	uint16_t mode;
	motor_fbpara_t para;
}Joint_Motor_t ;

/* 控制帧接口（实现在 DM_motor.c）：
 *   mit_ctrl        MIT 模式：pos/vel/kp/kd/torq 打包为 8 字节；
 *   pos_speed_ctrl  位置+速度模式：两个 float 原样发送（8 字节）；
 *   speed_ctrl      速度模式：一个 float（4 字节）；
 *   save_motor_zero / enable_motor_mode / unenable_motor_mode：
 *                   固定命令帧，末字节分别为 FE/FC/FD。 */
extern void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
extern void pos_speed_ctrl(hcan_t* hcan,uint16_t motor_id, float pos, float vel);
extern void speed_ctrl(hcan_t* hcan,uint16_t motor_id, float _vel);
extern void save_motor_zero(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
extern void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
extern void unenable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
extern void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id); // 仅声明：当前工程源码中未找到定义与调用
extern float Hex_To_Float(uint32_t *Byte,int num);//十六进制到浮点数
extern uint32_t FloatTohex(float HEX);//浮点数到十六进制转换

/* uint_to_float: 通用整数->物理量映射，实现在 app_can.c；
 * dm8009_fbdata() 用的是它，而不是 DM_motor.c 里的 uint_to_float_DM。 */
extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
#include "motor_encoding.h" // 末尾包含：旧模块只包含 DM_motor.h 时也能拿到 motor_encoding.h 的声明

/* save_motor_zero_all_leg: 一次性给 4 个电机存零点（FDCAN1 0x01/0x02、
 * FDCAN2 0x03/0x04），仅标定用；实现见 DM_motor.c。 */
extern void save_motor_zero_all_leg(void);
extern void save_motor_zero(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id); // 与上面的 save_motor_zero 重复声明，原型一致，C 语言允许


#endif
