/*
 * 模块说明：DM_motor.c —— 达妙（DM）电机 CAN 应用层
 * ------------------------------------------------------------------
 * 负责：
 *   1) 控制帧打包：MIT（mit_ctrl）、位置+速度（pos_speed_ctrl）、速度（speed_ctrl）；
 *   2) 固定命令帧：使能 0xFC、失能 0xFD、存零点 0xFE（前 7 字节均为 0xFF）；
 *   3) 反馈解析：dm8009_fbdata 解 8 字节反馈（当前无调用者，实际解码在 app_can.c）；
 *   4) 浮点与定长无符号整数的量程映射（float_to_uint_clamped / uint_to_float_DM）。
 *
 * 总线与调用者：
 *   - 发送统一走 canx_send_data()（app_can.c）；关节电机挂在 FDCAN1，
 *     classic CAN、1 Mbit/s、标准 11 位 ID；
 *   - mit_ctrl() 由 Moter_task.c 的 Arm_Output_Step() 周期调用，
 *     每关节每 4 ms 最多一帧（上限 250 Hz）；
 *   - 本文件不注册接收回调，反馈帧由 CAN 接收路径解析。
 *
 * 关键约束：
 *   - 控制帧一律 8 字节；ID = 电机 ID + 模式号（MIT 0x000 / POS 0x100 / SPEED 0x200）；
 *   - MIT 量程取 can_bsp.h 的 P/V/KP/KD/T_MIN..MAX（当前 P ±12.5 rad、
 *     V ±45 rad/s、T ±54 N·m）；DM_motor.h 里的 *_DM 宏（V ±30、T ±10）是历史值，
 *     未被任何代码引用；
 *   - 编码前必须过滤 NaN/Inf，否则移位打包会得到无意义帧；float_to_uint_clamped
 *     另外做双向钳位，越界不会回绕成反向量程；
 *   - float 按内存布局原样发送（本机小端），位置速度/速度模式不做量程检查。
 */
#include <math.h>
#include <string.h>
#include "motor_encoding.h"
/* 全局诊断计数：mit_ctrl 因 NaN/Inf 丢弃的帧数，按关节索引 0..6 累加。
 * 定义在 include 之间是历史写法；其他文件通过 motor_encoding.h 的 extern 访问。 */
volatile uint32_t motor_tx_invalid[7];
#include "DM_motor.h"
#include "can_bsp.h"
#include "fdcan.h"
#include "struct_typedef.h"
#include "cmsis_os.h"
//static CAN_TxHeaderTypeDef  chassis_tx_message;
/* 三路 FDCAN 句柄由 HAL/CubeMX 初始化（fdcan.c）后导出；本文件只发送，不接收。 */
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
#include "arm_math.h"

/* Hex_To_Float: 把 4 字节原始数据按 float 的内存布局还原。
 * 参数 Byte：指向 4 字节缓冲；num：保留参数，未使用（(void)num）。
 * 返回：还原出的 float。
 * 注意：本质是 memcpy，依赖本机小端字节序，不做任何字节交换。 */
float Hex_To_Float(uint32_t *Byte,int num)//十六进制到浮点数
{
	float value;
    (void)num;
    memcpy(&value, Byte, sizeof(value)); // 按内存布局复制 4 字节，不做字节交换
    return value;
}

/* FloatTohex: 取 float 的 32 位内存表示（反向操作见 Hex_To_Float）。
 * 参数 HEX：待转换的 float。
 * 返回：4 字节位模式拼成的 uint32_t。
 * 注意：用于调试打印/协议转发，同样依赖小端布局。 */
uint32_t FloatTohex(float HEX)//浮点数到十六进制转换
{
	uint32_t value;
    memcpy(&value, &HEX, sizeof(value)); // 取 float 的 32 位位模式，可用 %08X 打印
    return value;
}

/* float_to_uint_clamped: 物理量 -> 定长无符号整数（安全版 float_to_uint）。
 * 参数 x：待编码物理量；lo/hi：映射量程；bits：目标位宽（1..16）。
 * 返回：0..(1<<bits)-1；非法输入返回 0。
 * 调用者：mit_ctrl 对 pos/vel/kp/kd/torque 逐路量化。
 * 注意：旧实现对越界值直接强制转换，可能因回绕得到错误整数；本函数改为双向钳位：
 *       x<=lo 返回 0、x>=hi 返回满量程，NaN/Inf/lo>=hi/bits 越界直接返回 0。 */
/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
uint16_t float_to_uint_clamped(float x, float lo, float hi, int bits)
{
    uint32_t top;
    float scaled;
    if (!isfinite(x) || !isfinite(lo) || !isfinite(hi) || hi <= lo ||
        bits <= 0 || bits > 16) return 0U; // 非有限值/非法量程/位宽越界：返回 0，不进入移位打包
    top = (1UL << bits) - 1U; // bits 位满量程，如 12 位 = 4095
    if (x <= lo) return 0U; // 低于下界钳到 0
    if (x >= hi) return (uint16_t)top; // 高于上界钳到满量程
    scaled = ((x - lo) / (hi - lo)) * (float)top;
    return (uint16_t)(scaled >= (float)top ? top : (uint32_t)scaled); // 浮点误差可能让 scaled 略超 top，再夹一次后截断
}
/* uint_to_float_DM: 定长无符号整数 -> 物理量（DM 反馈解码公式）。
 * 参数 x_int：原始整数；x_min/x_max：物理量程；bits：位宽。
 * 返回：x_min + x_int * (x_max - x_min) / ((1<<bits)-1)。
 * 注意：本函数与 dm8009_fbdata 当前都没有调用者；实际反馈解码在
 *       app_can.c 的 APP_DecodeJoint（它调用 app_can.c 的 uint_to_float）。 */
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float_DM(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min; // 量程跨度
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset; // 1<<bits：bits ≤ 16 时仍在 int 范围内
}

/* enable_motor_mode: 发送 DM 使能帧。
 * 参数 hcan：总线句柄；motor_id：电机 ID（1..7）；mode_id：模式偏移（MIT 用 0）。
 * 返回：void（发送失败不返回错误）。
 * 调用者：初始化/使能流程；Moter_task 的握手状态机发的是等价的裸帧。
 * 注意：帧 ID = motor_id + mode_id；数据固定 8 字节，末字节 0xFC 表示使能。 */
/*使能*/
void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF,
	                   0xFF, 0xFF, 0xFF, 0xFC};
	uint16_t id = motor_id + mode_id; // 标准帧 ID = 电机 ID + 模式号

	canx_send_data(hcan, id, data, 8); // 发送结果不检查；需要重试时由调用方负责
}

/* unenable_motor_mode: 发送 DM 失能帧（末字节 0xFD）。
 * 参数同 enable_motor_mode。
 * 注意：失能后电机不再输出力矩，机械臂会因重力自由下落，
 *       调用前应确认机械臂有支撑或已进入安全停机流程。 */
/*失能*/
void unenable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF,
	                   0xFF, 0xFF, 0xFF, 0xFD};
	uint16_t id = motor_id + mode_id; // 与使能帧相同的 ID 计算方式

	canx_send_data(hcan, id, data, 8); // 失能后电机不再出力，机械臂可能因重力下落
}

/* save_motor_zero: 把电机当前机械位置保存为零点（末字节 0xFE）。
 * 参数同 enable_motor_mode；命令写入电机内部非易失存储，掉电保持。
 * 注意：运行中调用会把当前位置覆盖成零点，导致后续位置指令整体偏移，仅标定用。 */
/*保存位置零点*/
void save_motor_zero(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF,
	                   0xFF, 0xFF, 0xFF, 0xFE};
	uint16_t id = motor_id + mode_id; // 帧 ID 与使能帧一致

	canx_send_data(hcan, id, data, 8); // 单帧命令，电机收到后写入内部非易失存储
}

/* mit_ctrl: 打包并发送 MIT 模式控制帧（本工程关节电机的主力接口）。
 * 参数：hcan 总线句柄；motor_id 电机 ID（1..7）；pos 目标位置（rad）；
 *       vel 目标速度（rad/s）；kp 位置增益（N·m/rad）；kd 阻尼（N·m·s/rad）；
 *       torq 前馈力矩（N·m）。
 * 返回：void。
 * 调用者：Moter_task.c 的 Arm_Output_Step()，每关节每 4 ms 最多一帧。
 * 注意：帧 ID = motor_id + MIT_MODE(0x000)；8 字节布局为
 *       16 位 pos + 12 位 vel + 12 位 kp + 12 位 kd + 12 位 torque；
 *       量程取 can_bsp.h 的 P/V/KP/KD/T_MIN..MAX，越界由 float_to_uint_clamped 钳位；
 *       NaN/Inf 直接丢弃并累加 motor_tx_invalid[]，不会发出脏帧。 */
/**
************************************************************************
* @brief:      	mit_ctrl: MIT模式下的电机控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   kp:				位置比例系数
* @param[in]:   kd:				位置微分系数
* @param[in]:   torq:			转矩给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
************************************************************************
**/
void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;
    if (!isfinite(pos) || !isfinite(vel) || !isfinite(kp) ||
        !isfinite(kd) || !isfinite(torq)) {
        if (motor_id >= 1U && motor_id <= 7U) motor_tx_invalid[motor_id - 1U]++; // 仅 1..7 号关节纳入计数（数组下标 0..6）
        return;
    }

	pos_tmp = float_to_uint_clamped(pos,  P_MIN,  P_MAX,  16); // 位置 16 位：±12.5 rad
	vel_tmp = float_to_uint_clamped(vel,  V_MIN,  V_MAX,  12); // 速度 12 位：±45 rad/s
	kp_tmp  = float_to_uint_clamped(kp,   KP_MIN, KP_MAX, 12); // kp 12 位：0..500
	kd_tmp  = float_to_uint_clamped(kd,   KD_MIN, KD_MAX, 12); // kd 12 位：0..5
	tor_tmp = float_to_uint_clamped(torq, T_MIN,  T_MAX,  12); // 力矩 12 位：±54 N·m

	data[0] = (pos_tmp >> 8); // D0：pos[15:8]
	data[1] = pos_tmp; // D1：pos[7:0]
	data[2] = (vel_tmp >> 4); // D2：vel[11:4]
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8); // D3：vel[3:0]<<4 | kp[11:8]
	data[4] = kp_tmp; // D4：kp[7:0]
	data[5] = (kd_tmp >> 4); // D5：kd[11:4]
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8); // D6：kd[3:0]<<4 | torque[11:8]
	data[7] = tor_tmp; // D7：torque[7:0]
	
	canx_send_data(hcan, id, data, 8); // 统一发送入口：标准帧、8 字节
}
/* pos_speed_ctrl: 位置 + 速度模式控制帧（非 MIT）。
 * 参数 pos：目标位置（rad）；vel：目标速度（rad/s）；motor_id：电机 ID。
 * 返回：void。
 * 注意：帧 ID = motor_id + POS_MODE(0x100)；两个 float 按 IEEE754 小端
 *       原样发送（各 4 字节），不做量程钳位，调用者必须保证数值合理。 */
/**
************************************************************************
* @brief:      	pos_speed_ctrl: 位置速度控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   vel:			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送位置速度控制命令
************************************************************************
**/
void pos_speed_ctrl(hcan_t* hcan,uint16_t motor_id, float pos, float vel)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf;
	uint8_t data[8];
	
	id = motor_id + POS_MODE; // 帧 ID = 电机 ID + POS_MODE(0x100)
	pbuf=(uint8_t*)&pos; // 取 float 的字节视图（本机小端）
	vbuf=(uint8_t*)&vel;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	data[6] = *(vbuf+2);
	data[7] = *(vbuf+3);
	
	canx_send_data(hcan, id, data, 8); // 位置速度模式固定 8 字节
}
/* speed_ctrl: 速度模式控制帧。
 * 参数 vel：目标速度（rad/s）；motor_id：电机 ID。
 * 返回：void。
 * 注意：帧 ID = motor_id + SPEED_MODE(0x200)；只发 4 字节 float（小端），
 *       不经过 MIT 打包，也不做量程检查。 */
/**
************************************************************************
* @brief:      	speed_ctrl: 速度控制函数
* @param[in]:   hcan: 		指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   vel: 			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送速度控制命令
************************************************************************
**/
void speed_ctrl(hcan_t* hcan,uint16_t motor_id, float vel)
{
	uint16_t id;
	uint8_t *vbuf;
	uint8_t data[4];
	
	id = motor_id + SPEED_MODE; // 帧 ID = 电机 ID + SPEED_MODE(0x200)
	vbuf=(uint8_t*)&vel;
	
	data[0] = *vbuf;
	data[1] = *(vbuf+1);
	data[2] = *(vbuf+2);
	data[3] = *(vbuf+3);
	
	canx_send_data(hcan, id, data, 4); // 速度模式只有 4 字节
}



/* dm8009_fbdata: 解析 DM 电机 8 字节反馈帧到 Joint_Motor_t。
 * 参数 motor：目标结构；rx_data：反馈数据；data_len：DLC。
 * 返回：void；DLC 不是 8 时整帧忽略。
 * 调用者：当前工程中没有调用者（实际解码见 app_can.c 的 APP_DecodeJoint）。
 * 注意：D0 低 4 位 = 电机 ID，高 4 位 = 错误码（0 正常、1 已使能、
 *       8 超压、9 欠压、A 过流、B MOS 过温、C 线圈过温、D 通讯丢失、E 过载）；
 *       本函数只做帧内字段拆解，错误码原样存入 para.state，由上层判定。
 *       量程取 can_bsp.h 的 P/V/T_MIN..MAX（位置 ±12.5 rad、速度 ±45 rad/s、
 *       力矩 ±54 N·m）；函数内原有的行尾注释数字（±30、±10）是历史值。 */
/**
************************************************************************
* @brief:      	dm8009_fbdata: 获取DM4310电机反馈数据函数
* @param[in]:   motor:    指向motor_t结构的指针，包含电机相关信息和反馈数据
* @param[in]:   rx_data:  指向包含反馈数据的数组指针
* @param[in]:   data_len: 数据长度
* @retval:     	void
* @details:    	从接收到的数据中提取DM4310电机的反馈信息，包括电机ID、
*               状态、位置、速度、扭矩相关温度参数、寄存器数据等
************************************************************************
**/
void dm8009_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len)
{ 
	if(data_len==FDCAN_DLC_BYTES_8) // 只处理 8 字节反馈帧
	{//返回的数据有8个字节
	  motor->para.id = (rx_data[0])&0x0F; // D0[3:0]：电机 ID
	  motor->para.state = (rx_data[0])>>4; // D0[7:4]：状态/错误码
	  motor->para.p_int=(rx_data[1]<<8)|rx_data[2]; // D1..D2：16 位位置原始值
	  motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4); // D3 + D4[7:4]：12 位速度原始值
	  motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5]; // D4[3:0] + D5：12 位力矩原始值
	  motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
	  motor->para.vel = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12); // (-30.0,30.0)
	  motor->para.tor = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12);  // (-10.0,10.0)
	  motor->para.Tmos = (float)(rx_data[6]); // MOS 温度（℃）
	  motor->para.Tcoil =(float)(rx_data[7]); // 线圈温度（℃）
	}
}


/* save_motor_zero_all_leg: 依次给 4 个电机（0x01..0x04）发存零点命令。
 * 参数：无。返回：void。
 * 注意：FDCAN1 发 0x01/0x02，FDCAN2 发 0x03/0x04，每帧后 osDelay(5)
 *       给电机留处理时间；运行中调用会覆盖零点，仅标定/调试用。 */
void save_motor_zero_all_leg(void)
{
	save_motor_zero(&hfdcan1, 0X01, 0);
			osDelay(5);
	save_motor_zero(&hfdcan1, 0X02, 0);
			osDelay(5);
	save_motor_zero(&hfdcan2, 0X03, 0);
			osDelay(5);
	save_motor_zero(&hfdcan2, 0X04, 0);
		    osDelay(5);
}
