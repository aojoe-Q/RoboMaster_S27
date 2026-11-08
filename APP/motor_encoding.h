/* motor_encoding.h —— 电机帧数值编码与非法指令计数的公共接口
 *
 * 职责：
 *   1) float_to_uint_clamped()：把浮点物理量按 MIT 模式的线性映射压成无符号整数，
 *      供 MIT 控制帧打包；上下限由调用方按各电机实际量程传入。
 *   2) motor_tx_invalid[]：记录因 NaN/Inf、超量程等原因被拒绝发送的指令帧数。
 *
 * 边界：本文件只提供“纯函数 + 计数”，不直接访问 CAN 外设；真正发帧在
 *       Moter_task.c（输出层）完成。约定见下方英文注释：调用方必须先剔除
 *       非有限值，编码函数本身只做钳位。
 */
#ifndef MOTOR_ENCODING_H
#define MOTOR_ENCODING_H
#include <stdint.h>
/* 约定：调用方在编码前已拒绝 NaN/Inf 等非有限指令，本函数只做线性映射与钳位。 */
/* Callers reject non-finite commands before encoding any frame. */
uint16_t float_to_uint_clamped(float x, float lo, float hi, int bits); // 线性映射 x -> (x-lo)*(2^bits-1)/(hi-lo)，超出 [lo,hi] 时钳位，结果 0..2^bits-1
extern volatile uint32_t motor_tx_invalid[7]; // 每关节累计被丢弃的非法指令帧数，下标 0..6 对应电机 ID 1..7（volatile，中断/任务共享）
#endif // MOTOR_ENCODING_H
