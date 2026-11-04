#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include "struct_typedef.h"
#include "can_bsp.h"
#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPEED_MODE		0x200

#define P_MIN_DM -12.5f
#define P_MAX_DM 12.5f
#define V_MIN_DM -30.0f
#define V_MAX_DM 30.0f
#ifndef KP_MIN
#define KP_MIN 0.0f
#endif
#ifndef KP_MAX
#define KP_MAX 500.0f
#endif
#ifndef KD_MIN
#define KD_MIN 0.0f
#endif
#ifndef KD_MAX
#define KD_MAX 5.0f
#endif
#define T_MIN_DM -10.0f
#define T_MAX_DM 10.0f

#define P_MIN2 -12.5f
#define P_MAX2 12.5f
#define V_MIN2 -45.0f
#define V_MAX2 45.0f
#define KP_MIN2 0.0f
#define KP_MAX2 500.0f
#define KD_MIN2 0.0f
#define KD_MAX2 5.0f
#define T_MIN2 -10.0f
#define T_MAX2 10.0f

typedef struct 
{
	uint16_t id;
	uint16_t state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos;
	float vel;
	float tor;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
}motor_fbpara_t;
typedef struct
{
	uint16_t mode;
	motor_fbpara_t para;
}Joint_Motor_t ;

extern void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
extern void mit_ctrl_range(hcan_t* hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq, float p_min, float p_max);
extern void pos_speed_ctrl(hcan_t* hcan,uint16_t motor_id, float pos, float vel);
extern void speed_ctrl(hcan_t* hcan,uint16_t motor_id, float _vel);
extern void save_motor_zero(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
extern void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
extern void unenable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
extern void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
extern float Hex_To_Float(uint32_t *Byte,int num);//十六进制到浮点数
extern uint32_t FloatTohex(float HEX);//浮点数到十六进制转换

extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
extern int float_to_uint(float x_float, float x_min, float x_max, int bits);

extern void save_motor_zero_all_leg(void);
extern void save_motor_zero(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);


#endif
